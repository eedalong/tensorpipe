/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <cstring>

#include <future>

#include <tensorpipe/benchmark/channel_registry.h>
#include <tensorpipe/benchmark/measurements.h>
#include <tensorpipe/benchmark/options.h>
#include <tensorpipe/benchmark/transport_registry.h>
#include <tensorpipe/common/cpu_buffer.h>
#include <tensorpipe/common/defs.h>
#include <tensorpipe/core/context.h>
#include <tensorpipe/core/listener.h>
#include <tensorpipe/core/pipe.h>
#include <iostream>
#include <chrono>

using namespace tensorpipe;
using namespace tensorpipe::benchmark;

static constexpr int kNumWarmUpRounds = 0;

using Payload = std::unique_ptr<uint8_t[]>;
using CpuTensor = std::unique_ptr<uint8_t[]>;

struct Data {
  size_t numPayloads;
  size_t payloadSize;
  std::vector<Payload> expectedPayload;
  std::vector<std::string> expectedPayloadMetadata;
  std::vector<Payload> temporaryPayload;

  size_t numTensors;
  size_t tensorSize;
  TensorType tensorType;
  std::vector<CpuTensor> expectedCpuTensor;
  std::vector<std::string> expectedTensorMetadata;
  std::vector<CpuTensor> temporaryCpuTensor;
  std::string expectedMetadata;

};

struct MultiDeviceMeasurements {
  // The CPU time to do each ping-pong.
  Measurements cpu;
  // The CPU time of N iterations, including a final CUDA stream sync.
  Measurements cuda;
};

static void printMeasurements(Measurements& measurements, size_t dataLen) {
  measurements.sort();
  fprintf(
      stderr,
      "%-15s %-15s %-12s %-7s %-7s %-7s %-7s\n",
      "chunk-size",
      "# ping-pong",
      "avg (usec)",
      "p50",
      "p75",
      "p80",
      "p95");
  fprintf(
      stderr,
      "%-15lu %-15lu %-12.3f %-7.3f %-7.3f %-7.3f %-7.3f\n",
      dataLen,
      measurements.size(),
      measurements.avg_without_outlier().count() / 1000.0,
      measurements.percentile(0.50).count() / 1000.0,
      measurements.percentile(0.75).count() / 1000.0,
      measurements.percentile(0.80).count() / 1000.0,
      measurements.percentile(0.95).count() / 1000.0);
}

static void printMultiDeviceMeasurements(
    MultiDeviceMeasurements& measurements,
    size_t dataLen) {
  printMeasurements(measurements.cpu, dataLen);
}

static std::unique_ptr<uint8_t[]> createEmptyCpuData(size_t size) {
  return std::make_unique<uint8_t[]>(size);
}

static std::unique_ptr<uint8_t[]> createFullCpuData(size_t size) {
  std::unique_ptr<uint8_t[]> data = createEmptyCpuData(size);
  // Generate fixed data for validation between peers
  for (size_t i = 0; i < size; i++) {
    data[i] = (i >> 8) ^ (i & 0xff);
  }
  return data;
}

static void serverPongPingNonBlock(
    std::shared_ptr<Pipe> pipe,
    int& numWarmUps,
    int& numRoundTrips,
    std::promise<void>& doneProm,
    Data& data,
    Measurements& measurements) {

  pipe->readDescriptor(
      [pipe, &numWarmUps, &numRoundTrips, &doneProm, &data, &measurements](
          const Error& error, Descriptor descriptor) {
        TP_THROW_ASSERT_IF(error) << error.what();
        Allocation allocation;
        
        // no memory copy 
        TP_DCHECK_EQ(descriptor.metadata, data.expectedMetadata);
        if (data.payloadSize > 0) {
          TP_DCHECK_EQ(descriptor.payloads.size(), data.numPayloads);
          allocation.payloads.resize(data.numPayloads);
          for (size_t payloadIdx = 0; payloadIdx < data.numPayloads;
               payloadIdx++) {
            TP_DCHECK_EQ(
                descriptor.payloads[payloadIdx].metadata,
                data.expectedPayloadMetadata[payloadIdx]);
            TP_DCHECK_EQ(
                descriptor.payloads[payloadIdx].length, data.payloadSize);
            allocation.payloads[payloadIdx].data =
                data.temporaryPayload[payloadIdx].get();
          }
        } else {
          TP_DCHECK_EQ(descriptor.payloads.size(), 0);
        }
        if (data.tensorSize > 0) {
          TP_DCHECK_EQ(descriptor.tensors.size(), data.numTensors);
          allocation.tensors.resize(data.numTensors);
          for (size_t tensorIdx = 0; tensorIdx < data.numTensors; tensorIdx++) {
            TP_DCHECK_EQ(
                descriptor.tensors[tensorIdx].metadata,
                data.expectedTensorMetadata[tensorIdx]);
            TP_DCHECK_EQ(descriptor.tensors[tensorIdx].length, data.tensorSize);
            if (data.tensorType == TensorType::kCpu) {
              allocation.tensors[tensorIdx].buffer = CpuBuffer{
                  .ptr = data.temporaryCpuTensor[tensorIdx].get(),
              };
            } else {
              TP_THROW_ASSERT() << "Unknown tensor type";
            }
          }
        } else {
          TP_DCHECK_EQ(descriptor.tensors.size(), 0);
        }
        

        pipe->read(
            allocation,
            [pipe,
             &numWarmUps,
             &numRoundTrips,
             &doneProm,
             &data,
             &measurements,
             descriptor{std::move(descriptor)},
             allocation](const Error& error) {
              TP_THROW_ASSERT_IF(error) << error.what();

              Message message;
              
              if (data.payloadSize > 0) {
                TP_DCHECK_EQ(allocation.payloads.size(), data.numPayloads);
                message.payloads.resize(data.numPayloads);
                for (size_t payloadIdx = 0; payloadIdx < data.numPayloads;
                     payloadIdx++) {
                  TP_DCHECK_EQ(
                      descriptor.payloads[payloadIdx].length, data.payloadSize);
                  TP_DCHECK_EQ(
                      memcmp(
                          allocation.payloads[payloadIdx].data,
                          data.expectedPayload[payloadIdx].get(),
                          descriptor.payloads[payloadIdx].length),
                      0);
                  message.payloads[payloadIdx] = {
                      .data = data.expectedPayload[payloadIdx].get(),
                      .length = descriptor.payloads[payloadIdx].length,
                  };
                }
              } else {
                TP_DCHECK_EQ(allocation.payloads.size(), 0);
              }
              if (data.tensorSize > 0) {
                TP_DCHECK_EQ(allocation.tensors.size(), data.numTensors);
                message.tensors.resize(data.numTensors);
                for (size_t tensorIdx = 0; tensorIdx < data.numTensors;
                     tensorIdx++) {
                  TP_DCHECK_EQ(
                      descriptor.tensors[tensorIdx].length, data.tensorSize);
                  if (data.tensorType == TensorType::kCpu) {
                    // dont cmp memory
                    /*
                    TP_DCHECK_EQ(
                        memcmp(
                            allocation.tensors[tensorIdx]
                                .buffer.unwrap<CpuBuffer>()
                                .ptr,
                            data.expectedCpuTensor[tensorIdx].get(),
                            descriptor.tensors[tensorIdx].length),
                        0);
                    */

                  } else {
                    TP_THROW_ASSERT() << "Unknown tensor type";
                  }
                  message.tensors[tensorIdx] = {
                      .buffer = allocation.tensors[tensorIdx].buffer,
                      .length = descriptor.tensors[tensorIdx].length,
                      .targetDevice =
                          descriptor.tensors[tensorIdx].sourceDevice,
                  };
                }
              } else {
                TP_DCHECK_EQ(allocation.tensors.size(), 0);
              }
              

              /*
              pipe->write(
                  std::move(message),
                  [pipe,
                   &numWarmUps,
                   &numRoundTrips,
                   &doneProm,
                   &data,
                   &measurements](const Error& error) {
                    TP_THROW_ASSERT_IF(error) << error.what();
              */      
                if (numWarmUps > 0) {
                  numWarmUps -= 1;
                } else {
                  numRoundTrips -= 1;
                }
                if (numRoundTrips > 0) {
                  serverPongPingNonBlock(
                      pipe,
                      numWarmUps,
                      numRoundTrips,
                      doneProm,
                      data,
                      measurements);
                } else {
                  doneProm.set_value();
                }
          //        });
            });
      });
}

// Start with receiving ping
static void runServer(const Options& options) {
  std::string addr = options.address;
  int numWarmUps = kNumWarmUpRounds;
  int numRoundTrips = options.numRoundTrips;

  Data data;
  data.numPayloads = options.numPayloads;
  data.payloadSize = options.payloadSize;
  for (size_t payloadIdx = 0; payloadIdx < options.numPayloads; payloadIdx++) {
    data.expectedPayload.push_back(createFullCpuData(options.payloadSize));
    data.expectedPayloadMetadata.push_back(
        std::string(options.metadataSize, 0x42));
    data.temporaryPayload.push_back(createEmptyCpuData(options.payloadSize));
  }
  data.numTensors = options.numTensors;
  data.tensorSize = options.tensorSize;
  data.tensorType = options.tensorType;
  for (size_t tensorIdx = 0; tensorIdx < options.numTensors; tensorIdx++) {
    data.expectedTensorMetadata.push_back(
        std::string(options.metadataSize, 0x42));
    if (options.tensorType == TensorType::kCpu) {
      data.expectedCpuTensor.push_back(createFullCpuData(options.tensorSize));
      data.temporaryCpuTensor.push_back(createEmptyCpuData(options.tensorSize));
    } else {
      TP_THROW_ASSERT() << "Unknown tensor type";
    }
  }
  data.expectedMetadata = std::string(options.metadataSize, 0x42);

  Measurements measurements;
  measurements.reserve(options.numRoundTrips);

  std::shared_ptr<Context> context = std::make_shared<Context>();
  auto transportContext =
      TensorpipeTransportRegistry().create(options.transport);
  validateTransportContext(transportContext);
  context->registerTransport(0, options.transport, transportContext);

  auto channelContext = TensorpipeChannelRegistry().create(options.channel);
  validateChannelContext(channelContext);
  context->registerChannel(0, options.channel, channelContext);

  std::promise<std::shared_ptr<Pipe>> pipeProm;
  std::cout<<"start to listen on "<< addr<<std::endl;
  std::shared_ptr<Listener> listener = context->listen({addr});

  listener->accept([&](const Error& error, std::shared_ptr<Pipe> pipe) {
    TP_THROW_ASSERT_IF(error) << error.what();
    pipeProm.set_value(std::move(pipe));
  });
  std::shared_ptr<Pipe> pipe = pipeProm.get_future().get();


  std::promise<void> doneProm;
  serverPongPingNonBlock(
      std::move(pipe), numWarmUps, numRoundTrips, doneProm, data, measurements);

  doneProm.get_future().get();
  listener.reset();
  context->join();
}

static void clientPingPongNonBlock(
    std::shared_ptr<Pipe> pipe,
    int& numWarmUps,
    int& numRoundTrips,
    std::promise<void>& doneProm,
    Data& data,
    MultiDeviceMeasurements& measurements) {

  if (numWarmUps == 0) {
    measurements.cpu.markStart();
  }
  Message message;
  message.metadata = data.expectedMetadata;
  if (data.payloadSize > 0) {
    for (size_t payloadIdx = 0; payloadIdx < data.numPayloads; payloadIdx++) {
      Message::Payload payload;
      payload.data = data.expectedPayload[payloadIdx].get();
      payload.length = data.payloadSize;
      message.payloads.push_back(std::move(payload));
    }
  } else {
    TP_DCHECK_EQ(message.payloads.size(), 0);
  }
  if (data.tensorSize > 0) {
    for (size_t tensorIdx = 0; tensorIdx < data.numTensors; tensorIdx++) {
      Message::Tensor tensor;
      tensor.length = data.tensorSize;
      if (data.tensorType == TensorType::kCpu) {
        tensor.buffer =
            CpuBuffer{.ptr = data.expectedCpuTensor[tensorIdx].get()};
        tensor.targetDevice = Device(kCpuDeviceType, 0);
      } else {
        TP_THROW_ASSERT() << "Unknown tensor type";
      }
      message.tensors.push_back(std::move(tensor));
    }
  } else {
    TP_DCHECK_EQ(message.tensors.size(), 0);
  }
  std::cout << "=============================BEGIN TO TEST========================================="<<std::endl;

  pipe->write(
      std::move(message),
      [pipe, &numWarmUps, &numRoundTrips, &doneProm, &data, &measurements](
          const Error& error) {
        
        /*
        TP_THROW_ASSERT_IF(error) << error.what();
        pipe->readDescriptor([pipe,
                              &numWarmUps,
                              &numRoundTrips,
                              &doneProm,
                              &data,
                              &measurements](
                                 const Error& error, Descriptor descriptor) {
          TP_THROW_ASSERT_IF(error) << error.what();

          Allocation allocation;
          TP_DCHECK_EQ(descriptor.metadata, data.expectedMetadata);
          if (data.payloadSize > 0) {
            TP_DCHECK_EQ(descriptor.payloads.size(), data.numPayloads);
            allocation.payloads.resize(data.numPayloads);
            for (size_t payloadIdx = 0; payloadIdx < data.numPayloads;
                 payloadIdx++) {
              TP_DCHECK_EQ(
                  descriptor.payloads[payloadIdx].metadata,
                  data.expectedPayloadMetadata[payloadIdx]);
              TP_DCHECK_EQ(
                  descriptor.payloads[payloadIdx].length, data.payloadSize);
              allocation.payloads[payloadIdx].data =
                  data.temporaryPayload[payloadIdx].get();
            }
          } else {
            TP_DCHECK_EQ(descriptor.payloads.size(), 0);
          }
          if (data.tensorSize > 0) {
            TP_DCHECK_EQ(descriptor.tensors.size(), data.numTensors);
            allocation.tensors.resize(data.numTensors);
            for (size_t tensorIdx = 0; tensorIdx < data.numTensors;
                 tensorIdx++) {
              TP_DCHECK_EQ(
                  descriptor.tensors[tensorIdx].metadata,
                  data.expectedTensorMetadata[tensorIdx]);
              TP_DCHECK_EQ(
                  descriptor.tensors[tensorIdx].length, data.tensorSize);
              if (data.tensorType == TensorType::kCpu) {
                allocation.tensors[tensorIdx].buffer = CpuBuffer{
                    .ptr = data.temporaryCpuTensor[tensorIdx].get(),
                };
              }else {
                TP_THROW_ASSERT() << "Unknown tensor type";
              }
            }
          } else {
            TP_DCHECK_EQ(descriptor.tensors.size(), 0);
          }
          pipe->read(
              allocation,
              [pipe,
               &numWarmUps,
               &numRoundTrips,
               &doneProm,
               &data,
               &measurements,
               descriptor{std::move(descriptor)},
               allocation](const Error& error) {
                if (numWarmUps == 0) {
                  measurements.cpu.markStop();
                }

                TP_THROW_ASSERT_IF(error) << error.what();
                                
                if (data.payloadSize > 0) {
                  TP_DCHECK_EQ(allocation.payloads.size(), data.numPayloads);
                  for (size_t payloadIdx = 0; payloadIdx < data.numPayloads;
                       payloadIdx++) {
                    TP_DCHECK_EQ(
                        memcmp(
                            allocation.payloads[payloadIdx].data,
                            data.expectedPayload[payloadIdx].get(),
                            descriptor.payloads[payloadIdx].length),
                        0);
                  }
                } else {
                  TP_DCHECK_EQ(allocation.payloads.size(), 0);
                }
                if (data.tensorSize > 0) {
                  TP_DCHECK_EQ(allocation.tensors.size(), data.numTensors);
                  for (size_t tensorIdx = 0; tensorIdx < data.numTensors;
                       tensorIdx++) {
                    if (data.tensorType == TensorType::kCpu) {
                      // no memory compare
                      
                      TP_DCHECK_EQ(
                          memcmp(
                              allocation.tensors[tensorIdx]
                                  .buffer.unwrap<CpuBuffer>()
                                  .ptr,
                              data.expectedCpuTensor[tensorIdx].get(),
                              descriptor.tensors[tensorIdx].length),
                          0);
                       
                    }  else {
                      TP_THROW_ASSERT() << "Unknown tensor type";
                    }
                  }
                } else {
                  TP_DCHECK_EQ(allocation.tensors.size(), 0);
                }
                 });
                */
              if (numWarmUps == 0) {
                  measurements.cpu.markStop();
                  //std::cout<<"measurements size = " << measurements.cpu.size()<<std::endl;
              }
              if (numWarmUps > 0) {
                numWarmUps -= 1;
              } else {
                numRoundTrips -= 1;
              }
              if (numRoundTrips > 0) {
                clientPingPongNonBlock(
                    pipe,
                    numWarmUps,
                    numRoundTrips,
                    doneProm,
                    data,
                    measurements);
              } else {
                printMultiDeviceMeasurements(measurements, data.payloadSize);
                doneProm.set_value();
              }
              
      });
}

// Start with sending ping
static void runClient(const Options& options) {
  std::string addr = options.address;
  int numWarmUps = kNumWarmUpRounds;
  int numRoundTrips = options.numRoundTrips;

  Data data;
  data.numPayloads = options.numPayloads;
  data.payloadSize = options.payloadSize;
  for (size_t payloadIdx = 0; payloadIdx < options.numPayloads; payloadIdx++) {
    data.expectedPayload.push_back(createFullCpuData(options.payloadSize));
    data.expectedPayloadMetadata.push_back(
        std::string(options.metadataSize, 0x42));
    data.temporaryPayload.push_back(createEmptyCpuData(options.payloadSize));
  }
  data.numTensors = options.numTensors;
  data.tensorSize = options.tensorSize;
  data.tensorType = options.tensorType;
  for (size_t tensorIdx = 0; tensorIdx < options.numTensors; tensorIdx++) {
    data.expectedTensorMetadata.push_back(
        std::string(options.metadataSize, 0x42));
    if (data.tensorType == TensorType::kCpu) {
      data.expectedCpuTensor.push_back(createFullCpuData(options.tensorSize));
      data.temporaryCpuTensor.push_back(createEmptyCpuData(options.tensorSize));
    } else {
      TP_THROW_ASSERT() << "Unknown tensor type";
    }
  }
  data.expectedMetadata = std::string(options.metadataSize, 0x42);

  MultiDeviceMeasurements measurements;
  measurements.cpu.reserve(options.numRoundTrips);

  std::shared_ptr<Context> context = std::make_shared<Context>();
  auto transportContext =
      TensorpipeTransportRegistry().create(options.transport);
  validateTransportContext(transportContext);
  context->registerTransport(0, options.transport, transportContext);

  auto channelContext = TensorpipeChannelRegistry().create(options.channel);
  validateChannelContext(channelContext);
  context->registerChannel(0, options.channel, channelContext);

  std::cout<<"Clinet Trying to connect " << addr <<std::endl;
  std::shared_ptr<Pipe> pipe = context->connect(addr);



  std::promise<void> doneProm;
  clientPingPongNonBlock(
      std::move(pipe), numWarmUps, numRoundTrips, doneProm, data, measurements);

  doneProm.get_future().get();
  context->join();
}

// run  server on komodo2: ./benchmark_pipe --mode listen --transport ibv --channel basic --address ibv://155.198.152.17:3344 --num-round-trips 10
// run  client on any komodo machine: ./benchmark_pipe --mode connect --transport ibv --channel basic --address ibv://155.198.152.17:3344 --num-round-trips 10
int main(int argc, char** argv) {
  struct Options x = parseOptions(argc, argv);
  x.numTensors = 1;
  // tensorSize = 128MB
  x.tensorSize = 1024 * 1024 * 128 ;

  
  std::cout << "mode = " << x.mode << "\n";
  std::cout << "transport = " << x.transport << "\n";
  std::cout << "channel = " << x.channel << "\n";
  std::cout << "address = " << x.address << "\n";
  std::cout << "num_round_trips = " << x.numRoundTrips << "\n";
  std::cout << "num_payloads = " << x.numPayloads << "\n";
  std::cout << "payload_size = " << x.payloadSize << "\n";
  std::cout << "num_tensors = " << x.numTensors << "\n";
  std::cout << "tensor_size = " << x.tensorSize / 1024 / 1024  << "MB \n";
  std::cout << "tensor_type = "
            << (x.tensorType == TensorType::kCpu ? "cpu" : "cuda") << "\n";
  std::cout << "metadata_size = " << x.metadataSize << "\n";

  if (x.mode == "listen") {
    runServer(x);
  } else if (x.mode == "connect") {
    runClient(x);
  } else {
    // Should never be here
    TP_THROW_ASSERT() << "unknown mode: " << x.mode;
  }

  return 0;
}
