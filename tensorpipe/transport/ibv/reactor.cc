/*
 * Copyright (c) Facebook, Inc. and its affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <tensorpipe/transport/ibv/reactor.h>

#include <tensorpipe/common/system.h>
#include <tensorpipe/transport/ibv/constants.h>

namespace tensorpipe {
namespace transport {
namespace ibv {

Reactor::Reactor(IbvLib ibvLib, IbvDeviceList deviceList)
    : ibvLib_(std::move(ibvLib)) {
  TP_DCHECK_GE(deviceList.size(), 1);
  // 所有的都只用第一个device的话，会不会造成device的竞争

  // open device and get device context
  ctx_ = createIbvContext(getIbvLib(), deviceList[0]);
  // create protection domain
  pd_ = createIbvProtectionDomain(getIbvLib(), ctx_);
  // ibv_create_cq creates a completion queue (CQ). A completion queue holds completion queue
  //entries (CQE). Each Queue Pair (QP) has an associated send and receive CQ. A single CQ can be
  //shared for sending and receiving as well as be shared across multiple QPs
  cq_ = createIbvCompletionQueue(
      getIbvLib(),
      ctx_,
      kCompletionQueueSize,
      /*cq_context=*/nullptr,
      /*channel=*/nullptr,
      /*comp_vector=*/0);

  IbvLib::srq_init_attr srqInitAttr;
  std::memset(&srqInitAttr, 0, sizeof(srqInitAttr));
  srqInitAttr.attr.max_wr = kNumPendingRecvReqs;
  // creates a shared receive queue (SRQ).
  srq_ = createIbvSharedReceiveQueue(getIbvLib(), pd_, srqInitAttr);

  // ibv_query_port retrieveibv_query_gid retrieves an entry in the port’s global identifier (GID) table. Each port is assigned
  // at least one GID by the subnet manager (SM). The GID is a valid IPv6 address composed of the
  // globally unique identifier (GUID) and a prefix assigned by the SM. GID[0] is unique and contains
  // the port's GUID.s the various attributes associated with a port
  // 
  addr_ = makeIbvAddress(getIbvLib(), ctx_, kPortNum, kGlobalIdentifierIndex);

  postRecvRequestsOnSRQ(kNumPendingRecvReqs);

  startThread("TP_IBV_reactor");
}

void Reactor::postRecvRequestsOnSRQ(int num) {
  while (num > 0) {
    IbvLib::recv_wr* badRecvWr = nullptr;
    std::array<IbvLib::recv_wr, kNumPolledWorkCompletions> wrs;
    std::memset(wrs.data(), 0, sizeof(wrs));
    for (int i = 0; i < std::min(num, kNumPolledWorkCompletions) - 1; i++) {
      wrs[i].next = &wrs[i + 1];
    }
    /*
      ibv_post_srq_recv posts a list of work requests to the specified SRQ. It stops processing the
      WRs from this list at the first failure (which can be detected immediately while requests are
      being posted), and returns this failing WR through the bad_recv_wr parameter.
      The buffers used by a WR can only be safely reused after WR the request is fully executed and a
      work completion has been retrieved from the corresponding completion queue (CQ).
      If a WR is being posted to a UD QP, the Global Routing Header (GRH) of the incoming message
      will be placed in the first 40 bytes of the buffer(s) in the scatter list. If no GRH is present in the
      incoming message, then the first 40 bytes will be undefined. This means that in all cases for UD
      QPs, the actual data of the incoming message will start at an offset of 40 bytes into the buffer(s)
      in the scatter list.
    */
    int rv = getIbvLib().post_srq_recv(srq_.get(), wrs.data(), &badRecvWr);
    TP_THROW_SYSTEM_IF(rv != 0, errno);
    TP_THROW_ASSERT_IF(badRecvWr != nullptr);
    num -= std::min(num, kNumPolledWorkCompletions);
  }
}

void Reactor::setId(std::string id) {
  id_ = std::move(id);
}

void Reactor::close() {
  if (!closed_.exchange(true)) {
    stopBusyPolling();
  }
}

void Reactor::join() {
  close();

  if (!joined_.exchange(true)) {
    joinThread();
  }
}

Reactor::~Reactor() {
  join();
}

bool Reactor::pollOnce() {
  std::array<IbvLib::wc, kNumPolledWorkCompletions> wcs;
  auto rv = getIbvLib().poll_cq(cq_.get(), wcs.size(), wcs.data());

  if (rv == 0) {
    return false;
  }
  TP_THROW_SYSTEM_IF(rv < 0, errno);

  int numRecvs = 0;
  int numWrites = 0;
  int numAcks = 0;
  for (int wcIdx = 0; wcIdx < rv; wcIdx++) {
    IbvLib::wc& wc = wcs[wcIdx];

    TP_VLOG(9) << "Transport context " << id_
               << " got work completion for request " << wc.wr_id << " for QP "
               << wc.qp_num << " with status "
               << getIbvLib().wc_status_str(wc.status) << " and opcode "
               << ibvWorkCompletionOpcodeToStr(wc.opcode)
               << " (byte length: " << wc.byte_len
               << ", immediate data: " << wc.imm_data << ")";

    auto iter = queuePairEventHandler_.find(wc.qp_num);
    TP_THROW_ASSERT_IF(iter == queuePairEventHandler_.end())
        << "Got work completion for unknown queue pair " << wc.qp_num;

    if (wc.status != IbvLib::WC_SUCCESS) {
      iter->second->onError(wc.status, wc.wr_id);
      continue;
    }

    switch (wc.opcode) {
      case IbvLib::WC_RECV_RDMA_WITH_IMM:
        TP_THROW_ASSERT_IF(!(wc.wc_flags & IbvLib::WC_WITH_IMM));
        iter->second->onRemoteProducedData(wc.imm_data);
        numRecvs++;
        break;
      case IbvLib::WC_RECV:
        TP_THROW_ASSERT_IF(!(wc.wc_flags & IbvLib::WC_WITH_IMM));
        iter->second->onRemoteConsumedData(wc.imm_data);
        numRecvs++;
        break;
      case IbvLib::WC_RDMA_WRITE:
        iter->second->onWriteCompleted();
        numWrites++;
        break;
      case IbvLib::WC_SEND:
        iter->second->onAckCompleted();
        numAcks++;
        break;
      default:
        TP_THROW_ASSERT() << "Unknown opcode: " << wc.opcode;
    }
  }

  postRecvRequestsOnSRQ(numRecvs);

  numAvailableWrites_ += numWrites;
  while (!pendingQpWrites_.empty() && numAvailableWrites_ > 0) {
    postWrite(
        std::get<0>(pendingQpWrites_.front()),
        std::get<1>(pendingQpWrites_.front()));
    pendingQpWrites_.pop_front();
  }

  numAvailableAcks_ += numAcks;
  while (!pendingQpAcks_.empty() && numAvailableAcks_ > 0) {
    postAck(
        std::get<0>(pendingQpAcks_.front()),
        std::get<1>(pendingQpAcks_.front()));
    pendingQpAcks_.pop_front();
  }

  return true;
}

bool Reactor::readyToClose() {
  return queuePairEventHandler_.size() == 0;
}

void Reactor::registerQp(
    uint32_t qpn,
    std::shared_ptr<IbvEventHandler> eventHandler) {
  postWrite.emplace(qpn, std::move(eventHandler));
}

void Reactor::unregisterQp(uint32_t qpn) {
  queuePairEventHandler_.erase(qpn);
}

void Reactor::postWrite(IbvQueuePair& qp, WriteInfo info) {
  if (numAvailableWrites_ > 0) {
    IbvLib::sge list;
    list.addr = reinterpret_cast<uint64_t>(info.addr);
    list.length = info.length;
    list.lkey = info.lkey;

    IbvLib::send_wr wr;
    std::memset(&wr, 0, sizeof(wr));
    wr.wr_id = kWriteRequestId;
    wr.sg_list = &list;
    wr.num_sge = 1;
    wr.opcode = IbvLib::WR_RDMA_WRITE_WITH_IMM;
    wr.imm_data = info.length;
    wr.wr.rdma.remote_addr = info.remoteAddr;
    wr.wr.rdma.rkey = info.rkey;

    IbvLib::send_wr* badWr = nullptr;
    TP_VLOG(9) << "Transport context " << id_ << " posting RDMA write for QP "
               << qp->qp_num;
    TP_CHECK_IBV_INT(getIbvLib().post_send(qp.get(), &wr, &badWr));
    TP_THROW_ASSERT_IF(badWr != nullptr);
    numAvailableWrites_--;
  } else {
    TP_VLOG(9) << "Transport context " << id_
               << " queueing up RDMA write for QP " << qp->qp_num;
    pendingQpWrites_.emplace_back(qp, info);
  }
}

void Reactor::postAck(IbvQueuePair& qp, AckInfo info) {
  if (numAvailableAcks_ > 0) {
    IbvLib::send_wr wr;
    std::memset(&wr, 0, sizeof(wr));
    wr.wr_id = kAckRequestId;
    wr.opcode = IbvLib::WR_SEND_WITH_IMM;
    wr.imm_data = info.length;

    IbvLib::send_wr* badWr = nullptr;
    TP_VLOG(9) << "Transport context " << id_ << " posting send for QP "
               << qp->qp_num;
    TP_CHECK_IBV_INT(getIbvLib().post_send(qp.get(), &wr, &badWr));
    TP_THROW_ASSERT_IF(badWr != nullptr);
    numAvailableAcks_--;
  } else {
    TP_VLOG(9) << "Transport context " << id_ << " queueing send for QP "
               << qp->qp_num;
    pendingQpAcks_.emplace_back(qp, info);
  }
}

} // namespace ibv
} // namespace transport
} // namespace tensorpipe
