/*
 * Copyright (c) Facebook, Inc. and its affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <tensorpipe/channel/basic/context_impl.h>

#include <functional>
#include <utility>

#include <tensorpipe/channel/basic/channel_impl.h>

namespace tensorpipe {
namespace channel {
namespace basic {

std::shared_ptr<ContextImpl> ContextImpl::create() {
  // basic的模式下，deviceDescriptors之后CPU的信息
  std::unordered_map<Device, std::string> deviceDescriptors = {
      {Device{kCpuDeviceType, 0}, "any"}};
  return std::make_shared<ContextImpl>(std::move(deviceDescriptors));
}

ContextImpl::ContextImpl(
    std::unordered_map<Device, std::string> deviceDescriptors)
    : ContextImplBoilerplate<ContextImpl, ChannelImpl>(
          std::move(deviceDescriptors)) {
          
      std::cout<<"DalongLog:\tCreate Channel"<<std::endl;
        
    }

std::shared_ptr<Channel> ContextImpl::createChannel(
    std::vector<std::shared_ptr<transport::Connection>> connections,
    Endpoint /* unused */) {
  TP_DCHECK_EQ(numConnectionsNeeded(), connections.size());
  // 为啥只需要connections[0], 为啥其它的不需要了？
  return createChannelInternal(std::move(connections[0]));
}

void ContextImpl::handleErrorImpl() {}

void ContextImpl::joinImpl() {}

bool ContextImpl::inLoop() const {
  return loop_.inLoop();
};

void ContextImpl::deferToLoop(std::function<void()> fn) {
  loop_.deferToLoop(std::move(fn));
};

} // namespace basic
} // namespace channel
} // namespace tensorpipe
