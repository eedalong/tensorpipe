/*
 * Copyright (c) Facebook, Inc. and its affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <utility>

#include <tensorpipe/common/callback.h>
#include <tensorpipe/common/defs.h>
#include <tensorpipe/transport/connection_boilerplate.h>
#include <tensorpipe/transport/listener_boilerplate.h>

namespace tensorpipe {
namespace transport {

template <typename TCtx, typename TList, typename TConn>
class ContextImplBoilerplate : public virtual DeferredExecutor,
                               public std::enable_shared_from_this<TCtx> {
 public:
  ContextImplBoilerplate(std::string domainDescriptor);

  ContextImplBoilerplate(const ContextImplBoilerplate&) = delete;
  ContextImplBoilerplate(ContextImplBoilerplate&&) = delete;
  ContextImplBoilerplate& operator=(const ContextImplBoilerplate&) = delete;
  ContextImplBoilerplate& operator=(ContextImplBoilerplate&&) = delete;

  std::shared_ptr<Connection> connect(std::string addr);

  std::shared_ptr<Listener> listen(std::string addr);

  const std::string& domainDescriptor() const;

  ClosingEmitter& getClosingEmitter();

  void setId(std::string id);

  void close();

  void join();

  virtual ~ContextImplBoilerplate() = default;

 protected:
  virtual void closeImpl() = 0;
  virtual void joinImpl() = 0;

  // An identifier for the context, composed of the identifier for the context,
  // combined with the transport's name. It will only be used for logging and
  // debugging purposes.
  std::string id_{"N/A"};

 private:
  std::atomic<bool> closed_{false};
  std::atomic<bool> joined_{false};
  ClosingEmitter closingEmitter_;

  const std::string domainDescriptor_;

  // Sequence numbers for the listeners and connections created by this context,
  // used to create their identifiers based off this context's identifier. They
  // will only be used for logging and debugging.
  std::atomic<uint64_t> listenerCounter_{0};
  std::atomic<uint64_t> connectionCounter_{0};
};

template <typename TCtx, typename TList, typename TConn>
ContextImplBoilerplate<TCtx, TList, TConn>::ContextImplBoilerplate(
    std::string domainDescriptor)
    : domainDescriptor_(std::move(domainDescriptor)) {}

template <typename TCtx, typename TList, typename TConn>
std::shared_ptr<Connection> ContextImplBoilerplate<TCtx, TList, TConn>::connect(
    std::string addr) {
  std::string connectionId = id_ + ".c" + std::to_string(connectionCounter_++);
  TP_VLOG(7) << "Transport context " << id_ << " is opening connection "
             << connectionId << " to address " << addr;
  return std::make_shared<ConnectionBoilerplate<TCtx, TList, TConn>>(
      typename ConnectionImplBoilerplate<TCtx, TList, TConn>::
          ConstructorToken(),
      this->shared_from_this(),
      std::move(connectionId),
      std::move(addr));
}

template <typename TCtx, typename TList, typename TConn>
std::shared_ptr<Listener> ContextImplBoilerplate<TCtx, TList, TConn>::listen(
    std::string addr) {
  std::string listenerId = id_ + ".l" + std::to_string(listenerCounter_++);
  TP_VLOG(7) << "Transport context " << id_ << " is opening listener "
             << listenerId << " on address " << addr;
  return std::make_shared<ListenerBoilerplate<TCtx, TList, TConn>>(
      typename ListenerImplBoilerplate<TCtx, TList, TConn>::ConstructorToken(),
      this->shared_from_this(),
      std::move(listenerId),
      std::move(addr));
}

template <typename TCtx, typename TList, typename TConn>
const std::string& ContextImplBoilerplate<TCtx, TList, TConn>::
    domainDescriptor() const {
  return domainDescriptor_;
}

template <typename TCtx, typename TList, typename TConn>
ClosingEmitter& ContextImplBoilerplate<TCtx, TList, TConn>::
    getClosingEmitter() {
  return closingEmitter_;
};

template <typename TCtx, typename TList, typename TConn>
void ContextImplBoilerplate<TCtx, TList, TConn>::setId(std::string id) {
  TP_VLOG(7) << "Transport context " << id_ << " was renamed to " << id;
  id_ = std::move(id);
}

template <typename TCtx, typename TList, typename TConn>
void ContextImplBoilerplate<TCtx, TList, TConn>::close() {
  if (!closed_.exchange(true)) {
    TP_VLOG(7) << "Transport context " << id_ << " is closing";

    closingEmitter_.close();
    closeImpl();

    TP_VLOG(7) << "Transport context " << id_ << " done closing";
  }
}

template <typename TCtx, typename TList, typename TConn>
void ContextImplBoilerplate<TCtx, TList, TConn>::join() {
  close();

  if (!joined_.exchange(true)) {
    TP_VLOG(7) << "Transport context " << id_ << " is joining";

    joinImpl();

    TP_VLOG(7) << "Transport context " << id_ << " done joining";
  }
}

} // namespace transport
} // namespace tensorpipe
