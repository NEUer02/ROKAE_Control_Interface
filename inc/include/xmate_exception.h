
/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#pragma once
#include <exception>

#include "log.h"

namespace xmate {

class XmateException : public std::exception {
   public:
    explicit XmateException(const std::string &msg) : _Msg(msg){};

   public:
    virtual const char *what() const _GLIBCXX_USE_NOEXCEPT override { return _Msg.c_str(); }

   private:
    std::string _Msg;
};

class UdpException : public XmateException {
   public:
    explicit UdpException(const std::string &msg) : XmateException(msg){};
};

class TcpException : public XmateException {
   public:
    explicit TcpException(const std::string &msg) : XmateException(msg){};
};

class ThreadException : public XmateException {
   public:
    explicit ThreadException(const std::string &msg) : XmateException(msg){};
};

class NetworkException : public XmateException {
   public:
    explicit NetworkException(const std::string &msg) : XmateException(msg){};
};

class TypeException : public XmateException {
   public:
    explicit TypeException(const std::string &msg) : XmateException(msg){};
};
class EventException : public XmateException {
   public:
    explicit EventException(const std::string &msg) : XmateException(msg){};
};

class ControlException : public XmateException {
   public:
    // explicit ControlException(const std::string &msg) : XmateException(msg){};
    explicit ControlException(const std::string &msg, std::vector<xmate::Record> log = {}) noexcept;
    const std::vector<xmate::Record> log;
};

class ModelException : public XmateException {
    explicit ModelException(const std::string &msg) : XmateException(msg){};
};

struct ProtocolException : public XmateException {
    explicit ProtocolException(const std::string &msg) : XmateException(msg){};
};

struct CommandException : public XmateException {
    explicit CommandException(const std::string &msg) : XmateException(msg){};
};

struct RealtimeException : public XmateException {
    explicit RealtimeException(const std::string &msg) : XmateException(msg){};
};
struct InvalidOperationException : public XmateException {
    explicit InvalidOperationException(const std::string &msg) : XmateException(msg){};
};

struct MotionException : public XmateException {
    explicit MotionException(const std::string &msg) : XmateException(msg){};
};

}  // namespace xmate
