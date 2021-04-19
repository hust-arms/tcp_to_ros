/*                                                                                                                                                                                                    
 * Filename: async_async_tcp_server.h
 * Path: tcp_to_ros
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */
#pragma once

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <set>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>

using boost::asio::deadline_timer;
using boost::asio::ip::tcp;
using boost::asio::ip::udp;

//----------------------------------------------------------------------

class subscriber
{
public:
  virtual ~subscriber() {}
  virtual void deliver(uint8_t* msg, size_t bytes) = 0;
};

typedef boost::shared_ptr<subscriber> subscriber_ptr;

//----------------------------------------------------------------------

class channel
{
public:
  void join(subscriber_ptr subscriber)
  {
    subscribers_.insert(subscriber);
  }

  void leave(subscriber_ptr subscriber)
  {
    subscribers_.erase(subscriber);
  }

  void deliver(uint8_t* msg, size_t bytes)
  {
    std::for_each(subscribers_.begin(), subscribers_.end(),
        boost::bind(&subscriber::deliver, _1, boost::ref(msg), bytes));
  }

private:
  std::set<subscriber_ptr> subscribers_;
};

//----------------------------------------------------------------------

class async_tcp_server; // pre decalration

const int BUFFER_MAX_LEN = 1024;

class tcp_session
  : public subscriber,
    public boost::enable_shared_from_this<tcp_session>
{
    friend async_tcp_server;

public:
  tcp_session(boost::asio::io_service& io_service, channel& ch)
    : channel_(ch),
      socket_(io_service)
  {
  }

  tcp::socket& socket()
  {
    return socket_;
  }

  // Called by the server object to initiate the four actors.
  // void start()
  // {
  //   channel_.join(shared_from_this());
  // 
  //   start_read();
  // 
  //   input_deadline_.async_wait(
  //       boost::bind(&tcp_session::check_deadline,
  //       shared_from_this(), &input_deadline_));
  // 
  //   await_output();
  // 
  //   output_deadline_.async_wait(
  //       boost::bind(&tcp_session::check_deadline,
  //       shared_from_this(), &output_deadline_));
  // }

  void start()
  {
    channel_.join(shared_from_this());

    start_read();
  }

private:
  void stop()
  {
    channel_.leave(shared_from_this());

    socket_.close();
    std::cout << "[async_tcp_server]: close server" << std::endl;
  }

  bool stopped() const
  {
    return !socket_.is_open();
  }

  void deliver(uint8_t* msg, size_t bytes)
  {

  }

  void addSentMsg(const std::string& msg)
  {
    output_queue_.push_back(msg);
    std::cout << "[async_tcp_server]: outside deliver: " << msg << 
                 "output queue size: " << output_queue_.size() << std::endl;
  }

  void start_read()
  {
      socket_.async_read_some(boost::asio::buffer(buffer_, BUFFER_MAX_LEN),
          boost::bind(&tcp_session::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
  }

  void handle_read(const boost::system::error_code& ec, size_t bytes_transferred)
  {
    if (stopped())
      return;

    if (!ec)
    {
      // Extract the newline-delimited message from the buffer.
      // std::string msg(bytes_transferred, 0);
      // for(size_t i = 0; i < bytes_transferred; ++i)
      // {
      //     msg[i] = static_cast<char>(buffer_[i]);
      // }
      // channel_.deliver(msg);
      channel_.deliver(buffer_, bytes_transferred);

      // We received a heartbeat message from the client. If there's nothing
      // else being sent or ready to be sent, send a heartbeat right back.
      if (output_queue_.empty())
      {
        output_queue_.push_back("\n");
      }

      start_write();
    }
    else
    {
      stop();
    }
  }

  void start_write()
  {
    // Start an asynchronous operation to send a message.
    boost::asio::async_write(socket_,
        boost::asio::buffer(output_queue_.front()),
        boost::bind(&tcp_session::handle_write, shared_from_this(), _1));
  }

  void handle_write(const boost::system::error_code& ec)
  {
    if (stopped())
      return;

    if (!ec)
    {
      std::cout << "[async_tcp_server]: write: " << output_queue_.front() << std::endl;
      output_queue_.pop_front();
      start_read();
    }
    else
    {
      stop();
    }
  }

  channel& channel_;
  tcp::socket socket_;
  
  std::deque<std::string> output_queue_;
  uint8_t buffer_[BUFFER_MAX_LEN];
};

typedef boost::shared_ptr<tcp_session> tcp_session_ptr;

//----------------------------------------------------------------------

class udp_broadcaster
  : public subscriber
{
public:
  udp_broadcaster(boost::asio::io_service& io_service,
      const udp::endpoint& broadcast_endpoint)
    : socket_(io_service)
  {
    socket_.connect(broadcast_endpoint);
  }

private:
  void deliver(uint8_t* msg, size_t bytes)
  {
    // boost::system::error_code ignored_ec;
    // socket_.send(boost::asio::buffer(msg), 0, ignored_ec);
  }

  udp::socket socket_;
};

//----------------------------------------------------------------------

class async_tcp_server
{
public:
  async_tcp_server(boost::asio::io_service& io_service,
      const tcp::endpoint& listen_endpoint, 
      subscriber_ptr bc)
    : io_service_(io_service),
      acceptor_(io_service, listen_endpoint)
  {
    std::cout << "[async_tcp_server]: Server initialize" << std::endl;
    channel_.join(bc);

    start_accept();
  }

  void addSentMsg(const std::string& msg)
  {
    std::cout << "[async_tcp_server]: add message: " << msg << std::endl;
    // session_->deliver(msg);
    session_->addSentMsg(msg);
  }

  void start_accept()
  {
    std::cout << "[async_tcp_server]: listen" << std::endl;
    // tcp_session_ptr new_session(new tcp_session(io_service_, channel_));
    session_ = boost::shared_ptr<tcp_session>(new tcp_session(io_service_, channel_));

    // new_session->deliver(w_msg_);

    acceptor_.async_accept(session_->socket(),
        boost::bind(&async_tcp_server::handle_accept, this, session_, 
                    boost::asio::placeholders::error));
  }

  void handle_accept(tcp_session_ptr session,
      const boost::system::error_code& ec)
  {
    if (!ec)
    {
      session->start();

      start_accept();

      // tcp_session_ptr new_session(new tcp_session(io_service_, channel_));
      // new_session->deliver(w_msg_);

      
      // acceptor_.async_accept(new_session->socket(),
      //     boost::bind(&async_tcp_server::handle_accept, this, new_session, _1));
    }
    else
    {
        std::cerr << "[async_tcp_server]: error in handle acceptance" << std::endl;
        session->stop();
    }
  }

private:
  boost::asio::io_service& io_service_;
  tcp::acceptor acceptor_;
  channel channel_;
  
  tcp_session_ptr session_;

  // std::string w_msg_;
};

//----------------------------------------------------------------------


