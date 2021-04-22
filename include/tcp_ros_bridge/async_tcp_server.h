/*                                                                                 * Filename: async_async_tcp_server.h
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

#include <atomic>
#include <mutex>
#include <condition_variable>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
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

  void start()
  {
    channel_.join(shared_from_this());

    write_th_ = new boost::thread(boost::bind(&tcp_session::write_thread, this));
    read_th_ = new boost::thread(boost::bind(&tcp_session::read_thread, this));
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
  }

  void read_thread()
  {
      start_read();
  }

  void write_thread()
  {
      start_write();
  }

  void start_read()
  {
      std::cout << "[async_tcp_server]: start read" << std::endl;
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
      // std::cout << "[async_tcp_server]: read from port" << std::endl;
      channel_.deliver(buffer_, bytes_transferred);

      boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
      start_read();
    }
    else
    {
      stop();
    }
  }

  void start_write()
  {
      std::cout << "[async_tcp_server]: start write" << std::endl;
      if(output_queue_.empty())
      {
          output_queue_.push_back("\n"); // add heart beat msg
      }
       
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
      if(!output_queue_.empty())
      {
        output_queue_.pop_front();
      }

      boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
      start_write();
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

  boost::thread* read_th_;
  boost::thread* write_th_;
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
    acceptor_.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));

    std::cout << "[async_tcp_server]: Server initialize" << std::endl;
    channel_.join(bc);

    start_accept();
  }

  void addSentMsg(const std::string& msg)
  {
    session_->addSentMsg(msg);
  }

  void start_accept()
  {
    std::cout << "[async_tcp_server]: listen" << std::endl;
    // tcp_session_ptr new_session(new tcp_session(io_service_, channel_));
    session_ = boost::shared_ptr<tcp_session>(new tcp_session(io_service_, channel_));

    acceptor_.async_accept(session_->socket(),
        boost::bind(&async_tcp_server::handle_accept, this, session_, 
                    boost::asio::placeholders::error));
  }

  void handle_accept(tcp_session_ptr session,
      const boost::system::error_code& ec)
  {
    if (!ec)
    {
      std::cout << "[async_tcp_server]: start tcp session" << std::endl;
      session->start();

      start_accept();
    }
    else
    {
        std::cerr << "[async_tcp_server]: error in handle acceptance" << std::endl;
    }
  }

  void stop_server()
  {
      session_->stop();
  }

private:
  boost::asio::io_service& io_service_;
  tcp::acceptor acceptor_;
  channel channel_;
  
  tcp_session_ptr session_;
};

//----------------------------------------------------------------------


