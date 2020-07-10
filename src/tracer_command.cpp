#include "motion_tracer/tracer_command.h"

using namespace tracer;
using namespace controller;

///////////////////////////////
SerialCommunication::SerialCommunication()
: io_(),serial_(io_),timer_(io_),is_canceled_(false)
{
}

///////////////////////////////
SerialCommunication::~SerialCommunication()
{
  if(serial_.is_open())serial_.close();
}

///////////////////////////////
bool SerialCommunication::openPort(std::string _port, unsigned int _baud_rate)
{
  boost::system::error_code error_code;
  serial_.open(_port,error_code);
  if(error_code){
      return false;
  }
  else{
    serial_.set_option(serial_port_base::baud_rate(_baud_rate));
    return true;
  }
}

///////////////////////////////
void SerialCommunication::closePort()
{
  if(serial_.is_open())serial_.close();
}

///////////////////////////////
void SerialCommunication::writeAsync(std::vector<uint8_t>& _send_data)
{
  serial_.async_write_some( buffer( _send_data ), [](boost::system::error_code, std::size_t){});
  io_.reset();
  io_.run();
}

///////////////////////////////
void SerialCommunication::onReceive(const boost::system::error_code& _error, size_t _bytes_transferred)
{
  if (_error == boost::asio::error::operation_aborted) std::cout << "Timeout" << std::endl;
  else if (_error && _error != boost::asio::error::eof) {
#if DEBUG
      std::cout << "receive failed: " << std::endl;
#endif
  }
  else {
    const std::string data(boost::asio::buffer_cast<const char*>(stream_buffer_.data()), stream_buffer_.size());
    receive_buffer_ = data;

    stream_buffer_.consume(stream_buffer_.size());
    timer_.cancel();
    is_canceled_ = true;
  }
}

///////////////////////////////
void SerialCommunication::onTimer(const boost::system::error_code& _error)
{
    if (!_error && !is_canceled_) serial_.cancel();
}

///////////////////////////////
void SerialCommunication::readBufferAsync(uint8_t _size=1, uint16_t _timeout=10)
{
  receive_buffer_.clear();
  is_canceled_ = false;

  boost::asio::async_read(serial_,stream_buffer_,boost::asio::transfer_at_least(_size),
      boost::bind(&SerialCommunication::onReceive, this,
          boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

  //transfer_all() -> read until full
  //transfer_exactly(size_t size) -> read specific size
  //transfer_at_least(size_t size) -> read at leaset size
  timer_.expires_from_now(boost::posix_time::milliseconds(_timeout));
  timer_.async_wait(boost::bind(&SerialCommunication::onTimer, this, _1));
  io_.reset();
  io_.run();
}

///////////////////////////////
void SerialCommunication::startReceive(std::vector<uint8_t>& _receive_data, uint8_t _size = 1, uint16_t _timeout=1)
{
  boost::asio::async_read(serial_,buffer(_receive_data,_receive_data.size()),boost::asio::transfer_at_least(_size),
      boost::bind(&SerialCommunication::onReceive, this,
          boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  timer_.expires_from_now(boost::posix_time::milliseconds(_timeout));
  timer_.async_wait(boost::bind(&SerialCommunication::onTimer, this, _1));
  io_.reset();
  io_.run();
}

///////////////////////////////
void SerialCommunication::flushPort()
{
  ::tcflush(serial_.lowest_layer().native_handle(),TCIOFLUSH);
}

//*******************************************************************
//*******************************************************************
///////////////////////////////
TracerCommand::TracerCommand()
:check_sum_(0),count_(0),length_(0),serial_com_()
{

}

///////////////////////////////
TracerCommand::~TracerCommand()
{
  port_close();
}

bool TracerCommand::port_open(std::string _port, unsigned int _baud_rate){
  if(serial_com_.openPort(_port, _baud_rate)) is_open_ = true;
  else is_open_ = false;

  return is_open_;
}

void TracerCommand::port_close(){
  serial_com_.closePort();
  is_open_ = false;
}

void TracerCommand::port_flush(){
  serial_com_.flushPort();
}

///////////////////////////////
std::vector<uint8_t> TracerCommand::get_tracer_data(uint8_t _size)
{
  std::vector<uint8_t> receive_data;

  receive_data.resize(_size);
  //serial_com_.readBuffer(receive_data,receive_data.size());
  serial_com_.startReceive(receive_data,receive_data.size(),1000);

#if DEBUG
  for(size_t i=0;i<_size;++i){
    std::cout << std::hex << (int)(receive_data[i] & 0x000000FF)  << " ";
  }
  std::cout << std::endl;
#endif

  return receive_data;
}
