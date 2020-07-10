#ifndef TRACER_COMMAND_H_
#define TRACER_COMMAND_H_

#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <vector>
#include <mutex>

using namespace boost::asio;

namespace tracer
{
  namespace controller
  {
    class SerialCommunication
    {
    public:
      SerialCommunication();
      ~SerialCommunication();

      bool openPort(std::string _port, unsigned int _baud_rate);
      void closePort();
      void writeAsync(std::vector<uint8_t>& _send_data);
      void onReceive(const boost::system::error_code& _error, size_t _bytes_transferred);
      void onTimer(const boost::system::error_code& _error);
      void readBufferAsync(uint8_t _size, uint16_t _timeout);
      void startReceive(std::vector<uint8_t>& _receive_data, uint8_t _size, uint16_t _timeout);
      void flushPort();

      std::string receive_buffer_;
      bool on_receive_;

    private:
      io_service io_;
      serial_port serial_;
      deadline_timer timer_;

      bool is_canceled_;
      boost::asio::streambuf stream_buffer_;

    };

    class TracerCommand{
    public:
      //Construct
      TracerCommand();
      //destructor
      ~TracerCommand();

      bool port_open(std::string _port, unsigned int _baud_rate);
      void port_close();
      void port_flush();

      std::vector<uint8_t> get_tracer_data(uint8_t _size);

      bool is_open_;

    private:
      //Value
      unsigned int check_sum_,count_,length_;
      std::vector<uint8_t> send_byte_;
      std::vector<std::string> send_str_;
    protected:
      SerialCommunication serial_com_;
    };

  } //end namespce controller
} //end namespace tracer

#endif
