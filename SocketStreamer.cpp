#ifndef SOCKETSTREAMER
#define SOCKETSTREAMER
#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/bind.hpp>

#include <iostream>

#include "SocketServer.cpp"

class SocketStreamer :public boost::enable_shared_from_this<SocketStreamer>
{
public:
	typedef boost::shared_ptr<SocketStreamer> SocketStreamerPtr;


	SocketStreamer(boost::asio::io_service& io_service_) :sock_(io_service_){}

	SocketStreamerPtr makeShared()
	{
		return shared_from_this();
	}

	static SocketStreamerPtr create(boost::asio::io_service & io_service_)
	{
		return SocketStreamerPtr(new SocketStreamer(io_service_));
	}

	boost::asio::ip::tcp::socket & socket_(){
		return sock_;
	}

	void sendData(std::vector<unsigned char> &data, size_t size_)
	{

		sock_.async_send(boost::asio::buffer(data, size_), boost::bind(&SocketStreamer::sendHandler, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
	}

	void sendData()
	{
		data_ = "hello";
		sock_.async_send(boost::asio::buffer(data_, 5), boost::bind(&SocketStreamer::sendHandler, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
		sock_.async_receive(boost::asio::buffer(recvData), boost::bind(&SocketStreamer::recvHandler, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
	}

	//SocketServer server_;
private:
	boost::asio::ip::tcp::socket sock_;
	void sendHandler(const boost::system::error_code&, std::size_t)
	{
		std::cout << "send finished!" << std::endl;
	}

	void recvHandler(const boost::system::error_code & e, std::size_t)
	{
		if (e == boost::asio::error::eof)
		{
			/*auto it = std::find(server_->socketList_.begin(), server_->socketList_.end(), shared_from_this());
			if (it != server_->socketList_.end())
			{
			server_->socketList_.erase(it);
			}*/
			//server_->removeStreamFromServer(shared_from_this());
		}
	}
	std::string data_;
	std::string recvData;
};
#endif