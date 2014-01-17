#ifndef SOCKETSERVER
#define SOCKETSERVER
#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/bind.hpp>

#include <iostream>


class SocketServer;
typedef boost::shared_ptr<SocketServer> SocketServerPtr;

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
		sock_.async_receive(boost::asio::buffer(recvData), boost::bind(&SocketStreamer::recvHandler, shared_from_this(),boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
	}

	void recvHandler(const boost::system::error_code & e, std::size_t);

	SocketServerPtr  server_;
	
private:
	boost::asio::ip::tcp::socket sock_;
	void sendHandler(const boost::system::error_code&, std::size_t)
	{
		std::cout << "send finished!" << std::endl;
	}

	

	std::string data_;
	char recvData[256];
};

class SocketServer :public boost::enable_shared_from_this<SocketServer>
{
public:
	
	SocketServer(boost::asio::io_service &io_service_) :
		acptor_(io_service_, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 9876))
	{
		socketList_.clear();
	}
	void removeStreamFromServer(SocketStreamer::SocketStreamerPtr e)
	{
		auto it = std::find(socketList_.begin(), socketList_.end(), e);
		if (it != socketList_.end())
			socketList_.erase(it);
	}
	void start()
	{

		SocketStreamer::SocketStreamerPtr	newStreamerPtr(new SocketStreamer(acptor_.get_io_service()));
		newStreamerPtr->server_ = shared_from_this();
		acptor_.async_accept(newStreamerPtr->socket_(), boost::bind(&SocketServer::connectHandler, this, newStreamerPtr, boost::asio::placeholders::error));
		std::cout << "server started..." << std::endl;
	}


	std::vector<SocketStreamer::SocketStreamerPtr> socketList_;
private:
	boost::asio::ip::tcp::acceptor acptor_;

	void connectHandler(SocketStreamer::SocketStreamerPtr streamer_, const boost::system::error_code &e)
	{
		socketList_.push_back(streamer_);
		start();
	}
};

class SocketClient
{
public:
	typedef boost::shared_ptr<SocketClient> SocketClientPtr;
	SocketClient(boost::asio::io_service &io_service_, std::string ip_addr_, unsigned short port_) :sock_(io_service_), endpoint_(boost::asio::ip::address::from_string(ip_addr_), port_){}
	bool connect()
	{
		boost::system::error_code ec;
		do{
			sock_.connect(endpoint_, ec);
			if (ec)
			{
				std::cout << "error, retrying..." << std::endl;
			}
		} while (ec);
		return true;
	}




private:
	boost::asio::ip::tcp::socket sock_;
	boost::asio::ip::tcp::endpoint endpoint_;

};


#endif
//int main()
//{
//	boost::asio::io_service io_service;
//	SocketServer server(io_service);
//	
//	io_service.run();
//}