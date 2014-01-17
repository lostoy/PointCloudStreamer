//
// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <ctime>
#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include<boost/thread.hpp>

using boost::asio::ip::tcp;
bool flag;
void handler(char data[], const::boost::system::error_code &e, std::size_t bytes_rec)
{
	flag = true;
	std::cout << "okay!!" <<bytes_rec<< std::endl;
	data[bytes_rec] = 0;
}
int main()
{
	boost::asio::io_service socket_io;
	
	
	boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string("127.0.0.1"),9876);
	boost::asio::ip::tcp::socket sock(socket_io);
	
	boost::system::error_code ec;
	do{
		sock.connect(endpoint, ec);
		if (ec)
		{
			std::cout << "error!!!" << std::endl;
		}
	} while (ec);
	std::cout << "connected (message from clinet)" << std::endl;
	
	char data[256];
	memset(data, 0, sizeof(data));
	
	//size_t recBytes = sock.receive(boost::asio::buffer(data));
	sock.async_receive(boost::asio::buffer(data),bind(handler,data,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));//receive(boost::asio::buffer(data));
	flag = false;
	socket_io.run();
	while (!flag)
	{
		std::cout <<" I'm waiting..." << std::endl;
		boost::this_thread::sleep(boost::posix_time::millisec(500));
	}
	
	std:: cout << data << std::endl;

}