#include <boost/asio.hpp>
#include "SocketServer.hpp"
void SocketStreamer::recvHandler(const boost::system::error_code & e, std::size_t)
{
	if (e == boost::asio::error::eof)
	{
		server_->removeStreamFromServer(shared_from_this());

	}
}