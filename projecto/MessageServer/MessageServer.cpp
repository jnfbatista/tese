#include "MessageServer.h"

MessageServer::MessageServer(boost::asio::io_service& io_service, int port) : 
	socket_(io_service, udp::endpoint(udp::v6(),port ))
{ 
	start_receive();

}

void MessageServer::start_receive( ) {

	size_t bytes_transferred;
	socket_.async_receive_from(
			boost::asio::buffer(recv_buffer_),
			remote_endpoint_,
			boost::bind(
				&MessageServer::handle_receive,
				this,
				boost::asio::placeholders::error,

				boost::asio::placeholders::bytes_transferred
				)
			);

}

void MessageServer::handle_receive(const boost::system::error_code& error,
		std::size_t /*bytes_transferred*/) {

	printf("UDP Server - handling received packet: %s\n",recv_buffer_.data());
	if (!error || error == boost::asio::error::message_size)
	{
		boost::shared_ptr<std::string> message(
				new std::string("toma lá nutella!"));
		socket_.async_send_to(
				boost::asio::buffer(*message), 
				remote_endpoint_,
				boost::bind(
					&MessageServer::handle_send, 
					this, 
					message,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred
					)
				);

		start_receive();
	}
}

void MessageServer::handle_send(boost::shared_ptr<std::string> /*message*/,
		const boost::system::error_code& /*error*/,
		std::size_t /*bytes_transferred*/)
{
	printf("UDP Server - handle send\n");
}


bool MessageServer::start_server( int address) {

}

