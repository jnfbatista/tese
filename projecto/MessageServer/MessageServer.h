#include <iostream>
#include <ctime>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

class MessageServer {

	/* 
	 * inicia o servidor e espera mensagens para fazer o output
	 *
	 * @TODO define the server control parameters
	 */
	public:
		// constructor with given a port 
		MessageServer(boost::asio::io_service&,int); 

		// start the server
		bool start_server(int);

	private:
		void start_receive();
		void handle_receive(const boost::system::error_code& error,
				std::size_t /*bytes_transferred*/);
		void handle_send(boost::shared_ptr<std::string> /*message*/,
				const boost::system::error_code& /*error*/,
				std::size_t /*bytes_transferred*/);

		udp::socket socket_;
		udp::endpoint remote_endpoint_;
		boost::array<char, 255> recv_buffer_;

};

