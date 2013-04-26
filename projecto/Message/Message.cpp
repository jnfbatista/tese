#include "Message.h"

Message::Message() {
}

Message::Message(vector< vector<string> > message) {
	message_params = message_params;
}

bool Message::add_param(vector<string> params) {

	message_params.push_back(params);

	return true;
}

bool Message::add_param(string param, string value) {
	vector<string> param_t;
	param_t.push_back(param);
	param_t.push_back(value);

	message_params.push_back(param_t);

}


bool Message::send_message(string address, string port) {
	try {
		boost::asio::io_service io_service;

		udp::resolver resolver(io_service);
		printf("UDP Send - query the server\n");
		udp::resolver::query query(address, port, boost::asio::ip::resolver_query_base::numeric_service);
		printf("UDP Send - resolve the server\n");
		udp::endpoint receiver_endpoint = *resolver.resolve(query);

		printf("UDP Send - create socket\n");

		udp::socket socket(io_service);
		printf("UDP Send - open socket\n");
		socket.open(udp::v4());

		printf("UDP Send - creating buffer\n");

		//@TODO this must be the params transformed to a string
		std::string send_buf = "{";
		for(int i = 0; i < message_params.size(); i++) {

			send_buf += message_params[i][0] + ":";

			for (int j = 1; j < message_params[i].size(); j++ ) {
				send_buf += message_params[i][j];
				if (j + 1  != message_params[i].size() ) {
					send_buf += ",";
				}


			}

			send_buf += ";";
		}
		send_buf += "}";

		printf("UDP Send - sendng buffer\n");
		socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);
		printf("UDP Send - sent buffer\n");

		boost::array<char, 128> recv_buf;
		udp::endpoint sender_endpoint;
		size_t len = socket.receive_from(
				boost::asio::buffer(recv_buf), sender_endpoint);
		printf("data: %s\n",recv_buf.data());

	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

}


bool Message::test_message() {

	vector<string> temp(0);

	temp.push_back("str1");
	temp.push_back("str1");
	temp.push_back("str1");
	temp.push_back("str1");

	message_params.push_back(temp);

	printf("%d, %d\n", message_params.size(), message_params[0].size());

	return true;

}

