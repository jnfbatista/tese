#ifndef MESSAGE_H
#define MESSAGE_H

#include <vector>
#include <string>
#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

using std::vector;
using std::string;
using boost::asio::ip::udp;

/**
 * This class will help the message exchange system of the robot
 *
 * This class will allow to maintain a message with multiple parameters and ultimately send it 
 * using UDP
 * 
 * The objective is to use this class at both endpoints in order to maintain a coherent messaging
 * system, that allows the expansion of the messaging lexicon with no code changes.
 */
class Message {

	public:
		// Blank constructor
		Message(); 

		// constructor with data
		Message(vector< vector<string> >);

		// Add new param to message
		bool add_param(vector<string>);

		bool add_param(string, string);

		bool test_message();

		/**
		 * Sends messaga to a given address in a given port
		 *
		 * @param Address
		 * @param Port
		 */
		bool send_message(string,string);
	private:
		// Trnaslates the parameter to a single string by name
		string param_to_string(vector<string>);
		string param_to_string(string);

		vector< vector<string> > message_params;
};

#endif
