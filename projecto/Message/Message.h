#ifndef MESSAGE_H
#define MESSAGE_H

#include <vector>
#include <string>
#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

using std::vector;
using std::string;

/**
 * This class will help the message exchange system of te robot
 *
 * TODO explain how this works and its structure
 */
class Message {


	public:
		// Blank constructor
		Message(); 

		// constructor with data
		Message(vector< vector<string> >);

		// Add new param to message
		bool add_param(vector<string>);

		bool test_message();

		bool send_message(int);
	private:
		vector< vector<string> > message_params;
};

#endif
