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


bool Message::send_message(int address) {
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

