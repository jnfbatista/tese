#ifndef MODEL_H_
#define MODEL_H_

#include <string>
#include <vector>
#include <map>

using std::vector;
using std::string;
using std::map;

class Model
{
	public:
		Model();
		Model(string);

		// name of the model
		string name;

		// attributes
		map<string,string> params;

		bool add_param(string, string);

		//vector< vector<string> > parameters;

};

#endif
