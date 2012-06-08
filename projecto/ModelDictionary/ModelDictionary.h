#ifndef MODELDICTIONARY_H
#define MODELDICTIONARY_H

#include <tinyxml.h>
#include <string>

using std::string;

class ModelDictionary
{
public:
	// creates the object and attaches the logic to a file
	ModelDictionary(string);
	~ModelDictionary();

private:

	TiXmlDocument filepath;
};

#endif