#ifndef MODELDICTIONARY_H
#define MODELDICTIONARY_H

#include <tinyxml.h>
#include <string>
#include <vector>

#include "../Model/Model.h"

using std::string;

class ModelDictionary {

	public:
		std::vector<Model> dictionary;

		// creates the object and attaches the logic to a file
		ModelDictionary(string);
		//~ModelDictionary();
		//
		vector<Model> models;

	private:

		TiXmlDocument filepath;
};

#endif
