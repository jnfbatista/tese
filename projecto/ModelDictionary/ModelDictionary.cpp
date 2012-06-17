#include "ModelDictionary.h"


ModelDictionary::ModelDictionary(string file) {
	filepath = TiXmlDocument(file);

	// Loads file to a object
	filepath.LoadFile();

	// creates the dictionary

	TiXmlHandle docHandle(&filepath);

	TiXmlElement* table = docHandle.FirstChild("table").ToElement();

	for(table; table; table = table->NextSiblingElement()) {
		TiXmlElement* name= table->FirstChildElement("name");
		TiXmlElement* top = table->FirstChildElement("top");
		TiXmlElement* legs = table->FirstChildElement("legs");

		// Creates the model
		Model temp_model(name->GetText());

		//<top shape="circular" largest_diagonal="1.5" smallest_diagonal="1.5"/>
		temp_model.add_param("shape", top->Attribute("shape"));
		temp_model.add_param("large_dimension", top->Attribute("large_dimension"));
		temp_model.add_param("small_dimension", top->Attribute("small_dimension"));

		//<legs  number="4" height="1" parallel="true"/>
		temp_model.add_param("number", legs->Attribute("number"));
		temp_model.add_param("height", legs->Attribute("height"));
		temp_model.add_param("parallel", legs->Attribute("parallel"));

		// Then saves it
		models.push_back(temp_model);
		printf("Loaded model: %s\n", name->GetText());
	}

	printf("%s loaded successfuly!\n\n", file.c_str());
}
