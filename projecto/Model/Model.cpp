#include "Model.h"

Model::Model() {}

Model::Model(string model_name) {
	name = model_name;
}


bool Model::add_param(string key, string value) {
	params[key] = value;
}
