#include "ModelDictionary.h"


ModelDictionary::ModelDictionary(string file) {
	filepath = TiXmlDocument(file);
	filepath.LoadFile();
}