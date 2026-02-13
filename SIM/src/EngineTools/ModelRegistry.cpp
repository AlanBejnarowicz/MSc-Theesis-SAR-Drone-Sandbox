#include "ModelRegistry.h"




ModelManager::ModelManager(){

    std::cout << "ModelManager constructed! \r\n";

}


ModelManager::~ModelManager(){

    for (const auto& md : loaded_models) {

        UnloadModel(md.model);
    }

    loaded_models.clear();

}








void ModelManager::Update_ModelManager(float dt){

}




Model ModelManager::LoadModelFromReg(const std::string& name){

    for (const auto& md : loaded_models) {

        if(md.name == name){
            return md.model;
        }

    }

}



void ModelManager::RegisterModel(const std::string& path, const std::string& name) {

    for (const auto& md : loaded_models) {

        if (md.path == path || md.name == name) {
            return; 
        }
    }

    
    Model loaded = LoadModel(path.c_str());
    ModelInReg mir;

    if (loaded.meshCount > 0) {

        mir.name = name;
        mir.path = path;
        mir.model = loaded;

        loaded_models.push_back(mir);

        std::cout << path <<" model loaded successfully!" << std::endl;

    } else {
        std::cerr << "Failed to load model!   ---   " << path << std::endl;
        return;
    }

}






void ModelManager::LoadImportantModels(void) {
    RegisterModel("rsc/ONR_SHIP/ONR_TUMBLEHOME.obj", "ONR_TUMBLEHOME");
}