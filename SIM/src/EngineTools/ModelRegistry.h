#pragma once


#include <iostream>
#include <math.h>


#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <vector>
#include <memory>
#include <iostream>


#include <iostream>
#include <math.h>
#include <raylib.h>
#include "raymath.h"
#include <vector>

struct ModelInReg {
    std::string path;
    std::string name;
    Model model;
};


class ModelManager  {

    private:

        std::vector<ModelInReg> loaded_models;
        

    public:

        ModelManager(); // default constructor
        ~ModelManager();

        void LoadImportantModels(void);
        void Update_ModelManager(float dt);

        Model LoadModelFromReg(const std::string& name);
        void RegisterModel(const std::string& path, const std::string& name);


};


