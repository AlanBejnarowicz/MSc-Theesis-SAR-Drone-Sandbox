
#ifndef TCLASS_H
#define TCLASS_H

#include <iostream>
#include <math.h>
#include <SDL2/SDL.h>

#include "MyVector.h"
#include "GameObject.h"


class TemplateClass : public GameObject {

    private:


    public:

    TemplateClass(); // default constructor
    ~TemplateClass();

    void Update(float dt) override;
    void Draw() override;
    void Draw2D() override;
    

};


#endif