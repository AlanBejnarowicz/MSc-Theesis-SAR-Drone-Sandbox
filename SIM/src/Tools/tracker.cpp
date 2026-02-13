#include "tracker.h"


Tracker::Tracker(float marker_distance ){

    mark_dist = marker_distance;

}


Tracker::~Tracker(){


}



void Tracker::Update(float dt){

    Tools::Vector3 df = *position - last_position;

    if(df.magnitude() >= mark_dist) {
        last_position = *position;
        dots_vector.push_back(last_position);
    }


    int s = dots_vector.size() - max_points;

    for(int i = 0; i < s; i++ ){
        dots_vector.erase(dots_vector.begin());
    }
    


}



void Tracker::Draw(){

    

     for (const auto& dot : dots_vector) {
        Tools::Vector3 corr = {0,0,0};
        if(_sea_model != nullptr){
            corr.y = _sea_model->calculate_total_wave_height(dot);
        }
        // Optional: Draw a small sphere at the point of application
        DrawSphere(dot + corr, 0.05f, YELLOW);
    }

}


// Link the Rigidbody to the GameObject data
void Tracker::BindTransform(Tools::Vector3& pos, Tools::Quaternion& rot) {
    position = &pos;
    rotation = &rot;
}

// Link the Sea to the GameObject data
void Tracker::BindSea(SEA& sea) {

    _sea_model = &sea;
}

