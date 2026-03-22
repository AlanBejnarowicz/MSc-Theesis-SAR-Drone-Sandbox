#include "DroneShip.h"



DroneShip::DroneShip()
{
    position.y = 4.0;
    //rotation = Tools::Quaternion(0,0,0.5);
  
}


DroneShip::DroneShip(Tools::Vector3 pos){

    position = pos;
    position.y = 2.0;

}

DroneShip::~DroneShip()
{

}



// GameObject methods



void DroneShip::Start(){

    // init rigidbody
    rb.BindTransform(position,rotation);
    rb.mass = ship_mass;
    //rb.isKinematic = false;

    tr.BindTransform(position,rotation);

    Ship_CG = {0.0, ship_CG_possition, 0.0};
    rb.COR = {0, 0, 0};


    float half_L = ship_length / 2.0f;
    buoyancy_points.clear();
    buoyancy_points.push_back({ half_L, 0, 0});           // Front
    buoyancy_points.push_back({-half_L, 0, 0});           // Rear
    buoyancy_points.push_back({ 0, 0,  ship_width / 2.0f}); // Left
    buoyancy_points.push_back({ 0, 0, -ship_width / 2.0f}); // Right


    // 1. Create a centering offset 
    // If the tail is 0 and the front is ship_length, we move it back by half_length
    float offset_x = 0.5f;
    Matrix modelOffset = MatrixTranslate(-offset_x, 0.0f, 0.0f);

    // 2. Start with the Offset, then Scale
    static_base_transform = MatrixMultiply(modelOffset, MatrixScale(ship_length, ship_length, ship_length));

    // 3. Apply the mesh correction (e.g., if the Blender model is sideways)
    static_base_transform = MatrixMultiply(static_base_transform, model_correction.RotationMatrix());


    
    

    ship_model = model_manager->LoadModelFromReg("ONR_TUMBLEHOME");


    // Load reference to SEA model

    _sea_model = GetGameObjectByName<SEA>("MainSea");

    if (_sea_model != nullptr) {
        //std::cout << "Sea obj name: " << _sea_model->GameObjectName << " ! \r\n";
        tr.BindSea(*_sea_model);
    } else {
        std::cout << "Warning: DroneShip could not find 'MainSea' during Start()!" << std::endl;
    }


}




void DroneShip::Update(float dt) {

    rb.Update(dt);
    tr.Update(dt);

    FloatPhysics(dt);

    Matrix dynamicRotation = rotation.RotationMatrix();
    Matrix dynamicTranslation = MatrixTranslate(position.x, position.y, position.z);


    Matrix tmp_transform = MatrixMultiply(static_base_transform, MatrixMultiply(dynamicRotation, dynamicTranslation));


    if (renderQueue != nullptr && ship_model!= nullptr) {
        (*renderQueue)[ship_model].push_back(tmp_transform);
    }

    // Tools::Vector3 CG_2 (0.0, 0.0, 0.0);
    // Tools::Vector3 FG_2 (2.1, 0.0, 0.0);
    // rb.AddForce(CG_2, FG_2);

    // Tools::Vector3 CG_3 (-5.0, 2.5, 0.0);
    // Tools::Vector3 FG_3 (-2.10, 0.0, 0.0);
    // rb.AddForce(CG_3, FG_3);


}

void DroneShip::Draw() {

    // Draw the ship model with proper transformation
    //DrawModel(ship_model, {0, 0, 0}, 1.0, WHITE);
    //renderQueue[&ship_model].push_back(ship_model.transform);
    //DrawVectors();
    // rb.Draw();
    // tr.Draw();



    // for(int i = 0; i < buoyancy_points.size(); i++){
    //     Tools::Vector3 sp = buoyancy_points[i];
    //     sp = sp * rotation.inverse();
    //     sp = sp + position;

    //     DrawSphere(sp, 0.08f, GREEN);
    // }
    
    // Tools::Vector3 sg = Ship_CG;
    // sg = sg * rotation.inverse();
    // sg = sg + position;
    // DrawSphere(sg, 0.15f, RED);

    
}

void DroneShip::Draw2D() {
   

}





void DroneShip::FloatPhysics(float dt) {

    // Gravity and CG
    Tools::Vector3 FG (0.0,ship_mass * -9.81  ,0.0);
    rb.AddForce(Ship_CG, FG);

    // ######### buoyancy ##########

    for(int i = 0; i < buoyancy_points.size(); i++){
        Tools::Vector3 sp = buoyancy_points[i];
        sp = sp * rotation.inverse();
        sp = sp + position;

        Tools::Vector3 normal = _sea_model->get_wave_normal(sp);
        double immersion = sp.y - _sea_model->calculate_total_wave_height(sp) + ship_IM_correction;

        if(immersion <= 0 ){
            // point under water
            immersion = abs(immersion);
            Tools::Vector3 B = normal * immersion * immersion * (ship_mass * 1.0f)  ;
            rb.AddForce(buoyancy_points[i], B);

            Tools::Vector3 A = {0, immersion * immersion * (ship_mass * 10.0f)  ,0 };
            A = A * rotation;
            rb.AddForce(buoyancy_points[i], A);
        }


    }


    // add vertical drag in water

    Tools::Vector3 sp = Ship_CG;
    sp = sp * rotation.inverse();
    sp = sp + position;

    Tools::Vector3 normal = _sea_model->get_wave_normal(sp);
    double immersion = sp.y - _sea_model->calculate_total_wave_height(sp);

    if(immersion <= 0) {
        // add drag while in water
        double drag_force = rb.velocity.y * abs(rb.velocity.y) * -1.8;
        Tools::Vector3 vertical_drag (0,drag_force,0);
        rb.AddForce(Ship_CG, vertical_drag);
    }







    // // calculate normals

    // Tools::Vector3 P_front_normal = _sea_model->get_wave_normal(P_front_global);
    // Tools::Vector3 P_rear_normal = _sea_model->get_wave_normal(P_rear_global);
    // Tools::Vector3 P_left_normal = _sea_model->get_wave_normal(P_left_global);
    // Tools::Vector3 P_right_normal = _sea_model->get_wave_normal(P_right_global);

    // // calculate displacement
    // float P_front_h = _sea_model->calculate_total_wave_height(P_front_global) - P_front_global.y;
    // float P_rear_h = _sea_model->calculate_total_wave_height(P_rear_global) - P_rear_global.y;
    // float P_left_h = _sea_model->calculate_total_wave_height(P_left_global) - P_left_global.y;
    // float P_right_h = _sea_model->calculate_total_wave_height(P_right_global) - P_right_global.y;

    



}
