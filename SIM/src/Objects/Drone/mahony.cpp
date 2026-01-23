#include "mahony.h"


Mahony::Mahony(){

    ki = 0.01;
    kp = 0.5;
    imu_dt = 1/120;
    imu_hz = 120;


    gyro_bias.x = 0;
    gyro_bias.y = 0;
    gyro_bias.z = 0;

    est_quat.w = 1;
    est_quat.x = 0;
    est_quat.y = 0;
    est_quat.z = 0;

}



Mahony::~Mahony(){

}


void quat2dcm(Tools::Quaternion qt, float dcm[3][3]) {
    float qw = qt.w, qx = qt.x, qy = qt.y, qz = qt.z;

    dcm[0][0] = 1 - 2 * qy * qy - 2 * qz * qz;
    dcm[0][1] = 2 * qx * qy - 2 * qz * qw;
    dcm[0][2] = 2 * qx * qz + 2 * qy * qw;

    dcm[1][0] = 2 * qx * qy + 2 * qz * qw;
    dcm[1][1] = 1 - 2 * qx * qx - 2 * qz * qz;
    dcm[1][2] = 2 * qy * qz - 2 * qx * qw;

    dcm[2][0] = 2 * qx * qz - 2 * qy * qw;
    dcm[2][1] = 2 * qy * qz + 2 * qx * qw;
    dcm[2][2] = 1 - 2 * qx * qx - 2 * qy * qy;
}


Tools::Vector3 Mahony::quat2euler (Tools::Quaternion q) {
    Tools::Vector3 euler;
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    euler.x = atan2(2*(qw*qx + qy*qz), 1-2*(qx*qx + qy*qy));
    euler.y = asin(2*(qw*qy - qz*qx));
    euler.z = atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz));
    return euler;
}


void transpose_matrix(float *matrix , float *transposed_matrix){

    transposed_matrix[0] = matrix[0];
    transposed_matrix[1] = matrix[3];
    transposed_matrix[2] = matrix[6];
    transposed_matrix[3] = matrix[1];
    transposed_matrix[4] = matrix[4];
    transposed_matrix[5] = matrix[7];
    transposed_matrix[6] = matrix[2];
    transposed_matrix[7] = matrix[5];
    transposed_matrix[8] = matrix[8];

}


void matrix_multiply(const float a[9], const Tools::Vector3 &b, Tools::Vector3 &result) {
    result.x = a[0] * b.x + a[1] * b.y + a[2] * b.z;
    result.y = a[3] * b.x + a[4] * b.y + a[5] * b.z;
    result.z = a[6] * b.x + a[7] * b.y + a[8] * b.z;
}





Tools::Quaternion Mahony::perform_mahony_fusion_6DOF(Tools::Vector3 accel, Tools::Vector3 gyro, Tools::Quaternion est_quat, float dt){

    Tools::Vector3 v_g, v_m, b, h;
    v_g = v_m = b = h = (Tools::Vector3){{0, 0, 0}};

    Tools::Vector3 acc = accel;
    Tools::Vector3 gyr = gyro;
    Tools::Vector3 mag = {0,0,0};

    float gx, gy, gz;
    gx = gyr.x, gy = gyr.y, gz = gyr.z;

    //qw = est_quat.w, qx = est_quat.x, qy = est_quat.y, qz = est_quat.z;
    Tools::Vector3 g = {{0, -GRAVITY_NED, 0}};
    Tools::Vector3 acc_error = {{0, 0, 0}};
    Tools::Vector3 mag_error = {{0, 0, 0}};

    float dcm[3][3];
    float dcm_transposed[9];

    quat2dcm(est_quat, dcm);
    transpose_matrix((float*)dcm, dcm_transposed);

    float a_norm = sqrt(pow(acc.x,2)+pow(acc.y,2)+pow(acc.z,2));
    float m_norm = sqrt(pow(mag.x,2)+pow(mag.y,2)+pow(mag.z,2));

    if ((a_norm > 0))
    {
        // accelerometer part
        Tools::Vector3 a = accel.normalize();
        matrix_multiply(dcm_transposed, g, v_g);
        
        //matrix_cross_product(a.arr, v_g, &acc_error);
        acc_error = v_g.cross(a);
        
        // magnetometer part
        // Tools::Vector3 m = normalize_vector(magn);
        // matrix_multiply(dcm_transposed, m.arr, &h);

        // In NED frame, X axis in body frame align to north, Y axis will 0
        //b << sqrt(pow(h[0],2)+pow(h[1],2)), 0, h[2];
        b.x = sqrt(pow(h.x,2)+pow(h.y,2));
        b.y = 0;
        b.z = h.z;

        
        //v_m = DCM.transpose() * b; // convert the aligned magnetometer vector from navigation frame to body frame
        //Vector3d mag_error = m.cross(v_m); // error of magnetometer (aligned magnetometer vector in body frame cross prodcut with origin magnetometer vector)
        // matrix_multiply(dcm_transposed, b.arr, &v_m);
        // matrix_cross_product(m.arr, v_m.arr, &mag_error);

        // Mahony algorithm
        Tools::Vector3 total_error;
        total_error.x = acc_error.x + mag_error.x;
        total_error.y = acc_error.y + mag_error.y;
        total_error.z = acc_error.z + mag_error.z;

        //this->gyro_bias = this->gyro_bias + this->ki * total_error * this->imu_dt; // estimate current gyroscope bias by adding past changes
        gyro_bias.x = gyro_bias.x + ki * total_error.x * dt;
        gyro_bias.y = gyro_bias.y + ki * total_error.y * dt;
        gyro_bias.z = gyro_bias.z + ki * total_error.z * dt;

        gyr.x = gyr.x + gyro_bias.x + kp * total_error.x; // gyroscope correction
        gyr.y = gyr.y + gyro_bias.y + kp * total_error.y; // gyroscope correction
        gyr.z = gyr.z + gyro_bias.z + kp * total_error.z; // gyroscope correction
        gx = gyr.x, gy = gyr.y, gz = gyr.z;


        //std::cout << "Acc Error: " << acc_error << " | Gyro Bias: " << gyro_bias << std::endl;


    }

    //origin_q << 0, gx, gy, gz; // the current reading of gyroscope in quaternion form
    Tools::Quaternion origin_q = {{gx, gy, gz, 0}};

    //quat_diff = quat_multi(this->est_quat, origin_q);
    Tools::Quaternion quat_diff = {{0, 0, 0, 0}};
    quat_diff.w = est_quat.w*origin_q.w - est_quat.x*origin_q.x - est_quat.y*origin_q.y - est_quat.z*origin_q.z;
    quat_diff.x = est_quat.w*origin_q.x + est_quat.x*origin_q.w + est_quat.y*origin_q.z - est_quat.z*origin_q.y;
    quat_diff.y = est_quat.w*origin_q.y - est_quat.x*origin_q.z + est_quat.y*origin_q.w + est_quat.z*origin_q.x;
    quat_diff.z = est_quat.w*origin_q.z + est_quat.x*origin_q.y - est_quat.y*origin_q.x + est_quat.z*origin_q.w;

    Tools::Quaternion quat_change = {{0, 0, 0, 0}};

    quat_change.w = 0.5 * quat_diff.w; // compute the rate of change of quaternion
    quat_change.x = 0.5 * quat_diff.x; 
    quat_change.y = 0.5 * quat_diff.y; 
    quat_change.z = 0.5 * quat_diff.z; 

    est_quat.w = est_quat.w + quat_change.w * dt; // update the quaternion
    est_quat.x = est_quat.x + quat_change.x * dt; // update the quaternion
    est_quat.y = est_quat.y + quat_change.y * dt; // update the quaternion
    est_quat.z = est_quat.z + quat_change.z * dt; // update the quaternion
    est_quat = est_quat.normalize(); // normalize quaternion vector




    return est_quat;
}


