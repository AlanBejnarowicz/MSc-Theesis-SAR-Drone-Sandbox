
#ifndef VECTOR_H
#define VECTOR_H

#include <iostream>
#include <math.h>
#include<raylib.h>


#include "quaternion.h"

namespace Tools {

    class Quaternion;


    class Vector3 {

        private:
            /* data */
        public:
            double x, y, z;

            // constructors
            Vector3(double x, double y, double z);
            Vector3(); // default constructor

            ~Vector3();

            
            // functions
            double magnitude() const;
            Vector3 normalize() const;
            Vector3 cross(const Vector3& q) const;


            //compatibility 
            // Convert between Tools::Vector3 and Raylib::Vector3
            Vector3(const ::Vector3& v); // Convert from Raylib Vector3
            operator ::Vector3() const;  // Convert to Raylib Vector3

            // overrides

            //sum
            Vector3 operator+(const Vector3& q) const;
            Vector3 operator-(const Vector3& q) const;

            // Vector3 element-wise multiplication
            Vector3 operator*(const Vector3& v) const;

            //scaling
            Vector3 operator*(float scalar) const;

            // Rotate Vector by Quaternion
            Vector3 operator*(const Quaternion& q) const;
            


            //divide by scalar
            Vector3 operator/(float scalar) const;

            //cross product
            //Vector3 operator^(const Vector3& q) const;

            // is equal?
            bool operator==(const Vector3& q) const;

            // #####   friends   #####
            friend std::ostream& operator<<(std::ostream& os, const Vector3& q);



    };


    Vector3 operator*(const Quaternion& q, const Vector3& v);








}


#endif // VECTOR_H
