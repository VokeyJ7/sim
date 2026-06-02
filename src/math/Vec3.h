#pragma once
#include <vector>
#include <cmath>



struct Vector3 {
    double x;
    double y; 
    double z;

};


struct Quaternion { 
    double w;
    double x;
    double y; 
    double z;


};


struct Matrix3{
    double m[3][3];
};

std::vector<Vector3> compute_R(Vector3 v, Vector3 r, Vector3 a, double dt, int steps);
Vector3 computeAcceleration(Vector3 F_net, double m);
Vector3 compute_F_Net(double T, Vector3 u, double m, Vector3 r, Vector3 v, double area, double dce);
Vector3 rotateVector( Vector3 vec3a, Quaternion axis);
Vector3 computeAngularAcceleration(Matrix3 inertia, Vector3 torque);
std::vector<Quaternion> integrateQuaternion(Matrix3 inertia, Vector3 torque,Vector3 omega, Quaternion q, double dt,  int steps);
double magnitude(Vector3 vec3);

