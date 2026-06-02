#pragma once
#include <vector>
#include <cmath>


// bad practice to use namespace std in header files can cause naming conflicts in files later
// also .h files talk to each other, simply declare the functions and structs the functions required in the current .h file
// then declare the functions in the other .h file of the cpp file you want to transfer functions to, then you will
// be able to use functions from the 1st cpp file in the 2nd cpp file through communication bewteen .h files

// big idea: headers include other headers so they can borrow stuff from each other

// declare the structs used in the functions you export, then declare the functions that require these structs at the bottom

struct Vector3 {
    double x;
    double y; 
    double z;

};
struct Quaternion { // 4 dimensional struct
    double w;// angle component
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

