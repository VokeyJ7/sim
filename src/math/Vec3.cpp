#include <iostream>
#include <vector>
#include <cmath>
#include <vector>
#include "Vec3.h"






double dotProduct( Vector3 vec3a, Vector3 vec3b ){
    
    double a = vec3a.x * vec3b.x;
    double b = vec3a.y * vec3b.y;
    double c = vec3a.z * vec3b.z;
    double scalar = a + b + c;
 
    return scalar;
}


Vector3 crossProduct( Vector3  vec3a, Vector3 vec3b ){
    
    double a = (vec3a.y * vec3b.z) - (vec3a.z * vec3b.y);
    double b = (vec3a.z * vec3b.x) - (vec3a.x * vec3b.z);
    double c = (vec3a.x * vec3b.y) - (vec3a.y * vec3b.x);
 
    return {a, b, c};
}


Vector3 matrix_vec3_Multiplication( Matrix3 mat3a, Vector3 vec3a){
    Vector3 result;
    result.x = mat3a.m[0][0] * vec3a.x + mat3a.m[0][1] * vec3a.y + mat3a.m[0][2] * vec3a.z;
    result.y = mat3a.m[1][0] * vec3a.x + mat3a.m[1][1] * vec3a.y + mat3a.m[1][2] * vec3a.z; 
    result.z = mat3a.m[2][0] * vec3a.x + mat3a.m[2][1] * vec3a.y + mat3a.m[2][2] * vec3a.z;
    return result;
}


Matrix3 matrixMultiplication(Matrix3 (&mat3a), Matrix3 (&mat3b)){
    Matrix3 result = {}; // initializes a 0 matrix

    for (int i = 0; i < 3; i++) 
    {
        for (int j=0; j < 3; j++){ 
            for (int k = 0; k < 3; k++) 
            {
                result.m[i][j] += mat3a.m[i][k] * mat3b.m[k][j];
            }
            
        }
    }
    

    return result;
}


Vector3 vectorAddition(Vector3 vec3a, Vector3 vec3b){

    double a = vec3a.x + vec3b.x;
    double b = vec3a.y + vec3b.y;
    double c = vec3a.z + vec3b.z;
    
    return {a, b, c};
}


Vector3 vectorSubtraction( Vector3 vec3a, Vector3 vec3b){
    double a = vec3a.x - vec3b.x;
    double b = vec3a.y - vec3b.y;
    double c = vec3a.z - vec3b.z;

    return {a, b, c};
}


Vector3 scalarMultiplication(Vector3 vec3a, double k){

    double a = k * vec3a.x; 
    double b = k * vec3a.y;
    double c = k * vec3a.z;

    return {a, b, c};
}


Vector3 scalarDivision(Vector3 vec3a, double k){

    double a = vec3a.x / k; 
    double b = vec3a.y / k;
    double c = vec3a.z / k;

    return {a, b, c};
}


double magnitude(Vector3 vec3){
    return sqrt(pow(vec3.x, 2) + pow(vec3.y, 2) + pow(vec3.z, 2));
}


Vector3 unit(Vector3 (&vec3)){
    double x = vec3.x;
    double y = vec3.y;
    double z = vec3.z;
    double mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)); 
    x = x / mag;
    y = y / mag;
    z = z / mag;
    return {x, y, z};


}


std::vector<Vector3> compute_R(Vector3 v, Vector3 r, Vector3 a, double dt, int steps) { 
    std::vector<Vector3> positions; 
    for (int i = 0; i < steps; i++)
    {
        
        v = vectorAddition(v, scalarMultiplication(a, dt));
        r = vectorAddition(r, scalarMultiplication(v, dt));
        
        positions.push_back(r);

        
        
        
    }
    return positions;
    

}
Vector3 computeAcceleration(Vector3 F_net, double m){
    Vector3 a;

    a = scalarDivision(F_net, m);

   
    return a;
    
};

Vector3 compute_F_Net(double T, Vector3 u, double m, Vector3 r, Vector3 v, double area, double dce){

    Vector3 fg;
    const double G = 6.6743e-11;
    double m_planet = 5.972e24;

    double mag = magnitude(r);
    if (mag == 0){return {0,0,0};}
    else
    {
        double mag_cube = pow(mag, 3);
        double c1 = (-1 * m_planet * m * G) 

        
        fg = scalarMultiplication(r, c1);


        Vector3 fd;
        double c2 = -0.5 * 1.2250 * dce * area * magnitude(v);
        fd = scalarMultiplication(v, c2);

        Vector3 ft = scalarMultiplication(u, T);
        Vector3 ft_fd = vectorAddition(ft, fd);
        Vector3 ft_fd_fg = vectorAddition(ft_fd, fg);
        return ft_fd_fg;
    }
  


}

Quaternion computeQuaternion(Vector3 axis, double angle){
    axis = unit(axis); // quaternions rely solely on the direction
    double w = cos(angle/2);
    double x = axis.x * sin(angle/2);
    double y = axis.y * sin(angle/2);
    double z = axis.z * sin(angle/2);

    return {w, x, y, z};


}

double mag_quaternion(Quaternion axis){
    return sqrt(pow(axis.x, 2) + pow(axis.y, 2) + pow(axis.z, 2) + pow(axis.w, 2));
}


Quaternion quaternion_multiplication(Quaternion ax1, Quaternion ax2){
    Quaternion result;
    result.w = ax1.w*ax2.w - ax1.x*ax2.x - ax1.y*ax2.y - ax1.z*ax2.z;
    result.x = ax1.w*ax2.x + ax1.x*ax2.w + ax1.y*ax2.z - ax1.z*ax2.y;
    result.y = ax1.w*ax2.y - ax1.x*ax2.z + ax1.y*ax2.w + ax1.z*ax2.x;
    result.z = ax1.w*ax2.z + ax1.x*ax2.y - ax1.y*ax2.x + ax1.z*ax2.w;


    return result;
}

Quaternion conjugate(Quaternion axis){
    return {axis.w, -axis.x, -axis.y, -axis.z};
}

Quaternion normalize(Quaternion axis){
   double mag = mag_quaternion(axis);
   axis.w = axis.w / mag;
   axis.x = axis.x / mag;
   axis.y = axis.y / mag;
   axis.z = axis.z / mag;
   return axis;
}

Vector3 rotateVector( Vector3 vec3a, Quaternion axis){
    Vector3 v_prime;
    Quaternion v_quat;
    v_quat.w = 0.0;
    v_quat.x = vec3a.x;
    v_quat.y = vec3a.y;
    v_quat.z = vec3a.z;

    Quaternion result = quaternion_multiplication(quaternion_multiplication(axis, v_quat), conjugate(axis));
    v_prime.x = result.x;
    v_prime.y = result.y;
    v_prime.z = result.z;
    return v_prime;
}

Matrix3 invertMatrix3(Matrix3 I){
    I.m[0][0] = pow(I.m[0][0], -1);
    I.m[1][1] = pow(I.m[1][1], -1);
    I.m[2][2] = pow(I.m[2][2], -1);
    return I;
}

Quaternion scalarMultiplication_quat(Quaternion axis, double k){
    double w = k * axis.w;
    double a = k * axis.x; 
    double b = k * axis.y;
    double c = k * axis.z;

    return {w, a, b, c};
}

Vector3 computeAngularAcceleration(Matrix3 inertia, Vector3 torque){
    Vector3 angular_acceleration = matrix_vec3_Multiplication(invertMatrix3(inertia), torque);
    return angular_acceleration;
}




Quaternion quaternionAddition(Quaternion ax1, Quaternion ax2){
    double w = ax1.w + ax2.w;
    double a = ax1.x + ax2.x;
    double b = ax1.y + ax2.y;
    double c = ax1.z + ax2.z;
    
    return {w, a, b, c};
}

std::vector<Quaternion> integrateQuaternion(Matrix3 inertia, Vector3 torque,Vector3 omega, Quaternion q, double dt,  int steps){
    Vector3 alpha = computeAngularAcceleration(inertia, torque);
    std::vector<Quaternion> quaternions_integrated;
    
    for (int i = 0; i < steps; i++)
    {

       omega = vectorAddition(omega, scalarMultiplication(alpha, dt));
       Quaternion omega_quat;
       omega_quat.w = 0.0;
       omega_quat.x = omega.x;
       omega_quat.y = omega.y;
       omega_quat.z = omega.z;

       Quaternion q_dot = scalarMultiplication_quat(quaternion_multiplication(q, omega_quat), 0.5);
       q = quaternionAddition(q, scalarMultiplication_quat(q_dot, dt));
       q = normalize(q);
       quaternions_integrated.push_back(q);

    }
    return quaternions_integrated;
    
}

