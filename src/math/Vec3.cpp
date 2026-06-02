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
    result.x = mat3a.m[0][0] * vec3a.x + mat3a.m[0][1] * vec3a.y + mat3a.m[0][2] * vec3a.z; // the argument inherits the m as a method use m to access precise matrix coordinates
    result.y = mat3a.m[1][0] * vec3a.x + mat3a.m[1][1] * vec3a.y + mat3a.m[1][2] * vec3a.z; 
    result.z = mat3a.m[2][0] * vec3a.x + mat3a.m[2][1] * vec3a.y + mat3a.m[2][2] * vec3a.z;
    return result;
}
Matrix3 matrixMultiplication(Matrix3 (&mat3a), Matrix3 (&mat3b)){
    Matrix3 result = {}; // initializes a 0 matrix

    for (int i = 0; i < 3; i++) // all rows
    {
        for (int j=0; j < 3; j++){ // all cols
            for (int k = 0; k < 3; k++) // 3rd loops walks thru entire matrix (all cells in matrix)
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
    // this is called euler integration, you take your current velocity
    // and add it to acceleration multiplied by a slice of time, you get a new velocity 
    // in the next iteration the new velocity becomes your old velocity which is added to a * dt to make a newer velocity
    // this process is done over thousands of iterations to get approximations of velocity or position over time
    std::vector<Vector3> positions; // now it's 3d and can inherit the .push_back func
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
        double c1 = (-1 * m_planet * m * G) / mag_cube; // c is short for constant
        // I did this to simplify calculations and make arguments look cleaner for scalar multiplication
        
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

// result.w = w1*w2 - x1*x2 - y1*y2 - z1*z2
// result.x = w1*x2 + x1*w2 + y1*z2 - z1*y2
// result.y = w1*y2 - x1*z2 + y1*w2 + z1*x2
// result.z = w1*z2 + x1*y2 - y1*x2 + z1*w2

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

Vector3 rotateVector( Vector3 vec3a, Quaternion axis){// v' = q*v*q_conjugate. yes order matters.
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



// ============================================================
// ROTATIONAL EQUATIONS OF MOTION
// ============================================================
// 
// STEP 1 - invert inertia tensor (diagonal matrix only)
// I_inv[0][0] = 1 / I[0][0]
// I_inv[1][1] = 1 / I[1][1]
// I_inv[2][2] = 1 / I[2][2]
// all other cells = 0
//
// STEP 2 - compute angular acceleration
// alpha = I_inv * tau
//
// STEP 3 - update angular velocity
// omega = omega + alpha * dt
//
// STEP 4 - convert omega to quaternion
// omega_quat = {w=0, x=omega.x, y=omega.y, z=omega.z}
//
// STEP 5 - compute rate of change of orientation
// q_dot = 0.5 * q * omega_quat
// q_dot.w = 0.5 * (q * omega_quat).w
// q_dot.x = 0.5 * (q * omega_quat).x
// q_dot.y = 0.5 * (q * omega_quat).y
// q_dot.z = 0.5 * (q * omega_quat).z
//
// STEP 6 - integrate orientation forward in time
// q.w = q.w + q_dot.w * dt
// q.x = q.x + q_dot.x * dt
// q.y = q.y + q_dot.y * dt
// q.z = q.z + q_dot.z * dt
//
// STEP 7 - normalize to prevent drift
// q = normalize(q)

Quaternion quaternionAddition(Quaternion ax1, Quaternion ax2){
    double w = ax1.w + ax2.w;
    double a = ax1.x + ax2.x;
    double b = ax1.y + ax2.y;
    double c = ax1.z + ax2.z;
    
    return {w, a, b, c};
}

std::vector<Quaternion> integrateQuaternion(Matrix3 inertia, Vector3 torque,Vector3 omega, Quaternion q, double dt,  int steps){ // euler integration but for quaternions
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

// compute_R: Vector3 v, Vector3 r, Vector3 a, double dt, int steps
// Compute_F_Net: double T, Vector3 u, double m, Vector3 r, Vector3 v, double area, double dce
// integrateQuaternion: Matrix3 inertia, Vector3 torque, Vector3 omega, Quaternion q, double dt,  int steps

// maxThrust(mt), mass(m), dragCoefficient(dce) 

// what I learned:
// - creating a struct means that arguments will require the same struct type
// struc& x, struct& y is how you define the arguments that inherit that struct type
// stuct (&x)[n] can make n multiples of the struct, in this case it can create a vector containing n vectors
// push_back is he append but only works for objects under the <vector> class
// you can reference components inside the struct using . notation
// don't make the argument a const if you're planning to change it

// You can't have a reference as an argument (&vec3a) unless you make the arg const or if the arg is defined outside of the function
// if you do make it const you must make the function const as well, but if you make that function const, all the functions
// in the chain of functions that touch that value in the arg must me const as well, the compiler likes to keep things consistent.
// think of it like this, a predefined variable aka a variable defined outside of the function, is like a good friend you know well
// you know them so well so you give them a nickname, (&) a reference, however you can't do this to someone you just met
// aka an arg that was just placed as a parameter of your function, that value in the arg must have been defined before the function
// for you to give them a reference.