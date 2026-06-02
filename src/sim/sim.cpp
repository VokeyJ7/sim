
#include <iostream>
#include "sim.h"



std::istream& operator>>(std::istream& is, Matrix3& I){
    is >> I.m[0][0] >> I.m[1][1] >> I.m[2][2];
    return is;
}


std::istream& operator>>(std::istream& is, Vector3& v){
    is >> v.x >> v.y >> v.z; // tells the >> operator to take in 3 numbers as input for velocity
    return is;

}


std::istream& operator>>(std::istream& is, Quaternion& q){
    is >> q.w >> q.x >> q.y >> q.z; 
    return is; 
}


struct SpacecraftState {

    std::vector<Vector3> position;
    std::vector<Quaternion> quaternion;
    Vector3 acceleration;
    Vector3 velocity;
    Vector3 F_net;
    Matrix3 inertia; 
    Vector3 torque_vector; 
    Vector3 omega; 
    Vector3 unit_direction_vector;
    Vector3 angular_acceleration;
    Vector3 radius;
    double mass; 
    double Thrust; 
    double area;
    double dce;
    double dt;  
    int steps;

};


SpacecraftState input_R(){
    SpacecraftState state;

    double Thrust; 
    Vector3 acceleration; 
    double mass; 
    Vector3 radius; 
    Vector3 velocity; 
    Vector3 unit_direction_vector;
    double area;
    double dce;
    Matrix3 inertia; 
    Vector3 torque; 
    Vector3 omega; 
    Quaternion quaternion; 
    double dt;  
    int steps;

   
    std::cout << "Enter Mass: ";
    std::cin>> mass;
    std::cout << "" << std::endl;
    state.mass = mass;

    
    std::cout << "Enter initial R Vector: ";
    std::cin >> radius;
    state.radius = radius;
    std::cout << "" << std::endl;
    
    std::cout << "Enter initial Quaternion values: ";
    std::cin >> quaternion;
    state.quaternion.push_back(quaternion);
    std::cout << "" << std::endl;

    std::cout << "Enter initial Velocity Vector: ";
    std::cin >> velocity;
    state.velocity = velocity;
    std::cout << "" << std::endl;
    
    std::cout << "Enter initial angular Velocity Vector: ";
    std::cin >> omega;
    state.omega = omega;
    std::cout << "" << std::endl;

    std::cout << "Enter initial acceleration Vector: ";
    std::cin >> acceleration;
    state.acceleration = acceleration;
    std::cout << "" << std::endl;

    
    std::cout << "Enter dt (slice): ";
    std::cin >> dt;
    state.dt = dt;
    std::cout << "" << std::endl;
    
    std::cout << "Enter number of simulation iterations (steps): ";
    std::cin >> steps;
    state.steps = steps;
    std::cout << "" << std::endl;
    
    std::cout << "Enter Thrust Value: ";
    std::cin >> Thrust;
    state.Thrust = Thrust;
    std::cout << "" << std::endl;
    
    std::cout << "Enter Torque Vector: ";
    std::cin >> torque;
    state.torque_vector = torque;
    std::cout << "" << std::endl;
    
    std::cout << "Enter diagonal Inertia Matrix Values: ";
    std::cin >> inertia;
    state.inertia = inertia;
    std::cout << "" << std::endl;

    std::cout << "Enter Unit Direction Vector (Must be a 1 in either of the three dimensions ex: {1,0,0}): ";
    std::cin >> unit_direction_vector;
    state.unit_direction_vector = unit_direction_vector;
    std::cout << "" << std::endl;


    std::cout << "Enter Spacecraft area: ";
    std::cin >> area;
    state.area = area;
    std::cout << "" << std::endl;

    std::cout << "Enter Drag Coefficient: ";
    std::cin >> dce;
    state.dce = dce;
    std::cout << "" << std::endl;

    
    
   
    state.F_net = compute_F_Net(Thrust, unit_direction_vector, mass, radius, velocity, area, dce);
    state.acceleration = computeAcceleration(state.F_net, mass);
    
    state.position =  compute_R(velocity, radius, state.acceleration, dt, steps);
  

    state.angular_acceleration = computeAngularAcceleration( inertia, torque);
    state.quaternion = integrateQuaternion(inertia, torque, omega, quaternion, dt, steps);
   

    
    return state;

}





int main() 
{
    SpacecraftState state = input_R();
    std::cout << "===================" << std::endl;
    std::cout << "Quaternion Values" << std::endl;
    std::cout << "===================" << std::endl;
    for (int i = 0; i < state.quaternion.size(); i++)
    {
        std::cout << state.quaternion[i].w << " " << state.quaternion[i].x << " " << state.quaternion[i].y << " " << state.quaternion[i].z << std::endl;
    }
    std::cout << "Acceleration: " << state.acceleration.x << std::endl;
    std::cout << "===================" << std::endl;
    std::cout << "Position Values" << std::endl;
    std::cout << "===================" << std::endl;
    for (int i = 0; i < state.position.size(); i++)
    {
        std::cout << state.position[i].x << " " << state.position[i].y << " " << state.position[i].z << std::endl;
    }

    
}
