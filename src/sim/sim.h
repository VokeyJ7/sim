#pragma once
#include "../math/Vec3.h"

class Spacecraft{
    private:
        double maxThrust;
        double mass;
        double dragCoefficient;
        
        Vector3 position;
        Vector3 velocity;
        Vector3 refAngle;
        
        Quaternion orientation;
        Vector3 torque;
        Vector3 angularVelocity;
        Matrix3 inertia;


       
    public:
        Spacecraft(double mt, double m, double dce) : maxThrust(mt), mass(m), dragCoefficient(dce) {} 
        
        double setMaxThrust(double mt){
            maxThrust = mt;
            return maxThrust;
        }

        double getMaxThrust() const {
            return maxThrust;
        }

        double setMass(double m){
            if (m > 0)
            {
                mass = m;
                return mass;

            };
            return mass;

        };
        
        
        double getMass(){
            return mass;
        }


        double setDragCoefficient(double dce) {
            if (dce > 0)
            {
                dragCoefficient = dce;
                return dragCoefficient;
            }
            return dragCoefficient;
        }
        double getDragCoefficient() const {
            return dragCoefficient;
        }

        Matrix3 setInertia(double x, double y, double z){
            inertia = {};
            inertia.m[0][0] = x;
            inertia.m[1][1] = y;
            inertia.m[2][2] = z;
            return inertia;

           
        }
        Matrix3 getInertia() const {
            return inertia;
        }
        Vector3 setPositon(Vector3 r){
            
            position.x = r.x;
            position.y = r.y;
            position.z = r.z;
            return {position.x, position.y, position.z}; 

        }
        Vector3 getPosition() const {
            return {position.x, position.y, position.z}; 
        };
        Vector3 setRefAngle(double x, double y, double z){
            refAngle.x = x;
            refAngle.y = y;
            refAngle.z = z;
            return {x, y, z};

        }
        Quaternion setOrientation(Quaternion axis){
            orientation.w = axis.w;
            orientation.x = axis.x;
            orientation.y = axis.y;
            orientation.z = axis.z;
            return orientation;
        }
        Quaternion getOrientation() const {
            return orientation;
        }

        Vector3 setTorque(Vector3 t){
            torque.x = t.x;
            torque.y = t.y;
            torque.z = t.z;
            return torque;
        }

        Vector3 getTorque() const {
            return torque;
        }

        Vector3 setAngularVelocity(Vector3 omega){
            angularVelocity.x = omega.x;
            angularVelocity.y = omega.y;
            angularVelocity.z = omega.z;
            return angularVelocity;
        }

        Vector3 getAngularVelocity() const {
            return angularVelocity;
        }

};

std::vector<Vector3> compute_R(Vector3 v, Vector3 r, Vector3 a, double dt, int steps);
Vector3 computeAcceleration(Vector3 F_net, double m);
Vector3 compute_F_Net(double T, Vector3 u, double m, Vector3 r, Vector3 v, double area, double dce);
Vector3 rotateVector(Vector3 vec3a, Quaternion axis);
Vector3 computeAngularAcceleration(Matrix3 inertia, Vector3 torque);
std::vector<Quaternion> integrateQuaternion(Matrix3 inertia, Vector3 torque,Vector3 omega, Quaternion q, double dt,  int steps);
double magnitude(Vector3 vec3);