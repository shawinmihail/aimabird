#include <Eigen/Dense>
#include "Math.hpp"

using namespace Eigen;

struct PosVel{

    PosVel();

    PosVel(const Vector3f& p, const Vector3f& v){
        pos = p;
        vel = v;
    }

    PosVel operator-(const PosVel& a){
        return PosVel(pos - a.pos, vel - a.vel);
    }

    Vector3f pos;
    Vector3f vel;
};

class SimpleEstimator
{
public:
    SimpleEstimator(const PosVel& init=PosVel()){

    }

PosVel get(const PosVel& measured, Vector3f a, double dt){
    PosVel dMeasured = measured - lastMeasured;
    if (isZeroVector3f(dMeasured)){ // the same measuarments -- odometry fail
        state = estimate();
        return state;
    }
    Vector3f dvImu = a * dt;
    dMeasured.vel = cutTwosidesVector3f(dMeasured.vel, 0.5f * dvImu, 1.5f * dvImu);
}

PosVel estimate(){
    return PosVel;
}

private:
    posVel state;
    posVel lastMeasured;
};
