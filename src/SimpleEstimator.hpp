#include <Eigen/Dense>
#include "Math.hpp"

using namespace Eigen;

struct PosVel{

    PosVel(){
        pos = Vector3f(0.f, 0.f, 0.f);
        vel = Vector3f(0.f, 0.f, 0.f);
    };

    PosVel(const Vector3f& p, const Vector3f& v){
        pos = p;
        vel = v;
    }

    PosVel operator+=(const PosVel& a) const{
        return PosVel(pos + a.pos, vel + a.vel);
    }

    Vector3f pos;
    Vector3f vel;
};

PosVel operator-(const PosVel& a, const PosVel& b) {
    return PosVel(a.pos - b.pos, a.vel - b.vel);
}

PosVel operator+(const PosVel& a, const PosVel& b) {
    return PosVel(a.pos + b.pos, a.vel + b.vel);
}

PosVel operator*(const PosVel& pv, float a) {
    return PosVel(pv.pos*a, pv.vel*a);
}

PosVel operator*(float a, const PosVel& pv) {
    return PosVel(pv.pos*a, pv.vel*a);
}

PosVel operator/(const PosVel& pv, float a) {
    return PosVel(pv.pos/a, pv.vel/a);
}



class SimpleEstimator
{
public:
    SimpleEstimator(const PosVel& init=PosVel()):
    mesuredInited(false)
    {
    }

PosVel model(PosVel pv, const Vector3f& a){
    pv.pos = pv.vel;
    pv.vel = a;
    return pv;
}

PosVel get(const PosVel& measured, const Vector3f& a, float dt){

    if (!mesuredInited){
        lastMesured = measured;
        mesuredInited = true;
        statePoint = state;
        return get(a, dt);
    }
    PosVel dMeasured = measured - lastMesured;
    lastMesured = measured;

    PosVel dEstimation = dryGet(a, dt) - statePoint;
    dMeasured.vel = cutTwosidesVector3f(dMeasured.vel, -0.5f * dEstimation.vel, 2.5f * dEstimation.vel);
    dMeasured.pos = cutTwosidesVector3f(dMeasured.pos, -0.5f * dEstimation.pos, 2.5f * dEstimation.pos);

    state = statePoint + dMeasured;
    statePoint = state;
    return state;
}

PosVel get(Vector3f a, float dt, bool reset = false){

    if (reset) {
        mesuredInited = false;
    }

    PosVel k1 = model(state, a);
    PosVel k2 = model(state + 0.5f * dt * k1, a);
    PosVel k3 = model(state + 0.5f * dt * k2, a);
    PosVel k4 = model(state + dt * k3, a);
    state = state + (k1 + 2.f*k2 + 2.f*k3 + k4) * dt / 6.f;
    return state;
}

PosVel dryGet(Vector3f a, float dt){
    PosVel k1 = model(state, a);
    PosVel k2 = model(state + 0.5f * dt * k1, a);
    PosVel k3 = model(state + 0.5f * dt * k2, a);
    PosVel k4 = model(state + dt * k3, a);
    return state + (k1 + 2.f*k2 + 2.f*k3 + k4) * dt / 6.f;
}

private:
    PosVel state;
    PosVel statePoint;

    PosVel lastMesured;
    bool mesuredInited;
};
