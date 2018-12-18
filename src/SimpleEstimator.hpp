#pragma once
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
    ,dT(0.f)
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

    dT = 0.f;
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

    dT += dt;
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

    float dT;
};

class Ekf {
public:
    Ekf(): inited(false), T(0.f), x(0.f, 0.f)
    {
        E << 1.f, 0.f,
             0.f, 1.f;

        H << 1.f, 0.f,
             0.f, 1.f;

        Phi << 0.f, 1.f,
               0.f, 0.f;

        Q << 1e-3f, 0.f,
             0.f,  1e-3f;

        R << 1e-1f, 0.f,
             0.f,  1e0f;
    }

    Vector2f model(const Vector2f& x, float a, float dt){
        return Vector2f(x[0] + x[1] * dt, x[1] + a * dt);
    }

    void cutMesurements(float& rMes, float& vMes, float a, float dt){ // experimental TODO remake or remove
        float r = x[0];
        float v = x[1];
        float drMes = rMes - r;
        float dvMes = vMes - v;

        float dvModel = a*dt;
        float drModel = v*dt;

        drMes = cutTwosidesFloat(drMes, 0.f, (dvModel + dvModel / fabs(dvModel) * 0.1f) * 2.f);
        dvMes = cutTwosidesFloat(dvMes, 0.f, (dvModel + dvModel / fabs(dvModel) * 0.2f) * 2.f);

        rMes = r + drModel;
        vMes = v + dvModel;
    }

    Vector2f get(float rMes, float vMes, float aMes, float dt){

//         dt = T + dt;
        T = 0.f;

        if (!inited){
            Matrix2f F = E + Phi * dt;
            Matrix2f PApr = F * P * F.transpose() + Q;
            K = PApr * H.transpose() * (H * PApr * H.transpose() + R).inverse();
            P = (E - K * H) * PApr;
            inited = true;
            return x;
        }

        Vector2f xMes(rMes, vMes);
//         Vector2f k1 = model(x, aMes, dt);
//         Vector2f k2 = model(x + 0.5f * dt * k1, aMes, dt);
//         Vector2f k3 = model(x + 0.5f * dt * k2, aMes, dt);
//         Vector2f k4 = model(x + dt * k3, aMes, dt);
//         Vector2f xApr =  x + (k1 + 2.f * k2 + 2.f * k3 + k4) * dt / 6.f;
        Vector2f xApr =  model(x, aMes, dt);

        Matrix2f F = E + Phi * dt;
        Matrix2f PApr = F * P * F.transpose() + Q;

        K = PApr * H.transpose() * (H * PApr * H.transpose() + R).inverse();
        P = (E - K * H) * PApr;
        x = xApr + K * (xMes - H * xApr);
        return x;
    }


    Vector2f get(float aMes, float dt){

        if (!inited){
            return Vector2f(0.f , 0.f);
        }

        Vector2f k1 = model(x, aMes, dt);
        Vector2f k2 = model(x + 0.5f * dt * k1, aMes, dt);
        Vector2f k3 = model(x + 0.5f * dt * k2, aMes, dt);
        Vector2f k4 = model(x + dt * k3, aMes, dt);
        Vector2f xApr =  x + (k1 + 2.f * k2 + 2.f * k3 + k4) * dt / 6.f;


        T += dt;
        return get(xApr[0], xApr[1], aMes, dt);
    }

private:
    bool inited;

    Vector2f x;
    Matrix2f K;
    Matrix2f P;

    Matrix2f Q;
    Matrix2f R;
    Matrix2f H;
    Matrix2f E;
    Matrix2f Phi;

    float T;
};
