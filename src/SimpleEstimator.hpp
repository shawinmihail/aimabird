#pragma once
#include <Eigen/Dense>
#include "Math.hpp"

using namespace Eigen;

class Ekf {
public:
    Ekf(): inited(false), x(0.f, 0.f)
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

        P = 100 * Q;
    }

    Vector2f model(const Vector2f& x, float a, float dt){
        return Vector2f(x[0] + x[1] * dt, x[1] + a * dt);
    }

    Vector2f get(float rMes, float vMes, float aMes, float dt){

        if (!inited){
            Matrix2f F = E + Phi * dt;
            Matrix2f PApr = F * P * F.transpose() + Q;
            K = PApr * H.transpose() * (H * PApr * H.transpose() + R).inverse();
            P = (E - K * H) * PApr;
            inited = true;
            return x;
        }

        Vector2f xMes(rMes, vMes);
        Vector2f xApr =  model(x, aMes, dt);

        Matrix2f F = E + Phi * dt;
        Matrix2f PApr = F * P * F.transpose() + Q;
        K = PApr * H.transpose() * (H * PApr * H.transpose() + R).inverse();
        P = (E - K * H) * PApr;
        x = xApr + K * (xMes - H * xApr);
        return x;
    }

    Vector2f get(float aMes, float dt){
        return get(x[0], x[1], aMes, dt);
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
};
