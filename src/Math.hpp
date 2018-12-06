#include <Eigen/Dense>
#include <math.h>
#include <limits>
#include <cmath>
#include "Params.h"

bool isZeroFloat(float v, float eps = 1e-3){
    if (std::fabs(v) < eps){
        return true;
    }
    return false;
}

bool isZeroVector3f(const Eigen::Vector3f v, float eps = 1e-3){
    return isZeroFloat(v.norm(), eps);
}

Eigen::Vector3f quatRotate(const Eigen::Quaternion<float>& q, const Eigen::Vector3f v){

  Eigen::Quaternion<float> qRotated(0.f, v[0], v[1], v[2]);
  Eigen::Quaternion<float> rotatedP = q.inverse() * qRotated * q;
  return Eigen::Vector3f(qRotated.x(), qRotated.y(), qRotated.z());
}

Eigen::Quaternion<float> quatEnuToNed(const Eigen::Quaternion<float>& qEnu){
    float s = sqrt(2.f) / 2.f;
    Eigen::Quaternion<float> q1(s, s, 0.f, 0.f);
    Eigen::Quaternion<float> q2(1.f, 0.f, 0.f, 0.f);

    Eigen::Quaternion<float> qNed = qEnu*q2.inverse();
    qNed = q1 * qNed;
    return qNed;
}

float cutAbsFloat(float v, float lim){
    return fmin(fmax(v, -lim), lim);
};

float cutTwosidesFloat(float v, float min, float max){
    float v1 = fmax(v, min);
    float v2 = fmin(v1, max);
    return fmin(fmax(v, min), max);
};

Eigen::Vector3f cutAbsVector3f(const Eigen::Vector3f& v, const Eigen::Vector3f& lim){
    return Eigen::Vector3f(fmin(fmax(v(0), -lim(0)), lim(0)),
                           fmin(fmax(v(1), -lim(1)), lim(1)),
                           fmin(fmax(v(2), -lim(2)), lim(2))
                          );
};

Eigen::Vector3f cutAbsVector3f(const Eigen::Vector3f& v, float lim){
    return Eigen::Vector3f(fmin(fmax(v(0), -lim), lim),
                           fmin(fmax(v(1), -lim), lim),
                           fmin(fmax(v(2), -lim), lim)
                          );
};

Eigen::Vector3f cutTwosidesVector3f(const Eigen::Vector3f& v, const Eigen::Vector3f& min, const Eigen::Vector3f& max){
    return Eigen::Vector3f(fmin(fmax(v(0), min(0)), max(0)),
                           fmin(fmax(v(1), min(1)), max(1)),
                           fmin(fmax(v(2), min(2)), max(2))
                          );
};

Eigen::Vector3f cutTwosidesVector3f(const Eigen::Vector3f& v, float min, float max){
    return Eigen::Vector3f(fmin(fmax(v(0), min), max),
                           fmin(fmax(v(1), min), max),
                           fmin(fmax(v(2), min), max)
                          );
};

Eigen::Quaternion<float> quatFromDirAndYaw(const Eigen::Vector3f v, float yaw, float bowLimit=PI){

    Eigen::Quaternion<float> qYaw(cos(yaw/2.f), 0.f, 0.f, sin(yaw/2.f));
    if (isZeroVector3f(v)){
        return qYaw;
    }

    Eigen::Vector3f pin = UNIT_Z.cross(v);
    pin.normalize();
    float cosA = v.normalized().dot(UNIT_Z);
    cosA = cutAbsFloat(cosA, 1.f - 1e-3); // check if |cosA| < 1 for avoid acos -> Nan
    float A = acos(cosA);
    if(v[2] < 0){
        A = -A;
    }
    A = cutAbsFloat(A, bowLimit);
    float sinHalfA = sin(A/2.f);

    Eigen::Quaternion<float> qBow = Eigen::Quaternion<float>(cos(A/2.f), sinHalfA*pin[0], sinHalfA*pin[1], sinHalfA*pin[2]);

    return qYaw*qBow;
}

class IntegratorFloat{

public:
    IntegratorFloat(float lim = std::numeric_limits<float>::max()):
     _res(0.f)
    {
        _lim = lim;
    }

    float get(float x, float dt){
        _res += x * dt;
        _res = cutAbsFloat(_res, _lim);
        return _res;
    }

    void reset(){
        _res = 0.f;
    }

private:
    float _lim;
    float _res;
};


class DifferenciatorFloat{

public:
    DifferenciatorFloat(float lim = std::numeric_limits<float>::max())
    {
        _lim = lim;
    }

    float get(float x, float dt){
        if (_x0Inited){
            float dxdt = (x - _x0) / dt;
            dxdt = cutAbsFloat(dxdt, _lim);
            _x0 = x;
            return dxdt;
        }
        else{
            _x0 = x;
            _x0Inited = true;
            return 0.;
        }
    }

private:
    float _lim;
    float _x0;
    bool _x0Inited;
};

class IntegratorVector3f{


public:
    IntegratorVector3f(Eigen::Vector3f lim):
     _res(0., 0., 0.)
    {
        _lim = lim;
    }

    IntegratorVector3f(float lim = std::numeric_limits<float>::max()):
     _res(0., 0., 0.)
    {
        _lim = Eigen::Vector3f(lim, lim, lim);
    }

    Eigen::Vector3f get(const Eigen::Vector3f& x, float dt){
        _res += x * dt;
        _res = cutAbsVector3f(_res, _lim);
        return _res;
    }

    void reset(){
        _res *= 0.f;
    }

private:
    Eigen::Vector3f _lim;
    Eigen::Vector3f _res;
};

class DifferenciatorVector3f{

public:
    DifferenciatorVector3f(Eigen::Vector3f lim):
     _x0(0., 0., 0.)
    {
        _lim = lim;
    }

DifferenciatorVector3f(float lim = std::numeric_limits<float>::max())
    {
        _lim = Eigen::Vector3f(lim, lim, lim);
    }

    Eigen::Vector3f get(const Eigen::Vector3f& x, float dt){
        if (_x0Inited){
            Eigen::Vector3f dxdt = (x - _x0) / dt;
            dxdt = cutAbsVector3f(dxdt, _lim);
            _x0 = Eigen::Vector3f(x);
            return dxdt;
        }
        else{
            _x0 = Eigen::Vector3f(x);
            _x0Inited = true;
            return Eigen::Vector3f(0., 0., 0.);
        }
    }

private:
    Eigen::Vector3f _lim;
    Eigen::Vector3f _x0;
    bool _x0Inited;
};
