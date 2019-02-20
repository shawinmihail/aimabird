#pragma once
#include <Eigen/Dense>
#include <math.h>
#include <limits>
#include <cmath>
#include <math.h>
#include "Params.h"
#include <iostream>
#include <stdio.h>
#include <iostream>
#include <string>

bool isNanVect(const Eigen::Vector3f& v){
    return std::isnan(v[0]) || std::isnan(v[1]) || std::isnan(v[2]);
}

void printVect(const Eigen::Vector3f& v, std::string name="v"){
    std::cout << name << ": " << v[0] << " " << v[1]<< " " << v[2] << "\n\n";
}

void printQuat(const Eigen::Quaternion<float>& q, std::string name="q"){
    std::cout << name << ": "<< q.w() << " " << q.x()<< " " << q.y()<< " " << q.z() << "\n\n";
}

void printFloat(float f, std::string name="f"){
    std::cout << name << ": "<< f << "\n\n";
}

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
  qRotated = q.inverse() * qRotated * q;
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

Eigen::Quaternion<float> quatNedToEnu(const Eigen::Quaternion<float>& qNed){
    float s = sqrt(2.f) / 2.f;
    Eigen::Quaternion<float> q1(s, s, 0.f, 0.f);
    Eigen::Quaternion<float> q2(1.f, 0.f, 0.f, 0.f);

    Eigen::Quaternion<float> qEnu = qNed*q2;
    qEnu = q1.inverse() * qEnu;
    return qEnu;
}

float cutAbsFloat(float v, float lim){
    return fmin(fmax(v, -lim), lim);
};

float cutTwosidesFloat(float v, float a, float b){
    float max0 = fmax(a, b);
    float min0 = fmin(a, b);
    return fmin(fmax(v, min0), max0);
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
    return Eigen::Vector3f(cutTwosidesFloat(v(0), min(0), max(0)),
                           cutTwosidesFloat(v(1), min(1), max(1)),
                           cutTwosidesFloat(v(2), min(2), max(2))
                          );
};

Eigen::Vector3f cutTwosidesVector3f(const Eigen::Vector3f& v, float min, float max){
    return Eigen::Vector3f(cutTwosidesFloat(v(0), min, max),
                           cutTwosidesFloat(v(1), min, max),
                           cutTwosidesFloat(v(2), min, max)
                          );
};

Eigen::Quaternion<float> quatFromDirAndYaw(const Eigen::Vector3f v, float yaw, float bowLimit=PI){

    Eigen::Quaternion<float> qYaw(cos(yaw/2.f), 0.f, 0.f, sin(yaw/2.f));
    if (isZeroVector3f(v)){
        return qYaw;
    }

    Eigen::Vector3f pin = UNIT_Z.cross(v);
    if (isZeroVector3f(pin)){
        return qYaw;
    }
    pin.normalize();
    float cosA = v.normalized().dot(UNIT_Z);
    cosA = cutAbsFloat(cosA, 1.f - 1e-9); // check if |cosA| < 1 for avoid acos -> Nan
    float A = acos(cosA);
    if(v[2] < 0){
        A = -A;
    }
    A = cutAbsFloat(A, bowLimit);
    float sinHalfA = sin(A / 2.f);

    Eigen::Quaternion<float> qBow = Eigen::Quaternion<float>(cos(A/2.f), sinHalfA*pin[0], sinHalfA*pin[1], sinHalfA*pin[2]);


    return qBow * qYaw;
}

float yawOnAim(const Eigen::Vector3f& drH) {

    float cosA = UNIT_X.dot(drH.normalized());
    cosA = cutAbsFloat(cosA, 1.f - 1e-9); // check if |cosA| < 1 for avoid acos -> Nan
    float A = acos(cosA);
    if(drH[1] < 0){ // check it
        A = -A;
    }
    return A;
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

    void reset(float part = 0.f){
        _res = _res * part;
    }

private:
    float _lim;
    float _res;
};

Eigen::Vector3f quat2Eul(const Eigen::Quaternion<float>& q){
    float sinr = 2.0f * (q.w() * q.x() + q.y() * q.z());
    float cosr = 1.0f - 2.0f * (q.x() * q.x() + q.y() * q.y());
    float roll = atan2(sinr, cosr);

    float sinp = 2.0f * (q.w() * q.y() - q.z() * q.x());
    float pitch = 0.f;
    if (abs(sinp) >= 1) {
        pitch = sinp / fabs (sinp) * PI / 2.f;
    }
    else {
        pitch = asin(sinp);
    }

    float siny = 2.0f * (q.w() * q.z() + q.x() * q.y());
    float cosy = 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z());
    float yaw = atan2(siny, cosy);
    return Eigen::Vector3f(roll, pitch, yaw);
}



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

    Eigen::Vector3f get(){
        return _res;
    }

    void reset(float part = 0.f){
        _res = _res * part;
    }

    void deflate(float d){
        _res = _res - d * Eigen::Vector3f(1.f, 1.f, 1.f);
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


class BowRateLimiter{
public:
    BowRateLimiter(float lim=0.001):
    _lim(lim)
    {}

    Eigen::Quaternion<float> get(const Eigen::Quaternion<float>& q){
        Eigen::Vector3f v1 = Eigen::Vector3f(q.x(), q.y(), 0.f);
        Eigen::Vector3f dv = v1 - _v;
        dv = cutAbsVector3f(dv, _lim);
        v1 = _v + dv;
        _v = v1;
        return Eigen::Quaternion<float>(q.w(), _v[0], _v[1], q.z());
    }
private:
    Eigen::Vector3f _v;
    float _lim;
};

float modFloat(float x, float y)
{
    if (0. == y)
        return x;

    double m= x - y * floor(x/y);

    // handle boundary cases resulted from floating-point cut off:

    if (y > 0)              // modulo range: [0..y)
    {
        if (m>=y)           // Mod(-1e-16             , 360.    ): m= 360.
            return 0;

        if (m<0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14
        }
    }
    else                    // modulo range: (y..0]
    {
        if (m<=y)           // Mod(1e-16              , -360.   ): m= -360.
            return 0;

        if (m>0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14
        }
    }

    return m;
};

template <class T>
class DelayIters {
public:
    DelayIters(int i = 8){
        _i = i;
    }

    T get(const T& input){
        _v.push_back(input);
        if (_v.size()>_i){
            _v.erase(_v.begin());
        }
        return _v.at(0);
    }

private:
    std::vector<T> _v;
    int _i;
};

template <class T>
class SeriesDifferentiator {
public:
    struct Value {
        template <typename F>
        Value(F&& value, uint64_t time)
            : value(std::forward<F>(value))
            , time(time)
        {
        }

        T value;
        uint64_t time;
    };

    SeriesDifferentiator(int i = 3) : _r(false){
        assert(i != 0);
        _i = i;
    }

    template <typename F>
    T get(F&& input, uint64_t t){
        _v.emplace_back(std::forward<F>(input), t);
        if (_v.size() >_i){
            _r = true;
            _v.erase(_v.begin());
            float dt = (_v.back().time - _v.front().time) / 1e3f;
            return (_v.back().value - _v.front().value) / dt;
        }
        return T();
    }

    bool ready() {
        return _r;
    }

private:
    std::vector<Value> _v;
    int _i;
    bool _r;
};

class slowYawManager
{
public:
    slowYawManager(){};

    float get(float yaw, float yaw0, float dt){
        float dyaw = shortRot(yaw0, yaw);
        if(isZeroFloat(dyaw)){
            return yaw;
        };
        return yaw + cutAbsFloat(dyaw, 0.1f);
    }

    float shortRot(float from, float to)
{
    return std::fmod(from - to + 5.f * PI, PI * 2.f) - PI;
    return std::fmod(from - to + 5.f * PI, PI * 2.f) - PI;
}

};


