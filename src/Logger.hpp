#pragma once
#include <iostream>
#include <string>
#include <sstream>
#include <cstdint>
#include <stdio.h>
#include <ros/ros.h>
#include <fcntl.h>
#include <Eigen/Dense>
#include "Params.h"

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
struct passwd *pw = getpwuid(getuid());
const char *homedir = pw->pw_dir;

#define nameOf(name) #name


class LogData {
public:
    virtual std::string toCsvHeaderLine() const = 0;
    virtual std::string toCsvDataLine() const = 0;

    std::string arrayNameToStr(std::string vName, int length) const {
        std::stringstream ss;
        for (int i = 0; i < length; ++i){
            ss << vName << "." << std::to_string(i);
            ss << ",";
        }
        return ss.str();
    }

    std::string arrayToStr(const double* v, int length = 6) const {
        std::stringstream ss;
        for (int i = 0; i < length; ++i){
            ss << v[i];
            ss << ",";
        }
        return ss.str();
    }

    uint64_t timeMs;
};

class FlightData : public LogData {

public:
    FlightData()
    {};

    std::string toCsvDataLine() const override {
        std::stringstream ss;

        ss << timeMs << ",";
        ss << quaternionToStr(qPx);

        if (USE_GPS){
        ss << vector3fToStr(rPx);
        ss << vector3fToStr(vPx);
        }
        ss << vector3fToStr(aPx);
//         ss << vector3fToStr(oPx);

        if (USE_ODOMETRY){
        ss << quaternionToStr(qOd);
        ss << vector3fToStr(rOd);
        ss << vector3fToStr(vOd);

        ss << vector3fToStr(rEs);
        ss << vector3fToStr(vEs);
        }

        ss << quaternionToStr(q0);
        ss << vector3fToStr(r0);
        ss << vector3fToStr(v0);

        if (USE_ALTIMETR){
        ss << rAlt  << ",";
        ss << vAlt  << ",";
        ss << rAltEs  << ",";
        ss << vAltEs  << ",";
        }

        if (USE_LIDAR){
        ss << rLid  << ",";
        ss << vLid  << ",";
        ss << rLidEs  << ",";
        ss << vLidEs  << ",";
        }
        ss << "\n";

        return ss.str();
    }

    std::string toCsvHeaderLine() const override {
        std::stringstream ss;

        ss << nameOf(timeMs) << ",";

        ss << quaternionNameToStr(nameOf(qPx));

        if (USE_GPS){
        ss << vector3fNameToStr(nameOf(rPx));
        ss << vector3fNameToStr(nameOf(vPx));
        }

        ss << vector3fNameToStr(nameOf(aPx));
//         ss << vector3fNameToStr(nameOf(oPx));

        if (USE_ODOMETRY){
        ss << quaternionNameToStr(nameOf(qOd));
        ss << vector3fNameToStr(nameOf(rOd));
        ss << vector3fNameToStr(nameOf(vOd));
        ss << vector3fNameToStr(nameOf(rEs));
        ss << vector3fNameToStr(nameOf(vEs));
        }
//         ss << vector3fNameToStr(nameOf(aGz));
//         ss << vector3fNameToStr(nameOf(oGz));

        ss << quaternionNameToStr(nameOf(q0));
        ss << vector3fNameToStr(nameOf(r0));
        ss << vector3fNameToStr(nameOf(v0));
//         ss << vector3fNameToStr(nameOf(a0));
//         ss << vector3fNameToStr(nameOf(o0));

        if (USE_ALTIMETR){
        ss << nameOf(rAlt) << ",";
        ss << nameOf(vAlt) << ",";
        ss << nameOf(rAltEs) << ",";
        ss << nameOf(vAltEs) << ",";
        }
        if (USE_LIDAR){
        ss << nameOf(rLid) << ",";
        ss << nameOf(vLid) << ",";
        ss << nameOf(rLidEs) << ",";
        ss << nameOf(vLidEs) << ",";
        }
        ss << "\n";

        return ss.str();
    }

    std::string quaternionNameToStr(std::string qName) const {
        std::stringstream ss;
        ss << qName << ".w," << qName << ".x," << qName << ".y," << qName << ".z,";
        return ss.str();
    }

    std::string vector3fNameToStr(std::string vName) const {
        std::stringstream ss;
        ss << vName << ".x," << vName << ".y," << vName << ".z,";
        return ss.str();
    }

    std::string quaternionToStr(const Eigen::Quaternion<float>& q) const {
        std::stringstream ss;
        ss << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ",";
        return ss.str();
    }

    std::string vector3fToStr(const Eigen::Vector3f& v) const {
        std::stringstream ss;
        ss << v(0) << "," << v(1) << "," << v(2) << ",";
        return ss.str();
    }

public:

    Eigen::Quaternion<float> qPx; // attitude
    Eigen::Vector3f rPx; // pose
    Eigen::Vector3f vPx; // velocity
    Eigen::Vector3f aPx; // acceleration
    Eigen::Vector3f oPx; //ang velocity

    Eigen::Quaternion<float> qGz;
    Eigen::Vector3f rGz;
    Eigen::Vector3f vGz;
    Eigen::Vector3f aGz;
    Eigen::Vector3f oGz;

    Eigen::Quaternion<float> qOd;
    Eigen::Vector3f rOd;
    Eigen::Vector3f vOd;
    Eigen::Vector3f aOd;
    Eigen::Vector3f oOd;

    Eigen::Quaternion<float> qEs;
    Eigen::Vector3f rEs;
    Eigen::Vector3f vEs;
    Eigen::Vector3f aEs;
    Eigen::Vector3f oEs;

    Eigen::Quaternion<float> q0;
    Eigen::Vector3f r0;
    Eigen::Vector3f v0;
    Eigen::Vector3f a0;
    Eigen::Vector3f o0;


    float rAlt;
    float vAlt;
    float rAltEs;
    float vAltEs;
    float rLid;
    float vLid;
    float rLidEs;
    float vLidEs;

    float thr;
};

class Logger{

public:
    Logger(std::string dataPath = "/1simflightlogs/ctrl_data.csv",
           std::string eventsPath = "/1simflightlogs/ctrl_events.csv",
           int dataWritePeriod=10):
     _pathDataLog(std::string(homedir) + dataPath)
    ,_pathEventsLog(std::string(homedir) + eventsPath)
    ,_lastDataWriteTimeMs(-1)
    ,_dataWritePeriodMs(dataWritePeriod)
    ,_headered(false)
    {
        _eventsFile = ::fopen(_pathEventsLog.c_str(), "w");
        _dataFile = ::fopen(_pathDataLog.c_str(), "w");
        std::cout << _pathDataLog;
    }

    void addEvent(std::string event, uint64_t timeMs = -1, bool printToConsole = true){

        if(printToConsole){
            printf("%s [%d]\n", event.c_str(), timeMs);
        }

        if (_eventsFile == NULL)
            return;

        ::fprintf(_eventsFile, "%s [%d]\n", event.c_str(), timeMs);
        ::fflush(_eventsFile);
    }

    void addData(const LogData& logData){
        if (_dataFile == NULL)
            return;

        if (!_headered){
            ::fprintf(_dataFile, logData.toCsvHeaderLine().c_str());
            ::fflush(_dataFile);
            _headered = true;
        }

        int64_t tMs = logData.timeMs;
        if (tMs - _lastDataWriteTimeMs < _dataWritePeriodMs){
            return;
        }

        ::fprintf(_dataFile, logData.toCsvDataLine().c_str());
        ::fflush(_dataFile);

        _lastDataWriteTimeMs = tMs;
    }


private:
    std::string _pathEventsLog;
    std::string _pathDataLog;

    FILE* _eventsFile;
    FILE* _dataFile;

    uint64_t _dataWritePeriodMs;
    uint64_t _lastDataWriteTimeMs;

    bool _headered;
};

enum readResult{
    ok,
    bad,
    end

};

class Player{

public:
    Player():
     _pathDataLog("/home/acsl/1simflightlogs/ctrl_data.csv")
    ,_headerSkipped(false)
    {
        _dataFile = ::fopen(_pathDataLog.c_str(), "r");
    }


    readResult readData(FlightData* ld) {

        if (_dataFile == NULL){
            return end;
        }

        char * line = NULL;
        size_t len = 0;
        ssize_t read;

        read = ::getline(&line, &len, _dataFile);
        if (read == -1){
            ::fclose(_dataFile);
            _dataFile = NULL;
            return end;
        }
        if (!_headerSkipped){
            _headerSkipped = true;
            return bad;
        }

        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> v;
        while(std::getline(ss, token, ',')) {
            v.push_back(token);
        }

        ld->timeMs = biteUint64t(v);
        ld->qPx = biteQuatF(v);
        ld->rPx = biteVecF(v);
        ld->vPx = biteVecF(v);
        ld->aPx = biteVecF(v);
        ld->oPx = biteVecF(v);
        ld->rOd = biteVecF(v);
        ld->vOd = biteVecF(v);
        ld->rEs = biteVecF(v);
        ld->vEs = biteVecF(v);
        ld->q0 = biteQuatF(v);

        return ok;
    }

    uint64_t biteUint64t(std::vector<std::string>& v){
        uint64_t r = std::stoull(v.at(0));
        if (v.size() > 1){
            v = std::vector<std::string>(v.begin() + 1, v.end());
        }
        return r;
    }

    float biteFloat(std::vector<std::string>& v){
        float r = (float)std::stold(v.at(0));
        if (v.size() > 1){
            v = std::vector<std::string>(v.begin() + 1, v.end());
        }
        return r;
    }

    Eigen::Quaternion<float> biteQuatF(std::vector<std::string>& v){
        float w = biteFloat(v);
        float x = biteFloat(v);
        float y = biteFloat(v);
        float z = biteFloat(v);
        Eigen::Quaternion<float> r(w, x, y, z);
        return r;
    }

    Eigen::Vector3f biteVecF(std::vector<std::string>& v){
        float x = biteFloat(v);
        float y = biteFloat(v);
        float z = biteFloat(v);
        Eigen::Vector3f r(x, y, z);
        return r;
    }

private:
    std::string _pathDataLog;

    FILE* _dataFile;
    bool _headerSkipped;
};
