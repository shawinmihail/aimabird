#include <iostream>
#include <string>
#include <sstream>
#include <cstdint>
#include <stdio.h>
#include <ros/ros.h>
#include <fcntl.h>
#include <Eigen/Dense>

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


class flightData : public LogData {

public:
    flightData()
    {};

    std::string toCsvDataLine() const override {
        std::stringstream ss;

        ss << timeMs << ",";
        ss << quaternionToStr(qPx);
        ss << vector3fToStr(rPx);
        ss << vector3fToStr(vPx);
        ss << vector3fToStr(aPx);
        ss << vector3fToStr(oPx);

//         ss << quaternionToStr(qGz);
//         ss << vector3fToStr(rGz);
//         ss << vector3fToStr(vGz);
//         ss << vector3fToStr(aGz);
//         ss << vector3fToStr(oGz);

        ss << quaternionToStr(q0);
//         ss << vector3fToStr(r0);
//         ss << vector3fToStr(v0);
//         ss << vector3fToStr(a0);
//         ss << vector3fToStr(o0);

        ss << debug1  << ",";
        ss << debug2  << ",";
        ss << debug3  << ",";
        ss << debug4  << ",";
        ss << "\n";

        return ss.str();
    }

    std::string toCsvHeaderLine() const override {
        std::stringstream ss;

        ss << nameOf(timeMs) << ",";

        ss << quaternionNameToStr(nameOf(qPx));
        ss << vector3fNameToStr(nameOf(rPx));
        ss << vector3fNameToStr(nameOf(vPx));
        ss << vector3fNameToStr(nameOf(aPx));
        ss << vector3fNameToStr(nameOf(oPx));

//         ss << quaternionNameToStr(nameOf(qGz));
//         ss << vector3fNameToStr(nameOf(rGz));
//         ss << vector3fNameToStr(nameOf(vGz));
//         ss << vector3fNameToStr(nameOf(aGz));
//         ss << vector3fNameToStr(nameOf(oGz));

        ss << quaternionNameToStr(nameOf(q0));
//         ss << vector3fNameToStr(nameOf(r0));
//         ss << vector3fNameToStr(nameOf(v0));
//         ss << vector3fNameToStr(nameOf(a0));
//         ss << vector3fNameToStr(nameOf(o0));

        ss << nameOf(debug1) << ",";
        ss << nameOf(debug2) << ",";
        ss << nameOf(debug3) << ",";
        ss << nameOf(debug4) << ",";
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

    Eigen::Quaternion<float> q0;
    Eigen::Vector3f r0;
    Eigen::Vector3f v0;
    Eigen::Vector3f a0;
    Eigen::Vector3f o0;

    float debug1;
    float debug2;
    float debug3;
    float debug4;

    float thr;
};



class Logger{

public:
    Logger(int dataWritePeriod=10):
     _pathDataLog("/home/acsl/1simflightlogs/ctrl_data.csv")
    ,_pathEventsLog("/home/acsl/1simflightlogs/ctrl_events.csv")
    ,_lastDataWriteTimeMs(-1)
    ,_dataWritePeriodMs(dataWritePeriod)
    ,_headered(false)
    {
        _eventsFile = ::fopen(_pathEventsLog.c_str(), "w");
        _dataFile = ::fopen(_pathDataLog.c_str(), "w");
    }

    void addEvent(std::string event, uint64_t timeMs = -1, bool printToConsole = false){

        if(printToConsole){
            printf("[%d] %s\n", timeMs, event.c_str());
        }

        if (_eventsFile == NULL)
            return;

        ::fprintf(_eventsFile, "[%d] %s\n", timeMs, event.c_str());
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
