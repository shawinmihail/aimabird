#pragma once

#include <iostream>
#include <stdio.h>
#include <vector>
#include <chrono>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/Thrust.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include "Logger.hpp"
#include "Math.hpp"
#include "Params.h"
#include "SimpleEstimator.hpp"


enum Status{
    begin,
    inited,
    sensorsReady,
    offboarded,
    armed,
    tookoff,
    stabilized,
    stoped
};

enum Command {
    checkSensors,
    prepareToFly,
    takeoff,
    runTestFlight,
    land
};

enum YawStrategy {
    constant,
    onAim,
    aboutAim,
    rotation
};

class AttControl {

public:
    AttControl();
    void prepareToFly();
private:
    void main();
    void checkSensors();
    bool init();
    void subscribe();
    void initPubs();
    void initServs();
    void writeLogData();
    void clockTick();
    float getLastTickDuration();
    void wait(int s);
    bool waiting();
    bool sendIdling();
    bool sendPhotonTm();

    bool checkFeedback();
    bool estimateState();
    bool setOffboard();
    bool arm();
    bool takeoff(float h);
    bool emergencyLand();
    bool stop();
    bool goToLocalPoint(Eigen::Vector3f r0, YawStrategy strategy, float yaw);
    bool goOnRoute(std::vector<Eigen::Vector3f> route); //TODO not implemented yet
    bool goWithVelocity(Eigen::Vector3f v0, float yaw);
    //bool goWithVelocity2D(float h, Eigen::Vector3f v0, float yawRate);
    bool goWithAcc(Eigen::Vector3f a0);
    bool accumulateWithImu(const Eigen::Vector3f& v0, const Eigen::Vector3f& p0 = std::numeric_limits<float>::max() * Eigen::Vector3f(1.f, 1.f, 1.f));
    Eigen::Vector3f getRegVector(Eigen::Vector3f dr, Vector3f dv, float dt);
    Eigen::Vector3f getTwistVector(const Eigen::Quaternion<float>& q0, const Eigen::Vector3f& w0);

    void pubCtrl(float thr, const Eigen::Quaternion<float>& quat);
    void pubCtrl(float thr, const Eigen::Vector3f& v);

    void imuCb(const sensor_msgs::Imu& msg);
    void posCb(const geometry_msgs::PoseStamped& msg);
    void velCb(const geometry_msgs::TwistStamped& msg);
    void stateCb(const mavros_msgs::State& msg);
    void odometryCb(const nav_msgs::Odometry& msg);
    void photonCmdCb(const std_msgs::Float32MultiArray& msg);
    void lidCb(const sensor_msgs::LaserScan& msg);
    void altCb(const sensor_msgs::LaserScan& msg);

public:
    void test();

private:
    std::string imuTopicName;
    std::string posTopicName;
    std::string velTopicName;
    std::string stateTopicName;
    std::string setattTopicName;
    std::string setthrTopicName;
    std::string odometryTopicName;
    std::string setmodeServiceName;
    std::string armServiceName;
    std::string cmdVelTopicName;
    std::string photonCmdTopicName;
    std::string photonTmTopicName;
    std::string useQuatParamTopicName;
    std::string lidTopicName;
    std::string altTopicName;

    ros::NodeHandle nodeHandle;
    ros::ServiceClient modeService;
    ros::ServiceClient armService;
    ros::Subscriber imuSub;
    ros::Subscriber posSub;
    ros::Subscriber velSub;
    ros::Subscriber stateSub;
    ros::Subscriber cmdSub;
    ros::Subscriber odometrySub;
    ros::Subscriber lidSub;
    ros::Subscriber altSub;
    ros::Subscriber photonCmdSub;
    ros::Publisher attPub;
    ros::Publisher velPub;
    ros::Publisher thrPub;
    ros::Publisher photonTmPub;
    ros::Rate rate;


    Eigen::Quaternion<float> qPx;
    Eigen::Vector3f rPx;
    Eigen::Vector3f vPx;
    Eigen::Vector3f aPx;
    Eigen::Vector3f oPx;
    Eigen::Vector3f aPxClearI;
    Eigen::Quaternion<float> qPxInit;
    Eigen::Vector3f gIestimated;

    bool imuReady;
    bool posReady;
    bool velReady;
    bool odometryReady;
    bool lidReady;
    bool altReady;

    mavros_msgs::State mavState;

    Eigen::Quaternion<float> qOd;
    Eigen::Vector3f rOd;
    Eigen::Vector3f vOd;
    Eigen::Vector3f oOd;

    float rLid;
    float vLid;
    float rAlt;
    float vAlt;
    float rLidEs;
    float vLidEs;
    float rAltEs;
    float vAltEs;

    Eigen::Vector3f rInput;

    float yawRateInput;
    float yawPointerForRotate;
    bool yawRateCtrlMode;
    slowYawManager yawManager;

    bool aimAccepted;

    Status status;

    Logger logger;
    FlightData logData;

    DifferenciatorVector3f goLocalDr;
    IntegratorVector3f goLocalIr;
    IntegratorVector3f goAccIr;
    IntegratorVector3f accumulateVelIrEstimator;
    IntegratorVector3f accumulatePosIrEstimator;
    IntegratorVector3f accumulateVelIr;

    IntegratorVector3f goLocalVelIr;

    std::chrono::high_resolution_clock::time_point initTime;
    uint64_t timeMs;
    uint64_t lastTickMs;
    uint64_t dTimeMs;
    uint64_t waitCounter;

    Eigen::Vector3f rEs;
    Eigen::Vector3f vEs;
    Ekf ekfX;
    Ekf ekfY;
    Ekf ekfZ;

    Ekf ekfAlt;
    Ekf ekfLid;
    SeriesDifferentiator<float> altSerDiff;
    SeriesDifferentiator<float> lidSerDiff;

//     DelayIters<Eigen::Vector3f> delayR;
//     DelayIters<Eigen::Vector3f> delayV;
};
