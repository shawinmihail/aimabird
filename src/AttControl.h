#pragma once

#include <iostream>
#include <stdio.h>
#include <vector>
#include <chrono>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
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
    stabilized
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
    bool takeoff();
    bool goToLocalPoint(Eigen::Vector3f r0, YawStrategy strategy, float yawStrategyParam);
    bool goOnRoute(std::vector<Eigen::Vector3f> route); //TODO not implemented yet
    bool goRelativePosition(Eigen::Vector3f r0);
    bool goWithVelocity(float h, Eigen::Vector3f v0, YawStrategy strategy, float yawStrategyParam);
    bool goWithVelocity2D(float h, Eigen::Vector3f v0, float yawRate);
    bool goWithAcc(Eigen::Vector3f a0);
    bool accomulateVelocityWithImu(Eigen::Vector3f v0);
    bool accomulateVelocityWithImu(float vz);
    Eigen::Vector3f getRegVector(Eigen::Vector3f dr, Vector3f dv, Eigen::Vector3f v, float dt);

    void pubCtrl(float thr, const Eigen::Quaternion<float>& quat);

    void imuCb(const sensor_msgs::Imu& msg);
    void posCb(const geometry_msgs::PoseStamped& msg);
    void velCb(const geometry_msgs::TwistStamped& msg);
    void stateCb(const mavros_msgs::State& msg);
    void odometryCb(const nav_msgs::Odometry& msg);
    void photonCmdCb(const std_msgs::Float32MultiArray& msg);

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
    std::string cmdTopicName;
    std::string photonCmdTopicName;
    std::string photonTmTopicName;

    ros::NodeHandle nodeHandle;
    ros::ServiceClient modeService;
    ros::ServiceClient armService;
    ros::Subscriber imuSub;
    ros::Subscriber posSub;
    ros::Subscriber velSub;
    ros::Subscriber stateSub;
    ros::Subscriber cmdSub;
    ros::Subscriber odometrySub;
    ros::Subscriber photonCmdSub;
    ros::Publisher attPub;
    ros::Publisher thrPub;
    ros::Publisher photonTmPub;
    ros::Rate rate;


    Eigen::Quaternion<float> qPx;
    Eigen::Vector3f rPx;
    Eigen::Vector3f vPx;
    Eigen::Vector3f aPx;
    Eigen::Vector3f oPx;
    Eigen::Vector3f aPxClearI;
    Eigen::Vector3f gIestimated;
    bool gIinited;

    bool imuReady;
    bool posReady;
    bool velReady;

    mavros_msgs::State mavState;

    Eigen::Quaternion<float> qOd;
    Eigen::Vector3f rOd;
    Eigen::Vector3f vOd;
    Eigen::Vector3f oOd;

    Eigen::Vector3f rInput;
    float yawRateInput;
    float yawPointerForRotate;
    bool yawRateCtrlMode;
    bool aimAccepted;

    Status status;

    Logger logger;
    FlightData logData;

    DifferenciatorVector3f goLocalDr;
    IntegratorVector3f goLocalIr;
    IntegratorVector3f goAccIr;
    IntegratorVector3f accamulateVelIrEstimator;
    IntegratorVector3f accamulateVelIr;

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
    bool odometryReady;
};
