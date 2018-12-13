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
    void wait(int s);
    bool waiting();
    bool sendIdling();

    bool checkFeedback();
    bool estimateState(bool measured);
    bool setOffboard();
    bool arm();
    bool takeoff();
    bool goToLocalPoint(Eigen::Vector3f r0);
    bool goOnRoute(std::vector<Eigen::Vector3f> route); //TODO not implemented yet
    bool goRelativePosition(Eigen::Vector3f r0);
    bool goWithAcc(Eigen::Vector3f a0);
    bool accomulateVelocityWithImu(Eigen::Vector3f v0);
    bool accomulateVelocityWithImu(float vz);

    void pubCtrl(float thr, const Eigen::Quaternion<float>& quat);

    void imuCb(const sensor_msgs::Imu& msg);
    void posCb(const geometry_msgs::PoseStamped& msg);
    void velCb(const geometry_msgs::TwistStamped& msg);
    void stateCb(const mavros_msgs::State& msg);
    void odometryCb(const nav_msgs::Odometry& msg);
    void photonCmdCb(const std_msgs::UInt32MultiArray& msg);

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
    std::string photonTopicName;

    ros::NodeHandle nodeHandle;
    ros::ServiceClient modeService;
    ros::ServiceClient armService;
    ros::Subscriber imuSub;
    ros::Subscriber posSub;
    ros::Subscriber velSub;
    ros::Subscriber stateSub;
    ros::Subscriber cmdSub;
    ros::Subscriber odometrySub;
    ros::Subscriber photonSub;
    ros::Publisher attPub;
    ros::Publisher thrPub;
    ros::Rate rate;


    Eigen::Quaternion<float> qPx;
    Eigen::Vector3f rPx;
    Eigen::Vector3f vPx;
    Eigen::Vector3f aPx;
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

    Eigen::Quaternion<float> q0;

    Status status;

    Logger logger;
    flightData logData;

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
    SimpleEstimator se;
    bool odometryReceived;
};
