#include <iostream>
#include <stdio.h>
#include <vector>
#include <chrono>

#include <gazebo_msgs/ModelStates.h>
#include <Eigen/Dense>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "mavros_msgs/Thrust.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"

#include "Logger.hpp"
#include "Math.hpp"
#include "Params.h"


enum Status{
    begin,
    inited,
    sensorsReady,
    offboarded,
    armed,
    tookoff
};

class AttControl{

public:
    AttControl();
    void runTest();

private:

    bool init();
    void subscribe();
    void initPubs();
    void initServs();
    void writeLogData();

    void initPubsGz();
    void subscribeGz();

    bool checkFeedback();
    bool setOffboard();
    bool arm();
    bool takeoff();
    bool goToLocalPoint(const Eigen::Vector3f& r0);

    void pubCtrl(float thr, const Eigen::Quaternion<float>& quat);
    void pubFakeVisData();

    void imuCb(const sensor_msgs::Imu& msg);
    void posCb(const geometry_msgs::PoseStamped& msg);
    void velCb(const geometry_msgs::TwistStamped& msg);
    void stateCb(const mavros_msgs::State& msg);

    void modelStateCbGz(const gazebo_msgs::ModelStates& msg);

private:
    std::string imuTopicName;
    std::string posTopicName;
    std::string velTopicName;
    std::string stateTopicName;
    std::string setattTopicName;
    std::string setthrTopicName;
    std::string setmodeServiceName;
    std::string armServiceName;

    std::string visPoseTopicName;
    std::string visPoseCovTopicName;
    std::string visVelCovTopicName;

    std::string modelStateTopicNameGz;
    std::string setModelStateTopicNameGz;

    ros::NodeHandle nodeHandle;
    ros::ServiceClient modeService;
    ros::ServiceClient armService;
    ros::Subscriber imuSub;
    ros::Subscriber posSub;
    ros::Subscriber velSub;
    ros::Subscriber stateSub;

    ros::Subscriber modelStateSubGz;

    ros::Publisher attPub;
    ros::Publisher thrPub;

    ros::Publisher visPosePub;
    ros::Publisher visPoseCovPub;
    ros::Publisher visVelCovPub;

    ros::Rate rate;

    Eigen::Quaternion<float> qPx;
    Eigen::Vector3f rPx;
    Eigen::Vector3f vPx;
    Eigen::Vector3f aPx;
    bool imuReady;
    bool posReady;
    bool velReady;
    bool armed;
    mavros_msgs::State mavState;

    Eigen::Quaternion<float> qGz;
    Eigen::Vector3f rGz;
    Eigen::Vector3f vGz;
    Eigen::Vector3f aGz;
    Eigen::Vector3f oGz;

    Status status;

    Logger logger;
    FlightData logData;

    DifferenciatorVector3f thrustPidDiff;
    IntegratorVector3f thrustPidInt;

    std::chrono::high_resolution_clock::time_point initTime;
    uint64_t timeMs;
    uint64_t lastTickMs;
    uint64_t dTimeMs;
};
