#include <iostream>
#include <stdio.h>
#include <vector>

#include "ros/ros.h"

#include <Eigen/Dense>

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/Thrust.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"


enum Status{
    inited,
    feedbackReady,
    offboarded,
    armed,
    tookoff
};

class AttControl{
    
public:
    AttControl();
    void runTest();
    
private:    
    void subscribe();
    void initPubs();
    void initServs();
    void checkFeedback();
    void setOffboard();
    void arm();
    void takeoff();
    
    void imuCb(const sensor_msgs::Imu& msg);
    void posCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void stateCb(const mavros_msgs::State::ConstPtr& msg);
    void pubCtrl(float thr, const Eigen::Quaternion<float>& quat);
    
private:
    std::string imuTopicName;
    std::string posTopicName;
    std::string velTopicName;
    std::string stateTopicName;
    std::string setattTopicName;
    std::string setthrTopicName;
    std::string setmodeServiceName;
    std::string armServiceName;
    
    ros::NodeHandle nodeHandle;
    ros::ServiceClient modeService;
    ros::ServiceClient armService;
    ros::Subscriber imuSub;
    ros::Subscriber posSub;
    ros::Subscriber velSub;
    ros::Subscriber stateSub;
    
    ros::Publisher attPub;
    ros::Publisher thrPub;
    
    ros::Rate rate;
    
    Eigen::Quaternion<float> qPx;
    Eigen::Vector3d rPx;
    Eigen::Vector3d vPx;
    Eigen::Vector3d aPx;
    bool imuReady;
    bool posReady;
    bool velReady;
    bool armed;
    
    Status status;
};
