#include "AttControl.h"

int QUENUE_DEPTH = 10;
int FEEDBACK_CHANELS_COUNT = 3;
std::string OFFBOARD("OFFBOARD");
std::string FRAME_ID("map");
float MIN_THRUST = 0.2;
float MAX_THRUST = 0.8;

AttControl::AttControl():
 imuTopicName("/mavros/imu/data")
,posTopicName("mavros/local_position/pose")
,velTopicName("/mavros/local_position/velocity")
,stateTopicName("/mavros/state")
,armServiceName("/mavros/cmd/arming")
,setmodeServiceName("/mavros/set_mode")
,setattTopicName("/mavros/setpoint_attitude/attitude")
,setthrTopicName("/mavros/setpoint_attitude/thrust")

,armed(false)
,imuReady(false)
,posReady(false)
,velReady(false)

,rate(50)

{
  status = Status::inited;
  
  subscribe();
  initPubs();
  initServs();
}

void AttControl::runTest()
{
    while (ros::ok){
        
        //printf("status: %d", status);
        //printf("imu: %d\n", imuReady);
        //printf("pos: %d\n", posReady);
        //printf("vel: %d\n", velReady);
        switch (status){
            case Status::inited:
                checkFeedback();
                printf("1\n");
                break;
            case Status::feedbackReady:
                printf("2\n");
                pubCtrl(MIN_THRUST, qPx);
                setOffboard();
                break;
            case Status::offboarded:
                printf("3\n");
                pubCtrl(MIN_THRUST, qPx);
                arm();
                break;
            case Status::armed:
                printf("4\n");
                takeoff();
                break;
        }
    ros::spinOnce();
    rate.sleep();
    }
}

void AttControl::checkFeedback()
{
    if (imuReady && posReady && velReady)
        status = Status::feedbackReady;
}

void AttControl::setOffboard()
{
    mavros_msgs::SetMode setModeMessage;
    setModeMessage.request.custom_mode = "OFFBOARD";
    
    bool res = modeService.call(setModeMessage) && setModeMessage.response.mode_sent;
    if(res)
        status = Status::offboarded;
}

void AttControl::arm()
{
    mavros_msgs::CommandBool armingMsg;
    armingMsg.request.value = true;
    
    bool res = armService.call(armingMsg) && armingMsg.response.success;
    if (res)
        status = Status::armed;
    
}

void AttControl::initServs()
{
    modeService = nodeHandle.serviceClient<mavros_msgs::SetMode>(setmodeServiceName);
    armService = nodeHandle.serviceClient<mavros_msgs::CommandBool>(armServiceName);
}

void AttControl::takeoff(){
    pubCtrl(MAX_THRUST, qPx);
}

void AttControl::subscribe()
{
  imuSub = nodeHandle.subscribe(imuTopicName, QUENUE_DEPTH, &AttControl::imuCb, this);
  posSub = nodeHandle.subscribe(posTopicName, QUENUE_DEPTH, &AttControl::posCb, this);
  velSub = nodeHandle.subscribe(velTopicName, QUENUE_DEPTH, &AttControl::velCb, this);
  stateSub = nodeHandle.subscribe(stateTopicName, QUENUE_DEPTH, &AttControl::stateCb, this);
}

void AttControl::initPubs()
{
    attPub = nodeHandle.advertise<geometry_msgs::PoseStamped>(setattTopicName, QUENUE_DEPTH);
    thrPub = nodeHandle.advertise<mavros_msgs::Thrust>(setthrTopicName, QUENUE_DEPTH);
}

void AttControl::pubCtrl(float thr, const Eigen::Quaternion<float>& quat)
{
    geometry_msgs::PoseStamped quatMsg;
    quatMsg.header.stamp = ros::Time::now();
    quatMsg.header.frame_id = FRAME_ID;
    quatMsg.pose.position.x = 0.;
    quatMsg.pose.position.y = 0.;
    quatMsg.pose.position.z = 0.;
    quatMsg.pose.orientation.w = quat.w();
    quatMsg.pose.orientation.x = quat.x();
    quatMsg.pose.orientation.y = quat.y();
    quatMsg.pose.orientation.z = quat.z();
    
    mavros_msgs::Thrust thrMsg;
    thrMsg.header.stamp =  ros::Time::now();
    thrMsg.header.frame_id = FRAME_ID;
    thrMsg.thrust = thr;
    
    attPub.publish(quatMsg);
    thrPub.publish(thrMsg);
}

void AttControl::imuCb(const sensor_msgs::Imu& msg)
{
    aPx = Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    qPx = Eigen::Quaternion<float>(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    imuReady = true;
}

void AttControl::velCb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    vPx = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    velReady = true;
}

void AttControl::posCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    rPx = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    posReady = true;
}

void AttControl::stateCb(const mavros_msgs::State::ConstPtr& msg)
{
    armed = msg->armed;
}









