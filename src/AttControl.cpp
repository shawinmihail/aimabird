#include "AttControl.h"

AttControl::AttControl():
 imuTopicName("/mavros/imu/data")
,posTopicName("mavros/local_position/pose")
,velTopicName("/mavros/local_position/velocity")
,stateTopicName("/mavros/state")
,armServiceName("/mavros/cmd/arming")
,setmodeServiceName("/mavros/set_mode")
,setattTopicName("/mavros/setpoint_attitude/attitude")
,setthrTopicName("/mavros/setpoint_attitude/thrust")
,cmdTopicName("/mavros/setpoint_attitude/thrust")
,odometryTopicName("/rtabmap/odom")
,photonTopicName("/aimabird_node/photonCmd")

,status(Status::begin)
,gIinited(false)
,imuReady(false)
,posReady(false)
,velReady(false)

,rate(33)
,initTime(std::chrono::high_resolution_clock::now())
,timeMs(0)
,lastTickMs(0)
,dTimeMs(0)
,waitCounter(0)
{}


void AttControl::main()
{

}

void AttControl::clockTick()
{
        timeMs =  std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::high_resolution_clock::now() - initTime).count();
        dTimeMs = timeMs - lastTickMs;
        lastTickMs = timeMs;
}

void AttControl::prepareToFly()
{
    while (ros::ok){

        clockTick();
        bool done = false;
        if (!waiting()) {
            switch (status){

                case Status::begin:
                    done = init();
                    if (done){
                        logger.addEvent("AttCtrl: inited");
                        status = Status::inited;
                    }
                    break;

                case Status::inited:
                    done = checkFeedback();
                    if (done){
                        logger.addEvent("AttCtrl: sensors ready");
                        status = Status::sensorsReady;
                    }
                    else{
                        logger.addEvent("AttCtrl: sensors not ready yet");
                        wait(1);
                    }
                    break;

                case Status::sensorsReady:
                    done = setOffboard();
                    done = done & mavState.mode == OFFBOARD;
                    if (done){
                        logger.addEvent("AttCtrl: offboarded");
                        status = Status::offboarded;
                    }
                    else {
                        logger.addEvent("AttCtrl: can't offboard yet ...");
                        wait(1);
                    }
                    break;

                case Status::offboarded:
                    done = arm();
                    done = done & mavState.armed;
                    if (done){
                        logger.addEvent("AttCtrl: armed");
                        status = Status::armed;
                    }
                    else{
                        logger.addEvent("AttCtrl: can't arm yet ...");
                        wait(1);
                    }
                    break;

                case Status::armed:
                    done = goToLocalPoint(Eigen::Vector3f (0.f, 0.f, 5.0f));
                    if (done) {
                        logger.addEvent("AttCtrl: tookoff");
                        status = Status::tookoff;
                    }
                    break;

                case Status::tookoff:
                    done = goToLocalPoint(Eigen::Vector3f (0.f, 0.f, 2.0f));
                    if (done) {
                        logger.addEvent("AttCtrl: stabilized");
                        //status = Status::stabilized;
                    }
                    break;

                case Status::stabilized:
                    done = accomulateVelocityWithImu(Eigen::Vector3f (0.f, 0.f, 0.0f));
                    if (done) {
                        logger.addEvent("AttCtrl: stabilized");
                        status = Status::stabilized;
                    }
                    break;
            }
        }

        if (status >= Status::armed){
            writeLogData();
        }
        if (status >= Status::sensorsReady && status < Status::armed){
            sendIdling(); // send min thrust before flight to able offboard / arm
        }
        if (status < Status::armed){ // write empty data before fill it in control funcs
            logData.debug1 = 0.f;
            logData.debug2 = 0.f;
            logData.debug3 = 0.f;
            logData.debug4 = 0.f;
            q0 = Eigen::Quaternion<float>();
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void AttControl::checkSensors()
{
    bool done = checkFeedback();
    if (done){
        logger.addEvent("AttCtrl: sensors ready", true);
    }
    else{
        logger.addEvent("AttCtrl: sensors not ready yet");
        wait(3);
    }
}

void AttControl::wait(int s)
{
    waitCounter = s * CTRL_RATE;
}

bool AttControl::waiting()
{
    if (waitCounter > 0){
        waitCounter --;
        return true;
    }
    return false;
}

bool AttControl::init()
{
  subscribe();
  initPubs();
  initServs();
  return true;
}

bool AttControl::checkFeedback()
{
    if (imuReady && posReady && velReady) {
        status = Status::sensorsReady;
    }
}

bool AttControl::setOffboard()
{
    mavros_msgs::SetMode setModeMessage;
    setModeMessage.request.custom_mode = "OFFBOARD";

    bool res = modeService.call(setModeMessage) && setModeMessage.response.mode_sent;
    if(res){
        return true;
    }
    return false;
}

bool AttControl::arm()
{
    mavros_msgs::CommandBool armingMsg;
    armingMsg.request.value = true;

    bool res = armService.call(armingMsg) && armingMsg.response.success;
    return res;
}

bool AttControl::takeoff(){ // testing

//     Eigen::Vector3f r0(0., 0., 10.);
//     Eigen::Vector3f dr = r0 - rPx;
//     float dt = cutAbsFloat(dTimeMs / 1e3f, MIN_TICK_FOR_CALCS);
//
//     thrust = cutTwosidesFloat(thrust, MIN_THRUST, MAX_THRUST);
//     pubCtrl(thrust, qPx);
}

bool AttControl::goToLocalPoint(Eigen::Vector3f r0)
{
    float dt = cutAbsFloat(dTimeMs / 1e3f, MIN_TICK_FOR_CALCS);

    Eigen::Vector3f v = vOd;
    Eigen::Vector3f r = rOd;
    Eigen::Vector3f dr = r0 - r;
    Eigen::Vector3f dv = -v;
    dr = cutAbsVector3f(dr, Eigen::Vector3f(GO_LOCAL_INPUT_LIM_H, GO_LOCAL_INPUT_LIM_H, GO_LOCAL_INPUT_LIM_V));

    /** **/
    Eigen::Vector3f P = Eigen::Vector3f(GO_LOCAL_PID_P_H * dr[0], GO_LOCAL_PID_P_H * dr[1], GO_LOCAL_PID_P_V * dr[2]);
    Eigen::Vector3f I;
    if(dr[2] * v[2] < GO_LOCAL_ANTIWINDUP_PARAM_V) {
        I =  goLocalIr.get(Eigen::Vector3f(0.f, 0.f, GO_LOCAL_PID_I_V * dr[2]), dt);
    }
    else{
        I = goLocalIr.get();
    }
    I = cutAbsVector3f(I, GO_LOCAL_PID_I_LIM_V);
    Eigen::Vector3f D = Eigen::Vector3f(GO_LOCAL_PID_D_H * dv[0], GO_LOCAL_PID_D_H * dv[1], GO_LOCAL_PID_D_V * dv[2]);
    /** **/

    Eigen::Vector3f regOutVec = P + I + D + Eigen::Vector3f(0., 0., FREE_FALL_ACC_ABS);
    float yaw = 0*PI/2.f;
    q0 = quatFromDirAndYaw(regOutVec, yaw, 5.f*PI/ 180.f);
    float thrust = regOutVec.norm() / TW / FREE_FALL_ACC_ABS;
    thrust = cutTwosidesFloat(thrust, MIN_THRUST, MAX_THRUST);
    pubCtrl(thrust, q0);

    logData.debug1 = regOutVec[0];
    logData.debug2 = regOutVec[1];
    logData.debug3 = regOutVec[2];

    if (isZeroVector3f(dr, POSE_EPS)){
        return true;
    }
    return false;
}

bool AttControl::goRelativePosition(Eigen::Vector3f r0)
{
    Eigen::Vector3f r = rPx;
    r0 = r + r0;
    return goToLocalPoint(r0);
}


bool AttControl::goWithAcc(Eigen::Vector3f a0) // TODO remake
{
    Eigen::Vector3f a = aPxClearI;

    Eigen::Vector3f da =  a0 - a;
    float dt = cutAbsFloat(dTimeMs / 1e3f, MIN_TICK_FOR_CALCS);
    std::cout <<"dt_: "<< dt << std::endl;
    //Eigen::Vector3f integralPart = 1.f * cutAbsVector3f(goAccIr.get(da, dt), 1.f);
    Eigen::Vector3f integralPart = goAccIr.get(da, dt);
    Eigen::Vector3f regOutVec = 1.f * da + integralPart + Eigen::Vector3f(0., 0., FREE_FALL_ACC_ABS);

    Eigen::Quaternion<float> q0 = Eigen::Quaternion<float>(); // TODO assert q without flips
    q0.setFromTwoVectors(UNIT_Z, regOutVec); // TODO calc yaw and pitch/roll quat
    q0.normalize();

    float thrust = regOutVec.norm() / TW / FREE_FALL_ACC_ABS; //TODO check use norm for thrust
    thrust = cutTwosidesFloat(thrust, MIN_THRUST, MAX_THRUST);

    pubCtrl(thrust, q0);
    return true;
}

bool AttControl::accomulateVelocityWithImu(Eigen::Vector3f v0) // TODO how remove small oscilations?
{   /* d time for calc */
    float dt = cutAbsFloat(dTimeMs / 1e3f, MIN_TICK_FOR_CALCS);

    /* velocity estimation */
    Eigen::Vector3f v = accamulateVelIrEstimator.get(aPxClearI, dt);
    Eigen::Vector3f dv = v0 - v;

    /* velocity regulator */
    Eigen::Vector3f integralPart = ACCUM_VEL_PID_I * accamulateVelIr.get(dv, dt);
    integralPart = cutAbsVector3f(Eigen::Vector3f(0.f, 0.f, integralPart[2]), ACCUM_VEL_PID_I_ERROR_LIM); // integral ONLY for altitude

    Eigen::Vector3f regOutVec = cutAbsVector3f(ACCUM_VEL_PID_P * dv, ACCUM_VEL_PID_P_ERROR_LIM)
    + integralPart
    + Eigen::Vector3f(0., 0., 1.f / TW);

    float yaw = 0*PI/2.f;
    q0 = quatFromDirAndYaw(regOutVec, yaw, 30.f*PI/ 180.f);

    /* reg + hower thrust */
    float thrust = regOutVec.dot(UNIT_Z);

    /* pub */
    thrust = cutTwosidesFloat(thrust, MIN_THRUST, MAX_THRUST);
    logData.debug1 = v[0];
    logData.debug2 = v[1];
    logData.debug3 = v[2];
    logData.debug4 = thrust;
    pubCtrl(thrust, q0);

    if (isZeroVector3f(dv, EST_VEL_EPS)){
        return true;
    }
    return false;
}

bool AttControl::accomulateVelocityWithImu(float vz) // TODO dont ready yet
{   /* d time for calc */
//     float dt = cutAbsFloat(dTimeMs / 1e3f, MIN_TICK_FOR_CALCS);
//
//     /* velocity estimation */
//     Eigen::Vector3f v = accamulateVelIrEstimator.get(aPxClearI, dt);
//     Eigen::Vector3f dv = (vz - v[2]);
//
//     /* velocity regulator */
//     Eigen::Vector3f integralPart = ACCUM_VEL_PID_I * accamulateVelIr.get(dv, dt);
//     integralPart = cutAbsFloat(integralPart[2], ACCUM_VEL_PID_I_ERROR_LIM); // integral ONLY for altitude
//
//     Eigen::Vector3f regOutVec = cutAbsVector3f(ACCUM_VEL_PID_P * dv, ACCUM_VEL_PID_P_ERROR_LIM)
//     + integralPart
//     + Eigen::Vector3f(0., 0., 1.f / TW);
//
//     float yaw = 0*PI/2.f;
//     q0 = quatFromDirAndYaw(regOutVec, yaw, 30.f*PI/ 180.f);
//
//     /* reg + hower thrust */
//     float thrust = regOutVec.dot(UNIT_Z);
//
//     /* pub */
//     thrust = cutTwosidesFloat(thrust, MIN_THRUST, MAX_THRUST);
//     logData.debug1 = v[0];
//     logData.debug2 = v[1];
//     logData.debug3 = v[2];
//     logData.debug4 = thrust;
//     pubCtrl(thrust, q0);
//
//     if (isZeroVector3f(dv, EST_VEL_EPS)){
//         return true;
//     }
    return false;
}

void AttControl::initServs()
{
    modeService = nodeHandle.serviceClient<mavros_msgs::SetMode>(setmodeServiceName);
    armService = nodeHandle.serviceClient<mavros_msgs::CommandBool>(armServiceName);
}

void AttControl::subscribe()
{
  imuSub = nodeHandle.subscribe(imuTopicName, QUENUE_DEPTH, &AttControl::imuCb, this);
  posSub = nodeHandle.subscribe(posTopicName, QUENUE_DEPTH, &AttControl::posCb, this);
  velSub = nodeHandle.subscribe(velTopicName, QUENUE_DEPTH, &AttControl::velCb, this);
  stateSub = nodeHandle.subscribe(stateTopicName, QUENUE_DEPTH, &AttControl::stateCb, this);
  odometrySub = nodeHandle.subscribe(odometryTopicName, QUENUE_DEPTH, &AttControl::odometryCb, this);
  photonSub = nodeHandle.subscribe(photonTopicName, QUENUE_DEPTH, &AttControl::photonCmdCb, this);
}

void AttControl::initPubs()
{
    attPub = nodeHandle.advertise<geometry_msgs::PoseStamped>(setattTopicName, QUENUE_DEPTH);
    thrPub = nodeHandle.advertise<mavros_msgs::Thrust>(setthrTopicName, QUENUE_DEPTH);
}

bool AttControl::sendIdling()
{
    pubCtrl(MIN_THRUST, qPx);
}

void AttControl::pubCtrl(float thr, const Eigen::Quaternion<float>& quat)
{
    ros::Time now = ros::Time::now();
    geometry_msgs::PoseStamped quatMsg;
    quatMsg.header.stamp = now;
    quatMsg.header.frame_id = FRAME_ID;
    quatMsg.pose.position.x = 0.;
    quatMsg.pose.position.y = 0.;
    quatMsg.pose.position.z = 0.;
    quatMsg.pose.orientation.w = quat.w();
    quatMsg.pose.orientation.x = quat.x();
    quatMsg.pose.orientation.y = quat.y();
    quatMsg.pose.orientation.z = quat.z();

    mavros_msgs::Thrust thrMsg;
    thrMsg.header.stamp =  now;
    thrMsg.header.frame_id = FRAME_ID;
    thrMsg.thrust = thr;

    thrPub.publish(thrMsg);
    attPub.publish(quatMsg);
}

void AttControl::imuCb(const sensor_msgs::Imu& msg)
{
    aPx = Eigen::Vector3f(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    qPx = Eigen::Quaternion<float>(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);

    if (!gIinited){
        gIestimated =  Eigen::Vector3f(aPx);
        gIestimated = quatRotate(qPx.inverse(), gIestimated);
        gIinited = true;
    }
    aPxClearI = quatRotate(qPx.inverse(), aPx) - gIestimated;
    imuReady = true;
}

void AttControl::velCb(const geometry_msgs::TwistStamped& msg)
{
    vPx = Eigen::Vector3f(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
    velReady = true;
}

void AttControl::posCb(const geometry_msgs::PoseStamped& msg)
{
    rPx = Eigen::Vector3f(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    posReady = true;
}

void AttControl::stateCb(const mavros_msgs::State& msg)
{
    mavState = msg;
}

void AttControl::odometryCb(const nav_msgs::Odometry& msg)
{
    qOd = Eigen::Quaternion<float>(msg.pose.pose.orientation.w,
                                   msg.pose.pose.orientation.x,
                                   msg.pose.pose.orientation.y,
                                   msg.pose.pose.orientation.z);
    rOd = Eigen::Vector3f(msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.position.z);
    vOd = Eigen::Vector3f(msg.twist.twist.linear.x,
                          msg.twist.twist.linear.y,
                          msg.twist.twist.linear.z);
    oOd = Eigen::Vector3f(msg.twist.twist.angular.x,
                          msg.twist.twist.angular.y,
                          msg.twist.twist.angular.z);
}

void AttControl::photonCmdCb(const std_msgs::UInt32MultiArray& msg)
{
//     Command cmd = msg.data[0];
//     switch(cmd){
//         case Command::checkSensors:
//             break;
//         case Command::prepareToFly:
//             prepareToFly();
//             break;
//         case Command::takeoff:
//             int alt = msg.data[1];
//             Eigen::Vector3f r0(0, 0, (float)alt);
//             goToLocalPoint(r0);
//             break;
//         case Command::runTestFlight:
//             break;
//         case Command::land:
//             break;
//     }
}


void AttControl::writeLogData()
{
    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()
    );
    logData.timeMs = ms.count();

//     logData.timeMs = uint64_t(ros::Time::now().toSec() * 1e3);

    logData.qPx = qPx;
    logData.rPx = rPx;
    logData.vPx = vPx;
    logData.aPx = aPxClearI;

    logData.qGz = qOd; // TODO: rename log fields
    logData.rGz = rOd;
    logData.vGz = vOd;

    logData.q0 = q0;

    logger.addData(logData);
}

