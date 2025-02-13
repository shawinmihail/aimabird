#include "AttControl.h"

void AttControl::test()
{
//     Eigen::Quaternion<float> q(0.707072, 0.000500823, -0.000147876, -0.707141);
//     //q.normalize();
//
//     auto e = quat2Eul(q);
//
//     std::cout << "E\n" << e << std::endl;
// //     printQuat(q);
    Eigen::Vector3f v(0.f / 0.f, 1.f / 1.f, 0.f/0.f);
    std::cout << isNanVect(v) << std::endl;
    printVect(v);
}


AttControl::AttControl():
 imuTopicName("/mavros/imu/data")
,posTopicName("mavros/local_position/pose")
,velTopicName("/mavros/local_position/velocity")
,stateTopicName("/mavros/state")
,armServiceName("/mavros/cmd/arming")
,setmodeServiceName("/mavros/set_mode")
,setattTopicName("/mavros/setpoint_attitude/attitude")
,setthrTopicName("/mavros/setpoint_attitude/thrust")
,cmdVelTopicName("/mavros/setpoint_attitude/cmd_vel")
//,odometryTopicName("odometry/filtered")
,odometryTopicName("rtabmap/odom")
,photonCmdTopicName("/aimabird_control/photonCmd")
,photonTmTopicName("/aimabird_control/photonTm")
,useQuatParamTopicName("/mavros/setpoint_attitude/use_quaternion")
,lidTopicName("/leddar_vu8/read_data")
,altTopicName("/sf30/range")

,status(Status::begin)
,imuReady(false)
,posReady(false)
,velReady(false)
,odometryReady(false)
,lidReady(false)
,altReady(false)
,aimAccepted(false)

,rate(CTRL_RATE)
,initTime(std::chrono::high_resolution_clock::now())
,timeMs(0)
,lastTickMs(0)
,dTimeMs(0)
,waitCounter(0)

,yawRateInput(0.f)
,yawPointerForRotate(0.f)
,yawRateCtrlMode(false)

,ekfAlt(altimetr)
,altSerDiff(6)
,ekfLid(lidar)
,lidSerDiff(25)
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

float AttControl::getLastTickDuration()
{
    return fmin( fmax(dTimeMs / 1e3f, 0.2f/CTRL_RATE), 5.0f/CTRL_RATE);
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
                        logger.addEvent("AttCtrl: initialize...", timeMs);
                        status = Status::inited;
                    }
                    break;

                case Status::inited:
                    done = checkFeedback();
                    if (done){
                        logger.addEvent("AttCtrl: sensors ready", timeMs);
                        status = Status::sensorsReady;
                    }
                    else{
                        logger.addEvent("AttCtrl: waiting feedback ...", timeMs);
                        wait(1);
                    }
                    break;

                case Status::sensorsReady:
                    sendIdling();
                    done = setOffboard();
                    done = done & mavState.mode == OFFBOARD;
                    if (done){
                        logger.addEvent("AttCtrl: offboarded", timeMs);
                        status = Status::offboarded;
                    }
                    else {
                        logger.addEvent("AttCtrl: offboarding ...", timeMs);
                        wait(1);
                    }
                    break;

                case Status::offboarded:
                    sendIdling();
                    done = arm();
                    done = done & mavState.armed;
                    if (done){
                        //logger.addEvent("AttCtrl: armed", timeMs);
                        //status = Status::armed;
                        std::cout << "send idling" << std::endl;
                    }
                    else{
                        logger.addEvent("AttCtrl: arming ...", timeMs);
                        wait(1);
                    }
                    break;

                case Status::armed:
                    done = goToLocalPoint(Eigen::Vector3f (0.f, 0.f, 0.6f));
                    if (done) {
                        flightTimer.start(5000);
                    }
                    if (flightTimer.timeout()){
                        logger.addEvent("AttCtrl: tookoff", timeMs);
                        status = Status::tookoff;
                    }
                    break;

                case Status::tookoff:
                    done = goToLocalPoint(Eigen::Vector3f (0.f, 0.f, -POSE_EPS));
                    if (done) {
                        logger.addEvent("AttCtrl: landed", timeMs);
                        status = Status::landed;
                    }
//                     if (aimAccepted){
//                         //done = goWithVelocity2D(5.f, rInput, yawRateInput);
//                     }
//                     else{
//                         done = accumulateWithImu(Eigen::Vector3f (0.f, 0.f,-1.f * EST_VEL_EPS));
//                     }
                    break;

                case Status::landed:
                    status = Status::stopped;
                    break;

                case Status::stopped:
                    done = disarm();
                    if (done) {
                        logger.addEvent("AttCtrl: stopped", timeMs);
                        status = Status::stopped;
                        return;
                    }
                    break;
            }
        }

        if (status >= Status::sensorsReady){
            estimateState();
            writeLogData();
        }
        if (status >= Status::armed){
            sendPhotonTm();
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

    bool check = imuReady;
    if (USE_GPS) {check = check && velReady && posReady;}
    if (USE_ODOMETRY) {check = check && odometryReady;}
    if (USE_ALTIMETR) {check = check && altReady;}
    if (USE_LIDAR) {check = check && lidReady;}
    if (check) {
        return true;
    }
    return false;
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

bool AttControl::disarm()
{
    mavros_msgs::CommandBool armingMsg;
    armingMsg.request.value = false;

    bool res = armService.call(armingMsg) && armingMsg.response.success;
    return res;
}


Eigen::Vector3f AttControl::getRegVector(Eigen::Vector3f dr, Vector3f dv, float dt){
    dr = cutAbsVector3f(dr, Eigen::Vector3f(GO_LOCAL_INPUT_LIM_H, GO_LOCAL_INPUT_LIM_H, GO_LOCAL_INPUT_LIM_V));
    if (dr[2] < 0){
        dr[2] = dr[2] / 3.f;
    }

    Eigen::Vector3f P = Eigen::Vector3f(GO_LOCAL_PID_P_H * dr[0], GO_LOCAL_PID_P_H * dr[1], GO_LOCAL_PID_P_V * dr[2]);
    Eigen::Vector3f I;
    if(-dr[2] * dv[2] < GO_LOCAL_ANTIWINDUP_PARAM_V && dr[2] > 0) {
        I =  goLocalIr.get(Eigen::Vector3f(0.f, 0.f, GO_LOCAL_PID_I_V * dr[2]), dt);
    }
    else{
        I = goLocalIr.get();
    }
    Eigen::Vector3f D = Eigen::Vector3f(GO_LOCAL_PID_D_H * dv[0], GO_LOCAL_PID_D_H * dv[1], GO_LOCAL_PID_D_V * dv[2]);
    Eigen::Vector3f DI = cutAbsVector3f(goLocalVelIr.get
    (
        Eigen::Vector3f(0*GO_LOCAL_PID_DI_H * dv[0], 0*GO_LOCAL_PID_DI_H * dv[1], GO_LOCAL_PID_DI_V * dv[2]), dt),
        GO_LOCAL_PID_DI_LIM
    );
//     printVect(I, "I");
//     printVect(D, "D");
//     printVect(P, "P");
//     printVect(DI, "DI");
    Eigen::Vector3f regOutVec = P + I + D + Eigen::Vector3f(0., 0., FREE_FALL_ACC_ABS);
    return regOutVec;
}

bool AttControl::takeoff(float h){ // testing

    float dt = getLastTickDuration();
    Eigen::Quaternion<float> q = qPx;
    Eigen::Vector3f e = quat2Eul(q);
    float yaw = e[2];

    Eigen::Vector3f r;
    Eigen::Vector3f v;
    if (USE_ODOMETRY && USE_ALTIMETR){
        v = vEs;
    }
    else if (USE_GPS && USE_ALTIMETR){
        v = vPx;
    }
    else {
        emergencyLand();
    }

    Eigen::Vector3f dr = Eigen::Vector3f(0.f, 0.f, h - rAltEs);
    Eigen::Vector3f dv = -v;
    Eigen::Vector3f reg = getRegVector(dr, dv, dt);
    float thrust = reg.norm() / TW / FREE_FALL_ACC_ABS;
    thrust = cutTwosidesFloat(thrust, MIN_THRUST, MAX_THRUST);

    Eigen::Quaternion<float> qCtrlOut = quatFromDirAndYaw(reg, yaw, MAX_BOW);
    qCtrlOut = qCtrlOut * qPxInit;
    if (USE_QUATERNION){
        pubCtrl(thrust, qCtrlOut);
    }
    else {
        Eigen::Vector3f o0(0.f, 0.f, 0.f);
        Eigen::Vector3f oCtrlOut = getTwistVector(qCtrlOut, o0);
        pubCtrl(thrust, oCtrlOut);
    }

    if (isZeroVector3f(dr, POSE_EPS)){
        return true;
    }
    return false;
}

Vector3f AttControl::getTwistVector(const Eigen::Quaternion<float>& q0, const Eigen::Vector3f& w0)
{
    Eigen::Vector3f e0 = quat2Eul(q0);
    Eigen::Vector3f e = quat2Eul(qPx);
    Eigen::Vector3f de = e0 - e;
    de = cutAbsVector3f(de, 0.10f);
    Eigen::Vector3f dw = w0 - oPx;
    Eigen::Vector3f w = 4.0f * de + 1.0 * dw;
    return w;
}


bool AttControl::goToLocalPoint(Eigen::Vector3f r0)
{
    float dt = getLastTickDuration();
    Eigen::Quaternion<float> q = qPx;
    Eigen::Vector3f e = quat2Eul(q);
    float yaw = e[2];

    Eigen::Vector3f r;
    Eigen::Vector3f v;
    if (USE_ODOMETRY){
        r = rEs;
        v = vEs;
    }
    else if (USE_GPS){
        r = rPx;
        v = vPx;
    }
    else {
        logger.addEvent("wrong conf", timeMs);
        return false;
    }

    Eigen::Vector3f dr = r0 - r;
    dr = cutAbsVector3f(dr, Eigen::Vector3f(GO_LOCAL_INPUT_LIM_H, GO_LOCAL_INPUT_LIM_H, GO_LOCAL_INPUT_LIM_V));
    Eigen::Vector3f dv = -v;
    Eigen::Vector3f reg = getRegVector(dr, dv, dt);
    float thrust = reg.norm() / TW / FREE_FALL_ACC_ABS;
    thrust = cutTwosidesFloat(thrust, MIN_THRUST, MAX_THRUST);

    Eigen::Quaternion<float> qCtrlOut = quatFromDirAndYaw(reg, yaw, MAX_BOW);
    qCtrlOut = qCtrlOut * qPxInit;
    if (USE_QUATERNION){
        pubCtrl(thrust, qCtrlOut);
    }
    else {
        logger.addEvent("wrong conf", timeMs);
        return false;
    }

    if (isZeroVector3f(dr, POSE_EPS)){
        return true;
    }
    return false;
}

// bool AttControl::goWithVelocity2D(float h, Eigen::Vector3f v0, float yawRate){ // v in body frame
//
//     float dt = getLastTickDuration();
//     Eigen::Quaternion<float> q = qPx;
//
//     Eigen::Vector3f r;
//     Eigen::Vector3f v;
//     if (USE_ODOMETRY){
//         r = rEs;
//         v = vEs; //
//     }
//     else{
//         r = rPx;
//         v = vPx;
//     }
//
//     Eigen::Vector3f e = quat2Eul(q);
//     float yaw = e[2];
//
//     Eigen::Quaternion<float> qYaw(cos(yaw/2.f), 0.f, 0.f, sin(yaw/2.f));
//     Eigen::Vector3f v0I = quatRotate(qYaw.inverse(), v0);
//     Eigen::Vector3f dr(0.f, 0.f, h-r[2]);
//     Eigen::Vector3f dv = v0I - v;
//
//     Eigen::Vector3f reg = getRegVector(dr, dv, dt, v);
//
//     if (!yawRateCtrlMode){
//         yawRateCtrlMode = true;
//         yawPointerForRotate = yaw;
//     }
//
//     //yawRate = 0.3f * sin(timeMs / 1e3f / 4.f);
//     yawPointerForRotate += yawRate * dt;
//     //yawPointerForRotate = modFloat(yawPointerForRotate + PI, 2 * PI) - PI;
//
//     Eigen::Quaternion<float> qCtrlOut = quatFromDirAndYaw(reg, yawPointerForRotate, MAX_BOW);
//     qCtrlOut = qCtrlOut * qPxInit;
//     float thrust = reg.norm() / TW / FREE_FALL_ACC_ABS;
//     thrust = cutTwosidesFloat(thrust, MIN_THRUST, MAX_THRUST);
//     pubCtrl(thrust, qCtrlOut);
//     return true;
// }

bool AttControl::goWithVelocity(Eigen::Vector3f v0,  float yaw0) // v in ENU frame
{
    float dt = getLastTickDuration();
    Eigen::Quaternion<float> q = qPx;
    Eigen::Vector3f e = quat2Eul(q);
    float yaw = e[2];

    Eigen::Vector3f r;
    Eigen::Vector3f v;
    if (USE_ODOMETRY){
        r = rOd;
        v = vOd; //
    }
    else{
        r = rPx;
        v = vPx;
    }

    if (USE_LIDAR){
        if (rLidEs < EMERG_LID_VALUE){
            v = Eigen::Vector3f(vLidEs, v[1], v[2]);
            v0 = Eigen::Vector3f(0.f, 0.f, 0.f);
            logger.addEvent("LIDAR: SO CLOSE", timeMs);
        }
    }

    Eigen::Vector3f dr = v0 - v;
    dr = quatRotate(qPx.inverse(), dr);

    Eigen::Vector3f dv = v0 - v;
    dv = quatRotate(qPx.inverse(), dv);
    Eigen::Vector3f reg = getRegVector(dr, dv, dt);
    yaw0 = yawManager.get(yaw, yaw0, dt);

    Eigen::Quaternion<float> qCtrlOut = quatFromDirAndYaw(reg, yaw0, MAX_BOW);
    qCtrlOut = qCtrlOut * qPxInit;
    float thrust = reg.norm() / TW / FREE_FALL_ACC_ABS;
    thrust = cutTwosidesFloat(thrust, MIN_THRUST, MAX_THRUST);
    pubCtrl(thrust, qCtrlOut);
    return true;
}

bool AttControl::emergencyLand()
{
    if (USE_ALTIMETR && altReady){
        float dt = getLastTickDuration();
        Eigen::Quaternion<float> q = qPx;
        Eigen::Vector3f dr(0.f, 0.f, 0.f);
        float vel0 = EMERG_LAND_VEL;
        if (rAltEs < EMERG_ALT_VALUE){
            vel0 = vel0 /  2.f;
        }
        Eigen::Vector3f dv(0.f, 0.f, vel0 - vAltEs);
        Eigen::Vector3f reg = getRegVector(dr, dv, dt);
        Eigen::Vector3f e = quat2Eul(q);
        float yaw = e[2];
        Eigen::Quaternion<float> qCtrlOut = quatFromDirAndYaw(reg, yaw, MAX_BOW);
        qCtrlOut = qCtrlOut * qPxInit;
        float thrust = reg.norm() / TW / FREE_FALL_ACC_ABS;
        thrust = cutTwosidesFloat(thrust, MIN_THRUST, MAX_THRUST);
        pubCtrl(thrust, qCtrlOut);
        return true;
    }
}

bool AttControl::goWithAcc(Eigen::Vector3f a0) // TODO remake
{
//     Eigen::Vector3f a = aPxClearI;
//
//     Eigen::Vector3f da =  a0 - a;
//     float dt = cutAbsFloat(dTimeMs / 1e3f, MIN_TICK_FOR_CALCS);
//     //Eigen::Vector3f integralPart = 1.f * cutAbsVector3f(goAccIr.get(da, dt), 1.f);
//     Eigen::Vector3f integralPart = goAccIr.get(da, dt);
//     Eigen::Vector3f regOutVec = 1.f * da + integralPart + Eigen::Vector3f(0., 0., FREE_FALL_ACC_ABS);
//
//     Eigen::Quaternion<float> q0 = Eigen::Quaternion<float>(); // TODO assert q without flips
//     q0.setFromTwoVectors(UNIT_Z, regOutVec); // TODO calc yaw and pitch/roll quat
//     q0.normalize();
//
//     float thrust = regOutVec.norm() / TW / FREE_FALL_ACC_ABS; //TODO check use norm for thrust
//     thrust = cutTwosidesFloat(thrust, MIN_THRUST, MAX_THRUST);
//
//     pubCtrl(thrust, q0);
    return true;
}

bool AttControl::accumulateWithImu(const Eigen::Vector3f& v0, const Eigen::Vector3f& p0)
{
    float dt = getLastTickDuration();
    Eigen::Vector3f v = accumulateVelIrEstimator.get(aPxClearI, dt);
    Eigen::Vector3f p = accumulatePosIrEstimator.get(v, dt);
    Eigen::Vector3f dp = p0 - p;
    Eigen::Vector3f dv = v0 - v;
    Eigen::Vector3f e = quat2Eul(qPx);
    float yaw = e[2];

    Eigen::Vector3f P = cutAbsVector3f(ACCUM_VEL_PID_P * dv, ACCUM_VEL_PID_P_ERROR_LIM);
    Eigen::Vector3f I;
    if (fabs(aPxClearI[2]) < 0.2f){
        I = ACCUM_VEL_PID_I * accumulateVelIr.get(dv, dt);
    }
    else{
        I = ACCUM_VEL_PID_I * accumulateVelIr.get();
    }
    I = cutAbsVector3f(Eigen::Vector3f(0.f, 0.f, I[2]), ACCUM_VEL_PID_I_ERROR_LIM); // integral ONLY for altitude
    Eigen::Vector3f regOutVec = P + I;

    Eigen::Quaternion<float> qCtrlOut = quatFromDirAndYaw(regOutVec, yaw, MAX_BOW);
    qCtrlOut = qCtrlOut * qPxInit;
    float thrust = regOutVec.dot(UNIT_Z);

    if (aPxClearI.norm() > EMERG_ACC_LIM){
        status = Status::stopped;
        return false;
    };

    thrust = cutTwosidesFloat(thrust, MIN_THRUST, MAX_THRUST);
    pubCtrl(thrust, qCtrlOut);

//     if (/*isZeroVector3f(dv, EST_VEL_EPS) ||*/ isZeroVector3f(dp, EST_VEL_EPS)){
//         return true;
//     }
    if (dp[2] < 0.f){return true;}
    return false;
}

bool AttControl::stop()
{
    status = Status::stopped;
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
  lidSub = nodeHandle.subscribe(lidTopicName, QUENUE_DEPTH, &AttControl::lidCb, this);
  altSub = nodeHandle.subscribe(altTopicName, QUENUE_DEPTH, &AttControl::altCb, this);
  photonCmdSub = nodeHandle.subscribe(photonCmdTopicName, QUENUE_DEPTH, &AttControl::photonCmdCb, this);
}

void AttControl::initPubs()
{
    if (USE_QUATERNION){
        nodeHandle.setParam(useQuatParamTopicName, "true");
        usleep(100000);
        attPub = nodeHandle.advertise<geometry_msgs::PoseStamped>(setattTopicName, QUENUE_DEPTH);
    }
    else{
        nodeHandle.setParam(useQuatParamTopicName, "false");
        usleep(100000);
        velPub = nodeHandle.advertise<geometry_msgs::TwistStamped>(cmdVelTopicName, QUENUE_DEPTH);
    }
    thrPub = nodeHandle.advertise<mavros_msgs::Thrust>(setthrTopicName, QUENUE_DEPTH);
    photonTmPub = nodeHandle.advertise<std_msgs::Float32MultiArray>(photonTmTopicName, QUENUE_DEPTH);
}

bool AttControl::sendIdling()
{
    if (USE_QUATERNION){
        pubCtrl(MIN_THRUST, qPx * qPxInit);
    }
    else{
        pubCtrl(MIN_THRUST, Eigen::Vector3f(0.f, 0.f, 0.f));
    }
    return true;
}

bool AttControl::sendPhotonTm()
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(rPx[0]);
    msg.data.push_back(rPx[1]);
    msg.data.push_back(rPx[2]);
    photonTmPub.publish(msg);
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

void AttControl::pubCtrl(float thr, const Eigen::Vector3f& v)
{
    ros::Time now = ros::Time::now();
    geometry_msgs::TwistStamped twistMsg;
    twistMsg.header.stamp = now;
    twistMsg.header.frame_id = FRAME_ID;
    twistMsg.twist.angular.x = v[0];
    twistMsg.twist.angular.y = v[1];
    twistMsg.twist.angular.z = v[2];
    twistMsg.twist.linear.x = 0.;
    twistMsg.twist.linear.y = 0.;
    twistMsg.twist.linear.z = 0.;

    mavros_msgs::Thrust thrMsg;
    thrMsg.header.stamp =  now;
    thrMsg.header.frame_id = FRAME_ID;
    thrMsg.thrust = thr;

    thrPub.publish(thrMsg);
    velPub.publish(twistMsg);
}

void AttControl::imuCb(const sensor_msgs::Imu& msg)
{
    aPx = Eigen::Vector3f(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    Eigen::Quaternion<float> qPxInput(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    oPx = Eigen::Vector3f(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);

    if (!imuReady){
        qPxInit = qPxInput;
        gIestimated =  Eigen::Vector3f(aPx);
        gIestimated = quatRotate(qPxInit.inverse(), gIestimated);
    }
    qPx = qPxInput * qPxInit.inverse();
    aPxClearI = quatRotate(qPx.inverse(), aPx) - gIestimated;
    imuReady = true;
}

void AttControl::velCb(const geometry_msgs::TwistStamped& msg)
{
    if (imuReady){
        //vPx = quatRotate(qPx.inverse(),  Eigen::Vector3f(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z));
        vPx = Eigen::Vector3f(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
        velReady = true;
    }
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

    odometryReady = true;
}

void AttControl::lidCb(const sensor_msgs::LaserScan& msg)
{
    float r = msg.range_min;
    uint64_t time = timeMs;
    float rdot = -lidSerDiff.get(r, time);
    if (lidSerDiff.ready()){
        rLid = r;
        vLid = rdot;
        lidReady = true;
    }
}

void AttControl::altCb(const sensor_msgs::LaserScan& msg)
{

    float r = msg.range_min;
    uint64_t time = timeMs;
    float rdot = altSerDiff.get(r, time);
    if (altSerDiff.ready()){
        rAlt = r;
        vAlt = rdot;
        altReady = true;
    }

}

bool AttControl::estimateState()
{

    float dt = getLastTickDuration();

    if (isNanVect(aPxClearI)) {
        return false;
    }

    if (USE_ODOMETRY && !isNanVect(rOd) && !isNanVect(vOd)){

        bool goodOdometry = !isZeroVector3f(rOd) && !isZeroVector3f(vOd);

        if (goodOdometry){
            Eigen::Vector2f xEstState = ekfX.get(rOd[0], vOd[0], 0.f, dt);
            Eigen::Vector2f yEstState = ekfY.get(rOd[1], vOd[1], 0.f, dt);
            Eigen::Vector2f zEstState = ekfZ.get(rOd[2], vOd[2], 0.f, dt);
            rEs = Eigen::Vector3f(xEstState[0], yEstState[0], zEstState[0]);
            vEs = Eigen::Vector3f(xEstState[1], yEstState[1], zEstState[1]);
        }
        else {
            Eigen::Vector2f xEstState = ekfX.get(0.f, dt);
            Eigen::Vector2f yEstState = ekfY.get(0.f, dt);
            Eigen::Vector2f zEstState = ekfZ.get(0.f, dt);
            rEs = Eigen::Vector3f(xEstState[0], yEstState[0], zEstState[0]);
            vEs = Eigen::Vector3f(xEstState[1], yEstState[1], zEstState[1]);
        }
    }

    if (USE_ALTIMETR){
        Eigen::Vector2f altEstState = ekfAlt.get(rAlt, vAlt, aPxClearI[2], dt);
        rAltEs = altEstState[0];
        vAltEs = altEstState[1];
    }

    if (USE_LIDAR && !std::isnan(rLid)){
        Eigen::Vector2f lidEstState = ekfLid.get(rLid, vLid, quatRotate(qPx, aPxClearI)[0], dt); // TODO add acc x in body frame
        rLidEs = lidEstState[0];
        vLidEs = lidEstState[1];
    }


}

void AttControl::photonCmdCb(const std_msgs::Float32MultiArray& msg)
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

    Eigen::Vector3f r = Eigen::Vector3f(msg.data[0], msg.data[1], msg.data[2]);
    rInput = r;
    yawRateInput = msg.data[3];
    aimAccepted = true;
    //logger.addEvent("AttCtrl: new aim accepted", timeMs);

}

void AttControl::writeLogData()
{
    Eigen::Vector3f nullV(0.f, 0.f, 0.f);
    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()
    );

    logData.timeMs = ms.count();
    logData.qPx = qPx;
    logData.aPx = aPxClearI;
    logData.r0 = accumulatePosIrEstimator.get();
    logData.v0 = accumulateVelIrEstimator.get();

    if (USE_GPS){
        logData.vPx = vPx;
        logData.rPx = rPx;
    }
    else{
        logData.vPx = nullV;
        logData.rPx = nullV;
    }

    if (USE_ODOMETRY){
        logData.qOd = qOd;
        logData.rOd = rOd;
        logData.vOd = vOd;
        logData.rEs = rEs;
        logData.vEs = vEs;
    }else {
        logData.qOd = Eigen::Quaternion<float>(1, 0, 0, 0);
        logData.rOd = nullV;
        logData.vOd = nullV;
        logData.rEs = nullV;
        logData.vEs = nullV;
    }

    if (USE_ALTIMETR){
        logData.rAlt = rAlt;
        logData.vAlt = vAlt;
        logData.rAltEs = rAltEs;
        logData.vAltEs = vAltEs;
    }
    else{
        logData.rAlt = 0.f;
        logData.vAlt = 0.f;
        logData.rAltEs = 0.f;
        logData.vAltEs = 0.f;
    }

    if (USE_LIDAR){
        logData.rLid = rLid;
        logData.vLid = vLid;
        logData.rLidEs = rLidEs;
        logData.vLidEs = vLidEs;
    }
    else{
        logData.rLid = 0.f;
        logData.vLid = 0.f;
        logData.rLidEs = 0.f;
        logData.vLidEs = 0.f;
    }

    logger.addData(logData);
}

