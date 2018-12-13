#pragma once
#include <limits>
#include <Eigen/Dense>

/* constants */
const std::string OFFBOARD("OFFBOARD");                                     // ros offboard mode
const std::string FRAME_ID("map");                                          // ros rutines
const int QUENUE_DEPTH = 10;                                                // ros nodes quenue depth
const float FREE_FALL_ACC_ABS = 10.f;                                        // free fall acc approx
const float PI = 3.1415f;                                        // math Pi
const Eigen::Vector3f UNIT_Z(0.f, 0., 1.f);                                   // z unit vector

const int CTRL_RATE = 100;                                                   //control rate in HZ
const float MIN_TICK_FOR_CALCS = 1.f/CTRL_RATE;                                //min dt for integrator, differentiator and other math

/* control params */
const float MIN_THRUST = 0.2f;                                               // minimum in flight thrust (% / 100); value used for null thrsut at the ground
const float MAX_THRUST = 0.8f;                                               // maxim in flight thrust (% / 100)
const float TW = 1.7f;                                                       // aprox thrust / weight ratio

// goToLocal params
const float GO_LOCAL_INPUT_LIM_V = 1.f;                                  // cut dr on GO_LOCAL_INPUT_ERROR_LIM
const float GO_LOCAL_PID_P_V = 1.f;                                            // position error -> thrust PID proportional
const float GO_LOCAL_PID_I_V = 0.2f;                                           // position error -> thrust PID integral
const float GO_LOCAL_PID_I_LIM_V = 6.f;                                           // position error -> thrust PID integral
const float GO_LOCAL_PID_D_V = 1.4f;                                            // position error -> thrust PID integral
const float GO_LOCAL_ANTIWINDUP_PARAM_V = 0.5f;                                // integrator part raises only when z*vz < GO_LOCAL_ANTIWINDUP_COEFF

const float GO_LOCAL_INPUT_LIM_H = 1.f;                                  // cut dr on GO_LOCAL_INPUT_ERROR_LIM vertical
const float GO_LOCAL_PID_P_H = 0.4f;                                            // position error -> thrust PID proportional
const float GO_LOCAL_PID_D_H = 1.4f;                                            // position error -> thrust PID differential

const float POSE_EPS = 0.3f;

// accumulateVelocityWithImu params
const float ACCUM_VEL_PID_P = 0.33f;                                           // imu estimated velocity -> thrust proportional coeff
const float ACCUM_VEL_PID_P_ERROR_LIM = 0.5f;                                // imu estimated velocity -> thrust proportional out limit
const float ACCUM_VEL_PID_I = 1.f;                                           // imu estimated velocity -> thrust integral coeff
const float ACCUM_VEL_PID_I_ERROR_LIM = 0.2f;                                // imu estimated velocity -> thrust integral out limit
const float EST_VEL_EPS = 0.1f;                                             // epsilon to float comparison vel and desired vel
