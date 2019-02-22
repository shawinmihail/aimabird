#pragma once
#include <limits>
#include <Eigen/Dense>

/* constants */
const std::string OFFBOARD("OFFBOARD");                                     // ros offboard mode
const std::string FRAME_ID("map");                                          // ros rutines
const int QUENUE_DEPTH = 10;                                                // ros nodes quenue depth
const float FREE_FALL_ACC_ABS = 10.f;                                        // free fall acc approx
const float PI = 3.1415f;                                                     // math Pi
const Eigen::Vector3f UNIT_Z(0.f, 0., 1.f);                                   // z unit vector
const Eigen::Vector3f UNIT_X(1.f, 0., 0.f);                                   // z unit vector

const int CTRL_RATE = 100;                                                   //control rate in HZ

/* control params */
const bool USE_QUATERNION = true;                                             // if false use angular velocity control, else -- quaternion
const bool USE_GPS = true;                                             // if false use px4 pos vel feedback, else -- odometry estimated
const bool USE_ODOMETRY = false;                                             // if false use px4 pos vel feedback, else -- odometry estimated
const bool USE_ALTIMETR = false;
const bool USE_LIDAR = false;
const float MIN_THRUST = 0.2f;                                               // minimum in flight thrust (% / 100); value used for null thrsut at the ground
const float MAX_THRUST = 0.8f;                                               // maxim in flight thrust (% / 100)
const float TW = 1.7f;
const float MAX_BOW = 7.f*PI/ 180.f;

// goToLocal params
const float GO_LOCAL_PID_P_V = 1.f;                                             // position error -> thrust PID proportional
const float GO_LOCAL_PID_I_V = 0.2f;                                            // position error -> thrust PID integral
const float GO_LOCAL_PID_D_V = 3.0f;                                            // position error -> thrust PID integral
const float GO_LOCAL_INPUT_LIM_V = 1.5f;                                        // cut dr on GO_LOCAL_INPUT_ERROR_LIM
const float GO_LOCAL_PID_I_LIM_V = 6.f;                                         // position error -> thrust PID integral
const float GO_LOCAL_ANTIWINDUP_PARAM_V = 0.25f;                                // integrator part raises only when z*vz < GO_LOCAL_ANTIWINDUP_COEFF

const float GO_LOCAL_PID_P_H = 1.0f;                                            // position error -> thrust PID proportional
const float GO_LOCAL_PID_D_H = 3.0f;                                            // position error -> thrust PID differential
const float GO_LOCAL_INPUT_LIM_H = 1.5f;                                        // cut dr on GO_LOCAL_INPUT_ERROR_LIM vertical

const float GO_LOCAL_PID_DI_H = 1.0f;
const float GO_LOCAL_PID_DI_V = 0.5f;
const float GO_LOCAL_PID_DI_LIM = 0.35f;

const float POSE_EPS = 0.3f;
const float YAW_RATE_DES = 0.2f;

// accumulateVelocityWithImu params
const float ACCUM_VEL_PID_P = 0.05f;                                          // imu estimated velocity -> thrust proportional coeff
const float ACCUM_VEL_PID_P_ERROR_LIM = 0.5f;                                // imu estimated velocity -> thrust proportional out limit
const float ACCUM_VEL_PID_I = 0.2f;                                           // imu estimated velocity -> thrust integral coeff
const float ACCUM_VEL_PID_I_ERROR_LIM = MAX_THRUST;                                // imu estimated velocity -> thrust integral out limit
const float EST_VEL_EPS = 0.15f;                                             // epsilon to float comparison vel and desired vel

/* EKF */

/* path planner */
const float CELL_SIZE = 0.05f;

/*emergency*/
const float EMERG_LAND_VEL = -0.25f;
const float EMERG_ALT_VALUE = 0.7f;
const float EMERG_LID_VALUE = 1.0f;
const float EMERG_ACC_LIM = 5.0f;


