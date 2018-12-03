#include <limits>
#include <Eigen/Dense>

/* constants */
const std::string OFFBOARD("OFFBOARD");                                     // ros offboard mode
const std::string FRAME_ID("map");                                          // ros rutines
const int QUENUE_DEPTH = 10;                                                // ros nodes quenue depth
const float FREE_FALL_ACC_ABS = 10.;                                        // free fall acc approx
const Eigen::Vector3f UNIT_Z(0., 0., 1.);                                   // z unit vector

const int CTRL_RATE = 100;                                                   //control rate in HZ
const int MIN_TICK_FOR_CALCS = 1./CTRL_RATE;                                //min dt for integrator, differentiator and other math

/* control params */
const float MIN_THRUST = 0.2;                                               // minimum in flight thrust (% / 100); value used for null thrsut at the ground
const float MAX_THRUST = 0.8;                                               // maxim in flight thrust (% / 100)
const float TW = 1.7;                                                       // aprox thrust / weight ratio
const float THRUST_PID_P = 2e0;                                            // position error -> thrust PID proportional
const float THRUST_PID_I = 1e-1;                                            // position error -> thrust PID integral
const float THRUST_PID_D = 2e0;                                            // position error -> thrust PID differential
const float THRUST_PID_P_ERROR_LIM = 10.;                                   // position error -> thrust PID proportional
const float THRUST_PID_I_ERROR_LIM = 1.;                                    // position error -> thrust PID proportional
const float THRUST_PID_D_ERROR_LIM = std::numeric_limits<float>::max();     // position error -> thrust PID proportional
