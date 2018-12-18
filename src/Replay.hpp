#include "Logger.hpp";
#include "SimpleEstimator.hpp"
#include <Eigen/Dense>
#include "AttControl.h"

class Replay{
public:
    Replay() : logger("/home/acsl/1simflightlogs/ctrl_data_replay.csv")
    {
    }
    void ekfRecalk(){
        readResult rr = readResult::ok;
        while (rr != readResult::end){
            rr = player.readData(&data);

            if (data.debug3 < Status::armed){
                continue;
            }
            bool measured = data.debug2;
            float dt = data.debug1;
            bool goodOdometry = !isZeroVector3f(data.rOd) && !isZeroVector3f(data.vOd);

//             if (measured && goodOdometry){
                Eigen::Vector2f xEstState = ekfX.get(data.rOd[0], data.vOd[0], data.aPx[0], dt);
                Eigen::Vector2f yEstState = ekfY.get(data.rOd[1], data.vOd[1], data.aPx[1], dt);
                Eigen::Vector2f zEstState = ekfZ.get(data.rOd[2], data.vOd[2], data.aPx[2], dt);
                Eigen::Vector3f rEs = Eigen::Vector3f(xEstState[0], yEstState[0], zEstState[0]);
                Eigen::Vector3f vEs = Eigen::Vector3f(xEstState[1], yEstState[1], zEstState[1]);
                data.rEs = rEs;
                data.vEs = vEs;
//             }
//             else{
//                 Eigen::Vector2f xEstState = ekfX.get(data.aPx[0], dt);
//                 Eigen::Vector2f yEstState = ekfY.get(data.aPx[1], dt);
//                 Eigen::Vector2f zEstState = ekfZ.get(data.aPx[2], dt);
//                 Eigen::Vector3f rEs = Eigen::Vector3f(xEstState[0], yEstState[0], zEstState[0]);
//                 Eigen::Vector3f vEs = Eigen::Vector3f(xEstState[1], yEstState[1], zEstState[1]);
//                 data.rEs = rEs;
//                 data.vEs = vEs;
//             }

            logger.addData(data);
        }
    };

private:
    Player player;
    FlightData data;
    Ekf ekfX;
    Ekf ekfY;
    Ekf ekfZ;

    Logger logger;
};
