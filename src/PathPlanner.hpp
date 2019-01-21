#pragma once
#include "Eigen/Dense"
#include <iostream>
#include <stdio.h>
#include <vector>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "Params.h"

using std::string;
using Eigen::Matrix;
using Eigen::Map;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::Dynamic;
using ros::Publisher;
using ros::Subscriber;
using ros::NodeHandle;
using ros::Rate;
using nav_msgs::OccupancyGrid;

class PathPlanner {
public:
    PathPlanner():
    mapTopName("/rtabmap/proj_map")
    ,rate(10)
    ,mapInited(false)
    {}

    void init(){

        mapSub = nodeHandle.subscribe(mapTopName, QUENUE_DEPTH, &PathPlanner::mapCb, this);
        file = ::fopen("/home/acsl/1simflightlogs/map_data.csv", "w");

        while(ros::ok){
            getNextAim();
            rate.sleep();
            ros::spinOnce();
        }
    }

    void getNextAim(){
        if (mapInited){
            Vector2i indexes = poseToMapIndexes(Vector2f(0.f, 0.f));
            //std::cout << indexes << std::endl;
            bool find = false;
            int k = 1;
            while(!find) {
                int value = map(indexes[0], indexes[1]+k);
                std::cout << value << " ";
            }
            std::cout << std::endl;
        }
    }

    uint8_t getElem(const OccupancyGrid& map, int x, int y){
        int height = map.info.height;
        int width = map.info.width;
        return map.data[height * y + x];
    }

    void mapCb(const OccupancyGrid& map) {
        int height = map.info.height;
        int width = map.info.width;
        for (int y = 0; y < height; ++y){
            for (int x = 0; x < width; ++x){
                ::fprintf(file, "%d,", map.data[height * y + x]);
            }
            ::fprintf(file, "\n");
        }
        Vector2i origin = poseToMapIndexes(Vector2f(0.f, 0.f));
        std::cout << origin << std::endl;
        ::fflush(file);
    }

    Vector2i poseToMapIndexes(const Vector2f& r){
        Vector2f rn = r - poseOrigin;
        int h = rn[1] / resolution;
        int w = rn[0] / resolution;
        if(w < 0 || h < 0){
            printf("out of map");
        }
        return Vector2i(w, h);
    }

private:
    string  mapTopName;

    NodeHandle nodeHandle;
    Subscriber  mapSub;
    Rate rate;

    Matrix<uint8_t, Dynamic, Dynamic> map;
    Vector2f poseOrigin;
    float resolution;
    int height;
    int width;
    bool mapInited;


     FILE* file;

};
