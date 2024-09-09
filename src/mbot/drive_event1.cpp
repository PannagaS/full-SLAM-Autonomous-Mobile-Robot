#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

using namespace mbot_lcm_msgs;

int main(int argc, char** argv)
{
    std::cout << "Commanding robot to drive in a maze.\n";

    path2D_t path;

    // Set starting point
    pose2D_t startPoint;
    startPoint.x = 0.0f;
    startPoint.y = 0.0f;
    startPoint.theta = 0.0f;

    pose2D_t mazePoints[] = {
            {0, 0.00,  0.00, 0},
        {0, 0.61, 0.00, 0},
        {0, 0.61, 0.61, 0},
        {0, 1.22, 0.61, 0},
        {0, 1.22, 0.00, 0},
        {0, 1.83, 0.00, 0},
        {0, 1.83, 1.22, 0},
        {0, 0.00, 1.22, 0},
        {0, 0.00, 0.00, 0}
        };
    /*
        pose2D_t mazePoints[] = {
            {0, 0.00, 0.00, 0},
    {0, 0.30, 0.00,0},
    {0,0.30 ,0.00,0},
    {0,0.37 ,0.01,0},
    {0,0.44 ,0.03,0},
    {0,0.49 ,0.07,0},
    {0,0.54 ,0.11,0},
    {0,0.58 ,0.17,0},
    {0,0.60 ,0.24,0},
    {0,0.61 ,0.30,0},
    {0,0.61 ,0.30,0},
    {0,0.62 ,0.37,0},
    {0,0.64 ,0.44,0},
    {0,0.68 ,0.49,0},
    {0,0.72 ,0.54,0},
    {0,0.78 ,0.58,0},
    {0,0.85 ,0.60,0},
    {0,0.91 ,0.61,0},
    {0,1.05 ,0.58,0},
    {0,1.10 ,0.54,0},
    {0,1.15 ,0.49,0},
    {0,1.19 ,0.44,0},
    {0,1.21 ,0.37,0},
    {0,1.22 ,0.30,0},
    {0,1.22 ,0.30,0},
    {0,1.23 ,0.24,0},
    {0,1.25 ,0.17,0},
    {0,1.28 ,0.11,0},
    {0,1.33 ,0.07,0},
    {0,1.39 ,0.03,0},
    {0,1.46 ,0.01,0},
    {0,1.52 ,0.00,0},
    {0,1.83 ,0.00,0},
    {0,1.83 ,0.91,0},
    {0,1.83 ,0.91,0},
    {0,1.82 ,0.98,0},
    {0,1.80 ,1.05,0},
    {0,1.76 ,1.10,0},
    {0,1.71 ,1.15,0},
    {0,1.66 ,1.19,0},
    {0,1.59 ,1.21,0},
    {0,1.52 ,1.22,0},
    {0,0.30 ,1.22,0},
    {0,0.30, 1.22,0},
    {0,0.24  ,1.21,0},
    {0,0.17 ,1.19,0},
    {0,0.11 ,1.15,0},
    {0,0.07 ,1.10,0},
    {0,0.03 ,1.05,0},
    {0,0.01 ,0.98,0},
    {0,0.00 ,0.91,0},
    {0,0.00 ,0.00,0}
        };
    */  

    // Construct the path
    path.path.push_back(startPoint); // Add the starting point
    path.path.insert(path.path.end(), std::begin(mazePoints), std::end(mazePoints)); // Add maze points

    path.path_length = path.path.size();

    lcm::LCM lcmInstance(MULTICAST_URL);
    std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}