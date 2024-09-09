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

    // Set maze points
    pose2D_t mazePoints[] = {
        {0, 0.61f,  0.00f, 0.0f},
        {0, 0.61f, -0.61f, 0.0f},
        {0, 1.22f, -0.61f, 0.0f},
        {0, 1.22f,  0.61f, 0.0f},
        {0, 1.83f,  0.61f, 0.0f},
        {0, 1.83f, -0.61f, 0.0f},
        {0, 2.44f, -0.61f, 0.0f},
        {0, 2.44f,  0.00f, 0.0f},
        {0, 3.05f,  0.00f, 0.0f}
    };

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