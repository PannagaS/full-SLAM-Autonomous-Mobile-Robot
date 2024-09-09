#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <utils/geometric/point.hpp>
#include <queue>

SensorModel::SensorModel(void)
:   sigma_hit_(0.075),
	occupancy_threshold_(10),
	ray_stride_(7),
	max_ray_range_(1000),
    search_range(2),
    offset_quality_weight(3)
{
    initialize_bfs_offsets();
}

void SensorModel::initialize_bfs_offsets()
{
    /// TODO: Initialize the BFS offsets based on the search range 
    
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    /// TODO: Compute the likelihood of the given particle using the provided laser scan and map. 
    MovingLaserScan moving_scan(scan, sample.parent_pose, sample.pose);
    double scanScore = 0.0;

    for (auto& ray : moving_scan)
    {
        Point<double> endPoint(ray.origin.x + ray.range * std::cos(ray.theta),
                               ray.origin.y + ray.range * std::sin(ray.theta));
        auto rayEnd = global_position_to_grid_position(endPoint, map);
    //    double score = scoreRay(ray, map)+16;
        double score = map.logOdds(rayEnd.x, rayEnd.y);
        if(score > 0) scanScore += score;
    }

    return scanScore;
    
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Compute a score for a given ray based on its end point and the map. 
    // Consider the offset from the nearest occupied cell.  

    // Point<double>endpoint(ray.origin.x+ray.range * std::cos(ray.theta), ray.origin.y+ray.range * std::sin(ray.theta));
    // Point<int> end_cell = global_position_to_grid_position(end_point, map);
    Point<int> end_cell = getRayEndPointOnMap(ray, map);
    Point<int> ray_start = global_position_to_grid_cell(ray.origin, map);

    //return map(end_cell.x,end_cell.y)>0;    //this is the very simple model that works good
    
    if(!map.isCellInGrid(end_cell.x, end_cell.y) )//|| ray.range > kMaxLaserDistance_ || ray.range < kMinLaserDistance_)
        return -16;     //return a bad score if the endpoint goes beyond the map

    const int dx = abs(end_cell.x - ray_start.x);
    const int dy = abs(end_cell.y - ray_start.y);
    const int slope_x = ray_start.x < end_cell.x ? 1 : -1;
    const int slope_y = ray_start.y < end_cell.y ? 1 : -1;
    int err = dx - dy;
    int err2 = 2*err;
    int x = ray_start.x;
    int y = ray_start.y;

    int distance_to_nearest_occupied = std::numeric_limits<int>::max(); // Initialize distance to maximum

    //bool seen_wall = false;
    Point<int> first_wall;
    MAX_RATIO_FOR_WALL = 1;
    while( !(x == end_cell.x && y == end_cell.y)){
        if(map(x,y)>=std::numeric_limits<CellOdds>::max()*MAX_RATIO_FOR_WALL){  
            // Use gridBFS to find the nearest occupied cell to the end_cell
            Point<int> nearest_occupied_cell = gridBFS(Point<int>(x, y), map);
            if (nearest_occupied_cell != Point<int>(0, 0)) {
                // Calculate the distance from the current cell (x, y) to the nearest occupied cell
                int distance = abs(nearest_occupied_cell.x - x) + abs(nearest_occupied_cell.y - y);
                // Update distance_to_nearest_occupied if the calculated distance is smaller
                distance_to_nearest_occupied = std::min(distance_to_nearest_occupied, distance);
            }
            // Long hit... Check one after
            err2 = 2*err;
            if(err2 >= -dy){
                err -= dy;
                x += slope_x;
            }
            if(err2 <= dx){
                err += dx;
                y += slope_y;
            }
            if(map(x,y)>=std::numeric_limits<CellOdds>::max()*MAX_RATIO_FOR_WALL){
                // Not so bad
                return -8;
            }else{
                // Worse
                return -12;
            }
        }
        err2 = 2*err;
        if(err2 >= -dy){
            err -= dy;
            x += slope_x;
        }
        if(err2 <= dx){
            err += dx;
            y += slope_y;
        }
    }
    // At Ray end
    if(map(end_cell.x,end_cell.y)>=std::numeric_limits<CellOdds>::max()*MAX_RATIO_FOR_WALL){
        // Best Case
        return -4;
    }else{
        return -10;
    }
    
}

double SensorModel::NormalPdf(const double& x)
{
    return (1.0/(sqrt(2.0 * M_PI)*sigma_hit_))*exp((-0.5*x*x)/(sigma_hit_*sigma_hit_));
}

Point<int> SensorModel::gridBFS(const Point<int> end_point, const OccupancyGrid& map)
{
    /// TODO: Use Breadth First Search to find the nearest occupied cell to the given end point. 
    const std::vector<Point<int>> directions = {Point<int>(1, 0), Point<int>(-1, 0), Point<int>(0, 1), Point<int>(0, -1)};

    // Create a queue for BFS
    std::queue<Point<int>> bfsQueue;

    // Mark visited cells
    std::vector<std::vector<bool>> visited(map.widthInCells(), std::vector<bool>(map.heightInCells(), false));

    // Start BFS from the end_point
    bfsQueue.push(end_point);
    visited[end_point.x][end_point.y] = true;

    while (!bfsQueue.empty()) {
        Point<int> current = bfsQueue.front();
        bfsQueue.pop();

        // Check if the current cell is occupied
        if (map(current.x, current.y) > 0) {
            return current; // Found an occupied cell, return its coordinates
        }

        // Explore neighboring cells
        for (const auto& dir : directions) {
            Point<int> next = current + dir;

            // Check if the next cell is within the map boundaries and not visited
            if (map.isCellInGrid(next.x, next.y) && !visited[next.x][next.y]) {
                bfsQueue.push(next);
                visited[next.x][next.y] = true;
            }
        }
    }

    // If no occupied cell is found, return a default Point(0, 0)
    return Point<int>(0, 0);
    
}

Point<float> SensorModel::getRayEndPointOnMap(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Calculate the end point of a given ray on the map 
    Point<float> end_point(ray.origin.x + ray.range * std::cos(ray.theta), ray.origin.y + ray.range * std::sin(ray.theta));
    Point<int> end_cell = global_position_to_grid_position(end_point, map);

    return Point<float>(end_cell.x, end_cell.y);
    
}
