#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose2D_t& pose,
                        OccupancyGrid& map)
{
    if (!initialized_)
        previousPose_ = pose;
    initialized_ = true;

    MovingLaserScan movingScan(scan, previousPose_, pose);

    /// TODO: Update the map's log odds using the movingScan  
    //
    // Hint: Consider both the cells the laser hit and the cells it passed through.
    for(auto& ray : movingScan)
    {
        scoreEndpoint(ray, map);
    }

    for(auto& ray : movingScan){
        scoreRay(ray, map);
    }
    // std::cout << "map updated" << std::endl;
    previousPose_ = pose;
    return;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits  
    if (ray.range < kMaxLaserDistance_)
    {
        Point<int> cellStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> cellEnd = global_position_to_grid_cell(
            Point<float>(
                ray.origin.x + ray.range * cos(ray.theta),
                ray.origin.y + ray.range * sin(ray.theta)
                ),
            map
            );

        if (map.isCellInGrid(cellEnd.x, cellEnd.y))
        {
            increaseCellOdds(cellEnd.x, cellEnd.y, map);
        }

    }
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map)
{
    if (initialized_){
        auto odd = map.logOdds(x,y);
        if (127 - odd > kHitOdds_)
        {   
            odd += kHitOdds_;
        } else {
            odd = 127;
        }
        map.setLogOdds(x, y, odd);
    }
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map)
{
    if (initialized_){
        auto odd = map.logOdds(x,y);
        if (odd + 127 > kMissOdds_)
        {
            odd -= kMissOdds_;
        } else {
            odd = -127;
        }
        map.setLogOdds(x, y, odd);
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through  
    std::vector<Point<int>> cells = bresenham(ray, map);
    for (auto& cell : cells)
    {
        decreaseCellOdds(cell.x, cell.y, map);
    }
    
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
    Point<int> cellStart = global_position_to_grid_cell(ray.origin, map);

    Point<float> rayEnd = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * cos(ray.theta),
            ray.origin.y + ray.range * sin(ray.theta)
            ),
        map
        );
    Point<int> cellEnd;
    cellEnd.x = static_cast<int>(rayEnd.x);
    cellEnd.y = static_cast<int>(rayEnd.y);
    // ####### Start of bresenham algorithm #######
    const int x0 = cellStart.x;
    const int y0 = cellStart.y;
    const int x1 = cellEnd.x;
    const int y1 = cellEnd.y;
    int dx = x1-x0;
    int dy = y1-y0;
    int sx = (dx > 0) - (dx < 0);
    int sy = (dy > 0) - (dy < 0);
    dx = abs(dx);
    dy = abs(dy);
    int err = dx - dy;
    int x = x0;
    int y = y0;
    // std::cout << "(" << x0 << ", " << y0 << ")->(" << x1 << "," << y1 << ")" << std::endl;
    std::vector<Point<int>> cells;
    cells.push_back(cellStart);
    while (x != x1 || y != y1)
    {
        // std::cout << "(" << x << ", " << y << ")" << std::endl;
        // std::cout << "(" << x0 << ", " << y0 << ")->(" << x1 << "," << y1 << ")" << std::endl;
        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
        Point<int> cell(x,y);
        cells.push_back(cell);
    }
    return cells;
    
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray. 
    std::vector<Point<int>> cells;
    Point<float> rayStart = global_position_to_grid_cell(ray.origin, map);
    Point<int> rayCell;
    for (int i = 0; i < 2 * ray.range * map.cellsPerMeter(); i++)
    {
        rayCell.x = static_cast<int>(rayStart.x + (i * cos(ray.theta) * map.cellsPerMeter()));
        rayCell.y = static_cast<int>(rayStart.y + (i * sin(ray.theta) * map.cellsPerMeter()));
        cells.push_back(rayCell);
    }

    return cells;

}
