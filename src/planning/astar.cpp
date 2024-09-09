#include<planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

Node* current_goal = nullptr;
PriorityQueue open_list;  
PriorityQueue closed_list; 


mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                             mbot_lcm_msgs::pose2D_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    printf("\nStarting search for path\n");
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    bool found_path = false;
    // distances.enqueue_obstacle_cells()
    ////////////////// TODO: Implement your A* search here //////////////////////////
 

    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);
    current_goal = goalNode;
    
    if (startNode != goalNode)
    {
        printf("Start = %d, %d, Goal %d, %d\n", startNode->cell.x, startNode->cell.y, goalNode->cell.x, goalNode->cell.y);
        bool in_loop = true;
        open_list = PriorityQueue();
        closed_list = PriorityQueue();   
        startNode->g_cost = 0.0;
        open_list.push(startNode); 
        int i = 0;
        while (!open_list.empty()){ //Was skipping this loop
            Node* n = open_list.pop();

            std::cout.precision(4);
            if (i % 1000 == 0) std::cout << "[Astar] searching for "<< i << "nodes. x=" << n->cell.x << " y=" << n->cell.y << " f=" << n->f_cost() << " g=" << n->g_cost << " h=" << n->h_cost << std::endl;
            
            if (i < 100)
            {
                //printf("expanding %d, %d with g_cost = %f, h_cost = %f\n", n->cell.x, n->cell.y, n->g_cost, n->h_cost);
            }
            i++;
            std::vector<Node*> children = expand_node(n, distances, params);
            for (int i=0;i<children.size();i++)
            {
                Node* child = children.at(i);
                if (child->cell.x == goalNode->cell.x && child->cell.y == goalNode->cell.y)
                {
                    // Leave loop
                    goalNode->parent = child->parent;
                    in_loop = false;
                    found_path = true;
                    break;
                }
                if (!closed_list.is_member(child))
                {
                    open_list.push(child);
                }
            }
            if (!in_loop)
            {
                break;
            }
            closed_list.push(n);
        }
    }

    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;
    if (found_path)
    {
        //printf("Path found\n");
        auto nodePath = extract_node_path(goalNode, startNode);
        //printf("Node path created\n");
        path.path = extract_pose_path(nodePath, distances, start, goal);
        //printf("Pose path created\n");
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
    }

    else printf("[A*] Didn't find a path\n");
    path.path_length = path.path.size();
    return path;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    ////////////////// TODO: Implement your heuristic //////////////////////////
    double dx = abs(from->cell.x - goal->cell.x);
    double dy = abs(from->cell.y - goal->cell.y);
    double greedy = 100;
    return greedy* ((dx + dy) + (1.414-2) * std::min(dx,dy));
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double g_cost = from->parent->g_cost + 1;
    if (from->parent->cell.x != from->cell.x && from->parent->cell.y != from->cell.y)
    {
        g_cost += 0.414;
    }

    float dist = distances(from->cell.x, from->cell.y);
    
    
    // if (dist < params.minDistanceToObstacle)
    // {
    //     g_cost = 99999; // We shouldn't try this cell
    // }
    /*
    else if (dist < params.maxDistanceWithCost){
        g_cost += pow(params.maxDistanceWithCost - dist, params.distanceCostExponent);
    }
    */
    g_cost += 10 / dist;
    ////////////////// TODO: Implement your goal cost, use obstacle distances //////////////////////////
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    auto is_on_grid =[](const cell_t& cell, const ObstacleDistanceGrid& grid)
    {
        if ((cell.x >= 0) && (cell.x < grid.widthInCells()) && (cell.y >= 0) && (cell.y < grid.heightInCells()))
        {
            return grid(cell.x,cell.y) > 0;
        }
        else
        {
            return false;
        }
    };

    std::vector<Node*> children;
    ////////////////// TODO: Implement your expand node algorithm /////////////////////////
    
   
    
    // TODO: Expand to 8 way? yeees!
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};

    for(int i = 0; i < 8; i++){
        int x = node->cell.x + xDeltas[i];
        int y = node->cell.y + yDeltas[i];
        Node* current_neighbor = new Node(x,y);
        if (is_on_grid(cell_t(x,y), distances))
        {
            //printf("we are on the grid\n");
            current_neighbor->parent = node;
            current_neighbor->g_cost = g_cost(current_neighbor, current_goal, distances, params);
            if (current_neighbor->g_cost < 0)
            {
                continue;
            }
            current_neighbor->h_cost = h_cost(current_neighbor, current_goal, distances);
            if (open_list.is_member(current_neighbor)){
                Node* previous_best = open_list.get_member(current_neighbor);
                if (previous_best->f_cost() > current_neighbor->f_cost()){
                    children.push_back(current_neighbor);
                }
            }else{
                children.push_back(current_neighbor);
            }
        }
        
    }
    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    std::vector<Node*> path;
    ////////////////// TODO: Implement your extract node function //////////////////////////
    // Traverse nodes and add parent nodes to the vector
    while (goal_node->cell.x != start_node->cell.x || goal_node->cell.y != start_node->cell.y)
    {
        path.push_back(goal_node);
        goal_node = goal_node->parent;
    }
    
    // Reverse path
    std::reverse(path.begin(), path.end());
    return path;
}
// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances, mbot_lcm_msgs::pose2D_t start, mbot_lcm_msgs::pose2D_t goal)
{
    std::vector<mbot_lcm_msgs::pose2D_t> path;
    ////////////////// TODO: Implement your extract_pose_path function //////////////////////////
    // This should turn the node path into a vector of poses (with heading) in the global frame
    // You should prune the path to get a waypoint path suitable for sending to motion controller
    auto getXGlobal =[](int x, const ObstacleDistanceGrid& distances)
    {
        return distances.metersPerCell() * x + distances.originInGlobalFrame().x;
    };
    auto getYGlobal =[](int y, const ObstacleDistanceGrid& distances)
    {
        return distances.metersPerCell() * y + distances.originInGlobalFrame().y;
    };
    
    path.push_back(start);
    for (int i=1;i<nodes.size();i++){
        double x_global = getXGlobal(nodes.at(i)->cell.x, distances);
        double y_global = getYGlobal(nodes.at(i)->cell.y, distances);
        double x_global_before = getXGlobal(nodes.at(i-1)->cell.x, distances);
        double y_global_before = getYGlobal(nodes.at(i-1)->cell.y, distances);
        double dx = x_global - x_global_before;
        double dy = y_global - y_global_before;
        double theta = atan2(dx,dy);
        mbot_lcm_msgs::pose2D_t pose;
        pose.x = x_global;
        pose.y = y_global;
        pose.theta = theta;
        path.push_back(pose);
    }
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;

}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath)
{
    std::vector<Node*> new_node_path;
    ////////////////// TODO: Optionally implement a prune_node_path function //////////////////////////
    // This should remove points in the path along the same line
    
    return new_node_path;

}
