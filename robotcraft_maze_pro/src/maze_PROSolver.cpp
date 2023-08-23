
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

struct Vec2D {
    int32_t x, y;
};

std::array<Vec2D, 4> cardDirs = {
    Vec2D{1, 0}, Vec2D{-1, 0}, Vec2D{0, 1}, Vec2D{0, -1}
};

enum CellType {
    Wall = -5, // Wall can't be traversed
    Empty = -4, // Empty cells are reachable if their neighbors are reachable. Replaced with distance
    Goal = -3, // Goal cells are reachable if their neighbors are reachable. Replaced by Reached
    Reached = -2, // Goals that could be reached. 
    Start = -1
};

void map_fun(const std::vector<int8_t>& data, uint32_t width, uint32_t height){
    ROS_INFO("width: %u height: %u", width, height);

    auto origin = Vec2D{5, 5};
    auto goal = Vec2D{530, 530};

    std::vector<int16_t> grid;
    grid.reserve(data.size());
    // Convert occupancy grid to our own type of grid.
    for(auto cel : data){
        if(cel == 0) grid.push_back(CellType::Empty);
        else grid.push_back(CellType::Wall);
    }

    // Mark the start and end Goal.
    grid[origin.y * width + origin.x] = CellType::Start;
    grid[goal.y * width + goal.x] = CellType::Goal;

    std::vector<Vec2D> wave, next;
    wave.push_back(origin);
    int16_t distance = 1;

    while(wave.size() > 0){
        // Expand all neighbors of the reachable cells.
        for(auto& wa : wave){
            for(auto& di : cardDirs){
                auto pos = Vec2D{wa.x + di.x, wa.y + di.y};
                if(pos.x < 0 || pos.x >= width)  continue;
                if(pos.y < 0 || pos.y >= height) continue;
                switch(grid[pos.y * width + pos.x]){
                    case CellType::Empty: {
                        grid[pos.y * width + pos.x] = distance;
                        next.push_back(pos);
                        break;
                    }
                    case CellType::Goal: {
                        grid[pos.y * width + pos.x] = CellType::Reached;
                        next.push_back(pos);
                        break;
                    }
                }
            }
        }
        // The reached cells will be expanded.
        wave = std::move(next);
        distance += 1;
    }

    if(grid[goal.y * width + goal.x] == CellType::Reached){
        ROS_INFO("Path found !");
    } else {
        ROS_INFO("Path not found !");
        return;
    }

    // Replace CellType::Goal with a distance value.
    grid[goal.y * width + goal.x] = distance + 1;

    // Generate the path.
    std::vector<Vec2D> path;
    path.push_back(goal);
    while(grid[path.back().y * width + path.back().x] != CellType::Start){
        Vec2D lowest = Vec2D{0, 0};
        Vec2D current = path.back();
        // Search for the lowest neighbor.
        for(auto& di : cardDirs){
            auto pos = Vec2D{current.x + di.x, current.y + di.y};
            if(pos.x < 0 || pos.x >= width)  continue;
            if(pos.y < 0 || pos.y >= height) continue;
            auto celval = grid[pos.y * width + pos.x];
            if(celval < CellType::Start) continue;
            auto prev = Vec2D{current.x + lowest.x, current.y + lowest.y};
            if(celval < grid[prev.y * width + prev.x]){
                lowest = di;
            }
        }
        path.push_back(Vec2D{current.x + lowest.x, current.y + lowest.y});
    }

    // Simplifies the path.
    // If 3 cells are aligned, it is not necessary to add a waypoint to the middle one.
    std::vector<Vec2D>::iterator head = path.rbegin() + 2;
    std::vector<Vec2D>::iterator middle = path.rbegin() + 1;
    std::vector<Vec2D>::iterator tail = path.rbegin();

    std::vector<Vec2D> betterPath;
    while(head != path.rend()){
        auto diffFront = Vec2D{ head->x - middle->x , head->y - middle->y };
        auto diffBack  = Vec2D{ middle->x - tail->x, middle->y - tail->y };
        if(diffFront.x != diffBack.x || diffFront.y != diffBack.y) betterPath.push_back(*middle);

        head ++; middle ++; tail ++;
    }

    for(auto& ve : betterPath){
        ROS_INFO("x: %i y: %i", ve.x, ve.y);
    }

    // I am stuck here : How to tell where are the waypoints in the real world ?
    // the map coordinates must be transformed to real world ones, to know what directions the robot should go.
    // For exemple is the map rotated, or scaled in some way, we need to apply these transformations
    // to the waypoints.
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map){
    map_fun(map.get()->data, map.get()->info.width, map.get()->info.height);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "pro_solver");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, map_callback);
    ros::spin();

    return 0;
}
