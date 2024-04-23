#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher occupancy_grid_pub;
std::string elevation_map_topic, occupancy_grid_topic;
std::string layer = "traversability";

nav_msgs::OccupancyGrid convertToOccupancyGrid(const grid_map::GridMap& map, const std::string& layer) {
    // Find the min and max values in the traversability layer
    float dataMin = map[layer].minCoeffOfFinites();
    float dataMax = map[layer].maxCoeffOfFinites();

    // Use the GridMapRosConverter function to convert to an OccupancyGrid
    nav_msgs::OccupancyGrid occupancy_grid;
    grid_map::GridMapRosConverter::toOccupancyGrid(map, layer, dataMax, dataMin, occupancy_grid);
    // TODO make this a parameter
    for (auto& value : occupancy_grid.data) {
        if (value < 50) {
            value = 0;
        }
    }
    return occupancy_grid;
}

void elevationMapCallback(const grid_map_msgs::GridMap& msg) {
    // Convert the GridMap message to a GridMap object
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(msg, map);

    // Check if the traversability layer exists
    if (!map.exists(layer)) {
        ROS_WARN("%s layer not found in the grid map.", layer);
        return;
    }

    // Convert to OccupancyGrid
    nav_msgs::OccupancyGrid occupancy_grid = convertToOccupancyGrid(map, layer);

    // Publish the occupancy grid
    occupancy_grid_pub.publish(occupancy_grid);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "elevation_map_to_occupancy_grid");
    ros::NodeHandle nh;

    // Get parameters for topics
    nh.param<std::string>("elevation_map_topic", elevation_map_topic, "elevation_mapping/elevation_map_raw");

    nh.param<std::string>("occupancy_grid_topic", occupancy_grid_topic, "/map");

    // Subscribe to the elevation_map topic
    ros::Subscriber elevation_map_sub = nh.subscribe(elevation_map_topic, 1, elevationMapCallback);

    // Publisher for the occupancy grid
    occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>(occupancy_grid_topic, 1, true);

    ros::spin();
    return 0;
}
