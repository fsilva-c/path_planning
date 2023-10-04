#include <ros/ros.h>
#include <fs_path_planning/SphereCloud.h>

class AStar {
private:
    fs_path_planning::SphereCloud sphere_cloud;
    ros::Subscriber sub_sphere_cloud;

public:
    inline void callback_sphere_cloud(const fs_path_planning::SphereCloud &data) { sphere_cloud = data; }
};