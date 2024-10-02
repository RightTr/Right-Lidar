#include "cylinder.h"
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cylinder_node");
    ros::NodeHandle nh;

    PclCylinder pclcylinder(nh);

    return 0;
}
