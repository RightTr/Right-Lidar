#include "filter.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle nh;

    PclFilter pclcore(nh);


    return 0;
}