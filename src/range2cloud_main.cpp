#include "range2cloud_node.h"

using namespace cloud2range;

int main(int argc, char** argv) {
  ros::init(argc, argv, "range2cloud");

  Range2CloudNode node(ros::NodeHandle("~"));

  ros::spin();
}
