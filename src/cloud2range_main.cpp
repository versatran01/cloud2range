#include "cloud2range_node.h"

using namespace cloud2range;

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud2range");

  Cloud2RangeNode node(ros::NodeHandle("~"));

  ros::spin();
}
