/**
   \file main.cpp
   \brief Main entry point of the program, starts an instance of Planner
*/

//###################################################
//                      HYBRID A* ALGORITHM
//  AUTHOR:   Karl Kurzer
//  WRITTEN:  2015-03-02
//  REVISED: Jialiang Han
//###################################################

#include <ros/ros.h>
#include "constants.h"
#include "planner.h"

//###################################################
//                                               MAIN
//###################################################
/**
   \fn main(int argc, char** argv)
   \brief Starting the program
   \param argc The standard main argument count
   \param argv The standard main argument value
   \return 0
*/
int main(int argc, char** argv) {

  google::InitGoogleLogging(argv[0]);

  google::ParseCommandLineFlags(&argc, &argv, true);

  google::InstallFailureSignalHandler();

  google::EnableLogCleaner(5);

  ros::init(argc, argv, "hybrid_a_star");

  HybridAStar::Planner hy;
  hy.MakePlan();

  ros::spin();
  return 0;
}
