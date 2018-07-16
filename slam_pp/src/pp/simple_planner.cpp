#include "pp/simple_planner.hpp"

using namespace uavos;

void NoPlanner::noPlannerCallback(const ros::TimerEvent &event){
    setPlannedWaypointNWU(getInputWaypointNWU());
}
