#include "LoopCycle.h"
#include "Math/InverseKinematics.h"
#include "Math/ForwardKinematics.h"

void LoopCycle::control()
{
  /*Eigen7f xp = local->getFirstPosition();
  Eigen3f pos = Eigen3f::Zero();
  Eigen3x3f rotation = Eigen3x3f::Zero();
  forward_kinematics(xp,pos, rotation);
  Eigen6f input = Eigen6f::Zero();
  input.head(3) << pos;
  input.tail(3) << rotation.eulerAngles(0,1,2);*/
  Eigen6f input = planner->getPositionOrientationDemand();
  Eigen7f u = localControl->controller(input);
  local->set(u);
}

void LoopCycle::torqueMode()
{
	local->torqueMode();
}

void LoopCycle::positionMode()
{
	local->positionMode();
}