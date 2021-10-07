/**
 * @file FieldFeatureOverview.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "PassShare.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Role.h"

#include <iostream>


//#define PLOT_SINGE_TSL(name) \
//  PLOT("representation:FieldFeatureOverview:TimeSinceLast:" #name, theFrameInfo.getTimeSince(statuses[name].lastSeen));
//

void PassShare::operator >> (BHumanMessage& m) const
{
  m.theBHumanArbitraryMessage.queue.out.bin << passTarget.translation.x();
  m.theBHumanArbitraryMessage.queue.out.bin << passTarget.translation.y();
  m.theBHumanArbitraryMessage.queue.out.bin << myNumber;
  m.theBHumanArbitraryMessage.queue.out.bin << passingTo;
  m.theBHumanArbitraryMessage.queue.out.bin << readyReceive;
  m.theBHumanArbitraryMessage.queue.out.bin << readyPass;

  // OLD PASSSHARE REPRESENTATION
  // m.theBHumanArbitraryMessage.queue.out.bin << sharedGoalUtil;
  // m.theBHumanArbitraryMessage.queue.out.bin << myNumber;
  // m.theBHumanArbitraryMessage.queue.out.bin << passingTo;
  // m.theBHumanArbitraryMessage.queue.out.bin << readyReceive;
  // m.theBHumanArbitraryMessage.queue.out.bin << readyPass;
  // m.theBHumanArbitraryMessage.queue.out.bin << role;

  //m.theBSPLStandardMessage.role = (float)role;
  //std::cout<<"role ="<<role<<std::endl;
  m.theBHumanArbitraryMessage.queue.out.finishMessage(this->id());
}

bool PassShare::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());
  m.bin >> passTarget.translation.x();
  m.bin >> passTarget.translation.y();
  m.bin >> myNumber;
  m.bin >> passingTo;
  m.bin >> readyReceive;
  m.bin >> readyPass;

  // OLD PASSSHARE REPRESENTATION
  // ASSERT(m.getMessageID() == id());

  // m.bin >> sharedGoalUtil;

  // m.bin >> myNumber;
  // m.bin >> passingTo;
  // m.bin >> readyReceive;
  // m.bin >> readyPass;
  // m.bin >> role;

  return true;
}


void PassShare::draw() const
{
  #ifdef TARGET_SIM
  
  /**
   * Draw a 3D arc parallel to the field plane.
   * @param id The drawing id.
   * @param xCenter The x coordinate of the center.
   * @param yCenter The y coordinate of the center.
   * @param zCenter The z coordinate of the center.
   * @param radius The radius.
   * @param fromAngle The start angle.
   * @param angleSize The angular size of the arc. The arc is in the range
   *                  [fromAngle ... fromAngle + angleSize].
   * @param thickness The line thickness.
   * @param color The color of the circle.
   */
  #define ARC3D(id, xCenter, yCenter, zCenter, radius, fromAngle, angleSize, thickness, color) \
    do \
    { \
      constexpr Angle _angleStep = pi2 / 32.f; \
      Vector2f _from((xCenter) + std::cos(fromAngle) * (radius), (yCenter) + std::sin(fromAngle) * (radius)); \
      for(Angle _angle = _angleStep; _angle <= (angleSize) - _angleStep; _angle += _angleStep) \
      { \
        Vector2f _to((xCenter) + std::cos(_angle + (fromAngle)) * (radius), (yCenter) + std::sin(_angle + (fromAngle)) * (radius)); \
        LINE3D(id, _from.x(), _from.y(), (zCenter), _to.x(), _to.y(), (zCenter), thickness, color); \
        _from = _to; \
      } \
      Vector2f _to((xCenter) + std::cos((fromAngle) + (angleSize)) * (radius), (yCenter) + std::sin((fromAngle) + (angleSize)) * (radius)); \
      LINE3D(id, _from.x(), _from.y(), (zCenter), _to.x(), _to.y(), (zCenter), thickness, color); \
    } \
    while(false)
  // drawing of the model in the field view
        
  DEBUG_DRAWING3D("representation:PassShare", "field")
  {
    const Role& theRole = static_cast<const Role&>(Blackboard::getInstance()["Role"]);

    if(theRole.role==Role::RoleType::striker || theRole.role==Role::RoleType::undefined)
    {
      const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
      const PassShare& thePassShare = static_cast<const PassShare&>(Blackboard::getInstance()["PassShare"]);

      ColorRGBA lineColor = ColorRGBA(255,255,0);
      if(!thePassShare.readyPass)
      {
        lineColor = ColorRGBA(125,125,125);
      }

      LINE3D("representation:PassShare", theRobotPose.translation.x(), theRobotPose.translation.y(), 10, thePassShare.passTarget.translation.x(), thePassShare.passTarget.translation.y(), 10, 6, lineColor);
    
      SPHERE3D("representation:PassShare", 
        thePassShare.passTarget.translation.x(), thePassShare.passTarget.translation.y(), 10, 
        30, lineColor);

      ColorRGBA dangerZoneColor = ColorRGBA(0,0,0);
      if(thePassShare.tooManyOpponentsNearTarget)
      {
        dangerZoneColor = ColorRGBA(255,125,0);
      }

      ARC3D("representation:PassShare",
        thePassShare.passTarget.translation.x(), thePassShare.passTarget.translation.y(), 10, 
        thePassShare.MAX_ALLOWED_OPPONENT_DISTANCE_FROM_PASS_TARGET, 
        0, pi2, 6, dangerZoneColor
      );
    }
  }
  #endif
}
