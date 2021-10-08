/**
 * @file BallController.cpp
 *
 *
 * @author Akshay Dhonthi
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/spqr_representations/BallPath.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Settings.h"

#include "Tools/Math/BHMath.h"
#include <iostream>

#define __STRINGIFY_I(arg) #arg
#define __STRINGIFY(arg) __STRINGIFY_I(arg)
#define DEBUG_CODE(code) \
  std::cout<<__STRINGIFY(code)<<": "<<std::to_string(code)<<std::endl;

using namespace std;

CARD(StrikerFighterCard,
{,
  CALLS(Activity),
  CALLS(Say),

  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(WalkToBallController),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(InWalkKick),
  CALLS(Kick),
  USES(LibCheck),
  REQUIRES(RobotModel),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(BallModel),
  REQUIRES(ObstacleModel),
  REQUIRES(BallPath),
  REQUIRES(GameInfo),

  LOADS_PARAMETERS(
  {,
  (float) walkSpeed,
  (int) ballNotSeenTimeout,

  (bool) oppKickFlag,

  (float) ball_distance_threshold_pre,
  (float) ball_distance_threshold_post,
  
  (float) opponent_distance_threshold_pre,
  (float) opponent_distance_threshold_post,

  (float) opponent_minimum_local_x,
  
  }),
});

class StrikerFighterCard : public StrikerFighterCardBase
{
  
  // needs to be declared like this to be passed as an argument in the pre/postconds
  std::function<bool(Obstacle)> isAhead = [&](Obstacle opp) {
    /*
    Vector2f oppPosition = opp.center;
    Vector2f oppPosRelative = theLibCheck.glob2Rel(oppPosition.x(), oppPosition.y()).translation;
    if (theGameInfo.kickingTeam == Global::getSettings().teamNumber) {
      DEBUG_CODE(oppPosRelative.x());
      if (oppPosRelative.x() > opponent_minimum_local_x) {
        std::cout << "OLEEEEEEEE" << std::endl;
      }
    }
    return oppPosRelative.x() > opponent_minimum_local_x;
    */

    /*
    Vector2f oppPosition = opp.center;
    return oppPosition.x() > opponent_minimum_local_x;
    */

    return true;
  };

  bool preconditions() const override
  {
    std:cout << "=====================================================" << std::endl;
    Vector2f nearestOpponentPosition = theLibCheck.nearestOpponent().translation;
    nearestOpponentPosition = theLibCheck.glob2Rel(nearestOpponentPosition.x(), nearestOpponentPosition.y()).translation;

    bool ballNear = theBallModel.estimate.position.norm() < ball_distance_threshold_pre;
    bool oppNear = nearestOpponentPosition.norm() < opponent_distance_threshold_pre;
    if(theGameInfo.kickingTeam == Global::getSettings().teamNumber){
      DEBUG_CODE(ballNear);
      DEBUG_CODE(nearestOpponentPosition.norm());
      DEBUG_CODE(opponent_distance_threshold_pre);
      DEBUG_CODE(oppNear);
    }
    return ballNear && oppNear;
  }

  bool postconditions() const override
  {
    Vector2f nearestOpponentPosition = theLibCheck.nearestOpponent().translation;
    nearestOpponentPosition = theLibCheck.glob2Rel(nearestOpponentPosition.x(), nearestOpponentPosition.y()).translation;

    bool ballNear = theBallModel.estimate.position.norm() < ball_distance_threshold_post;
    bool oppNear = nearestOpponentPosition.norm() < opponent_distance_threshold_post;

    return !ballNear || !oppNear;

    // return theBallModel.estimate.position.squaredNorm() > std::pow(ball_distance_threshold_post, 2) &&
    // nearestOpponentPosition.squaredNorm() > std::pow(opponent_distance_threshold_post, 2);
  }

  option
  {
    theActivitySkill(BehaviorStatus::goalieCore);
    initial_state(start)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        else
        {
          goto lookforBall;
        }
      }
      action
      {
        //theSaySkill("striker fighting");
        theStandSkill();
        theLookForwardSkill();
      }
    }

    state(lookforBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto approach_Ball;
      }
      action
      {
        theStandSkill();
        theLookLeftAndRightSkill();
      }
    }

    state(approach_Ball)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;

        float obstacleDistance = std::numeric_limits<float>::infinity();
        for(auto obs : theObstacleModel.obstacles)
        {
          float dist = theLibCheck.distance(obs.center, theBallModel.estimate.position);
          if (dist < obstacleDistance) {
            obstacleDistance =  dist;
          }
        }

        if (obstacleDistance > 300.0 && oppKickFlag) {
          goto kickBall;
        }
      }
      action
      {
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));

        float xOffset = 180.0, yOffset = 0.0;
        Pose2f fromTarget = Vector2f(theFieldDimensions.xPosOwnGoalPost, 0.0);
        Pose2f toTarget = theTeamBallModel.position;
        theWalkToBallControllerSkill(fromTarget, toTarget,
                                     xOffset, yOffset,
                                     false, 2.5, 0.05, -0.003, 0.003);

      }
    }

    state(kickBall) {
      transition {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
          
        float obstacleDistance = std::numeric_limits<float>::infinity();
        for(auto obs : theObstacleModel.obstacles)
        {
          float dist = theLibCheck.distance(obs.center, theBallModel.estimate.position);
          if (dist < obstacleDistance) {
            obstacleDistance =  dist;
          }
        }
        if (obstacleDistance <= 300.0) {
          goto approach_Ball;
        }

      }
      action {
        float xOffset = 180.0, yOffset = 0.0;
        Pose2f fromTarget = theTeamBallModel.position;
        Pose2f toTarget = Vector2f(theFieldDimensions.xPosOpponentGoalPost, 0.0);

        if (theRobotPose.translation.y() < 0.0) {
          theWalkToBallControllerSkill(fromTarget, toTarget,
                                        xOffset, yOffset,
                                     false, 2.5, 0.05, -0.003, 0.003);
        }
        else {
          theWalkToBallControllerSkill(fromTarget, toTarget,
                                        xOffset, yOffset,
                                     true, 2.5, 0.05, -0.003, 0.003);
        }

      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto approach_Ball;
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }

  }
};

MAKE_CARD(StrikerFighterCard);
