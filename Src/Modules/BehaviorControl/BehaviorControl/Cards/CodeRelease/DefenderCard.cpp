/**
 * @file ApproachAndKickCard.cpp
 *
 * This file implements a behavior for approaching the ball and kick it to a given target.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Role.h"
#include "Tools/Math/BHMath.h"
#include <iostream>
CARD(DefenderCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(WalkToBallController),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(WalkToApproach),
  USES(TeamData),
  REQUIRES(FieldBall),
  REQUIRES(LibCheck),
  REQUIRES(RobotModel),
  REQUIRES(TeamBallModel),
  REQUIRES(BallModel),
  REQUIRES(ObstacleModel),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(Role),

  REQUIRES(GameInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (float)(400.f) positionTh,
    (float)(300.f) smallPositionTh,
    }),
});


class DefenderCard : public DefenderCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return false;
  }

  option
  {
    theActivitySkill(BehaviorStatus::defender);

    initial_state(start)
    {
      transition
      {
        if(theRole.role == Role::searcher_1 || theRole.role == Role::searcher_2 || theRole.role == Role::searcher_4)
          goto searchForBall;

        goto walkToLine;
          
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        theStandSkill();
      }
    }

    state(walkToLine)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;

        float obstacleDistance = std::numeric_limits<float>::infinity();
        for(auto obs : theObstacleModel.obstacles)
        {
          float dist = theBallModel.estimate.position.norm();
          if (dist < 300) {
            obstacleDistance =  dist;
          }
        }

        if (obstacleDistance > 300.0) {
          //goto kickBall;
        }
      }
      action
      {
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        int y_offset_from = 0;
        int y_offset_to = 0;
        int x_offset_additional = 0;
        int y_goalie = 0;
        bool goalie_isFallen = false;
          // We want to cover the "internal" pole
        if(theTeamBallModel.position.y()>0){
          //case in which the ball is on the right
          y_offset_from=600;
        }else{
          //case in which the ball is on the left
          y_offset_from=-600;
        }
        
        /* let's find the goalie
        for (auto const& teammate : theTeamData.teammates){ //Teammate utilities
            if (teammate.number==1 and theTeamData.teammates.size()>0){ // Robot #1 is the goalie
              y_goalie = teammate.theRobotPose.translation.y();
              if(teammate.theBehaviorStatus.activity == BehaviorStatus::fallen){
                // add part abou fallen robot. mo devo anna e continuo stanotte o domani
              }
            }
        }*/
        /*/ but if the ball is close to the goal..
        if(theTeamBallModel.position.y()>-300 && theTeamBallModel.position.y()<300){
          if (theTeamData.teammates.at(0).theRobotPose.translation.y()>-300 && theTeamData.teammates.at(0).theRobotPose.translation.y()<300){
            y_offset_from=700;
            x_offset_additional = 300;
          }
        }*/
        float yOffset = 0;
        Pose2f fromTarget = Vector2f(theFieldDimensions.xPosOwnGoalPost, 0.0);
        Pose2f toTarget = theTeamBallModel.position;
        fromTarget = Pose2f(fromTarget.translation.x(),fromTarget.translation.y()+y_offset_from);
        
        float xOffset;
        xOffset = theTeamBallModel.position.x() + 3500.f;
        std::cout<<"From target x:"<<fromTarget.translation.x()<<" y:"<<fromTarget.translation.y()<<"\n";
        std::cout<<"To   target x:"<<toTarget.translation.x()<<" y:"<<toTarget.translation.y()<<"\n";
        
        /* THIS IS JUST TO REMEMBER THE PARAMETERS
                  (const Pose2f&) fromTarget,
                  (const Pose2f&) toTarget,
                  (float)(0.0) offsetX,
                  (float)(0.0) offsetY,
                  (bool)(true) useLeftFoot,
                  (float)(1.0) gainWhenBallOnSide,
                  (float)(0.05) kp1,
                  (float)(-0.003) kp2,
                  (float)(0.003) kp3
        */
        // I NEED TO KNOW THE ERRORS
        // offsetX and offsetY are the gap between the robot's foot and the ball in the front and in the side respectively
        Vector3f error;
        float forwardError, sidewaysError, rotationError;
        Pose2f targetPoint1, targetPoint2, feedbackPoint1, feedbackPoint2;

        // Initialize the Points on the robot's food to get the vector based on the choosen left/right foot
        if (false == true) {
            feedbackPoint1 = theLibCheck.rel2Glob(theRobotModel.soleLeft.translation.x(), theRobotModel.soleLeft.translation.y());
            feedbackPoint2 = theLibCheck.rel2Glob( theRobotModel.soleLeft.translation.x()+xOffset, theRobotModel.soleLeft.translation.y()+yOffset );
        } else {
            feedbackPoint1 = theLibCheck.rel2Glob(theRobotModel.soleRight.translation.x(), theRobotModel.soleRight.translation.y());
            feedbackPoint2 = theLibCheck.rel2Glob(theRobotModel.soleRight.translation.x()+xOffset, theRobotModel.soleRight.translation.y()-yOffset );
        }
        
        // Initialize the points to position the robot. First point is the ball and second points is the target optained as a parameter
        // if (theTeamBallModel.position.x() > p.target.translation.x()) {
        targetPoint1 = fromTarget;
        targetPoint2 =  toTarget;
        // } else {
        //   targetPoint1 =  p.target;
        //   targetPoint2.translation = theTeamBallModel.position;
        // }

        // --------------------Calculate orientation error------------------------ 
        Vector2f target_vec(targetPoint2.translation.x() - targetPoint1.translation.x(), targetPoint2.translation.y() - targetPoint1.translation.y());
        Vector2f feedback_vec(feedbackPoint2.translation.x()-feedbackPoint1.translation.x(), feedbackPoint2.translation.y() - feedbackPoint1.translation.y());

        float dot = target_vec.dot(feedback_vec);
        float det = target_vec.x()*feedback_vec.y() - target_vec.y()*feedback_vec.x();
        rotationError = -(atan2( det, dot ) * 180)/pi;
        // --------------------Get Sideways Error (Contact Distance)------------------------  
        // -------------------- Shortest Perpendiculr Distance between two lines --------------------
        // Equation of the line in the form of Ax + By + C = 0
        
        float A = -(targetPoint2.translation.y() - targetPoint1.translation.y()) / (targetPoint2.translation.x() - targetPoint1.translation.x());
        float B = 1;
        float C = -(targetPoint1.translation.y() + A*targetPoint1.translation.x());
        // Estimated Point of contact
        float x1 = feedbackPoint2.translation.x();
        float y1 = feedbackPoint2.translation.y();

        // Calculate perpendicular distance from the point to the line d = |A*x_1 + B*y_1 + C| / (A^2 + B^2)^(1/2)
        sidewaysError = (A*x1 + B*y1 + C) / sqrt(pow(A,2) + pow(B,2));

        // -------------------- Get Forward Error(Closest point on target vector) --------------------
        float m1 = (targetPoint2.translation.y() - targetPoint1.translation.y()) / (targetPoint2.translation.x() - targetPoint1.translation.x());
        float m2 = (targetPoint2.translation.y() - feedbackPoint2.translation.y()) / (targetPoint2.translation.x()-feedbackPoint2.translation.x());
        float d = sqrt(pow((targetPoint2-feedbackPoint2).translation.x(), 2) + pow((targetPoint2-feedbackPoint2).translation.y(), 2)) * cos(atan2( m2-m1, (1+(m1*m2)) ));
        float m = (targetPoint2.translation.y() - targetPoint1.translation.y()) / (targetPoint2.translation.x() - targetPoint1.translation.x());
        Pose2f pointOnLine, point1, point2;

        point1.translation.y() = targetPoint2.translation.y() + ((m*d)/sqrt(1+pow(m,2)));
        point2.translation.y() = targetPoint2.translation.y() - ((m*d)/sqrt(1+pow(m,2)));
        point1.translation.x() = targetPoint2.translation.x() - (targetPoint2.translation.y() - point1.translation.y())/m;
        point2.translation.x() = targetPoint2.translation.x() - (targetPoint2.translation.y() - point2.translation.y())/m;
        
        if ((pow((point2-feedbackPoint2).translation.x(),2) + pow((point2-feedbackPoint2).translation.y(),2)) < (pow((point1-feedbackPoint2).translation.x(),2) + pow((point1-feedbackPoint2).translation.y(),2))) {
            pointOnLine = point2;
        } else { 
            pointOnLine = point1;
        }

        if (theTeamBallModel.position.x() > pointOnLine.translation.x()) {
            forwardError = sqrt(pow((pointOnLine-theTeamBallModel.position).translation.x(), 2)+pow((pointOnLine-theTeamBallModel.position).translation.y(), 2));
        }else {
            forwardError = -sqrt(pow((pointOnLine-theTeamBallModel.position).translation.x(), 2)+pow((pointOnLine-theTeamBallModel.position).translation.y(), 2));
        }

        // I NEED TO KNOW THE ERROR
        std::cout<<"[Debug] F S R :"<<forwardError;
        std::cout<<"      :"<<sidewaysError;
        std::cout<<"      :"<<rotationError<<"\n";

        if(forwardError>-20 && forwardError<20 && sidewaysError>-40 && sidewaysError<40 && rotationError>-5 && rotationError<5 ){
          theStandSkill();
        }else{
          theWalkToBallControllerSkill(fromTarget, toTarget,
                                     xOffset, yOffset,
                                     false, 2.5);
        }

      }
    }
    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }
};

MAKE_CARD(DefenderCard);
