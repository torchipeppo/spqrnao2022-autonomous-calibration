/**
 * @file WalkToBallControllerDefender.cpp
 *
 * This Skill computes the error in forward, sideways and angular errors from the robot's foot to the ball and
 * walks at relative speed based on P controller to place the robot behind the ball.
 * 
 * @author Graziano Specchi with the guidelines of Dr.Akshay
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Configuration/FieldDimensions.h"

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/BHMath.h"
#include <iostream>
#include <math.h>

#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Sensing/RobotModel.h"

SKILL_IMPLEMENTATION(WalkToBallControllerDefenderImpl,
{,
  IMPLEMENTS(WalkToBallControllerDefender),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  REQUIRES(FieldBall),
  REQUIRES(PassShare),
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  CALLS(Activity),

  MODIFIES(MotionRequest),
  MODIFIES(BehaviorStatus),

//   My Reps
  REQUIRES(TeamBallModel),
  REQUIRES(RobotModel),


  DEFINES_PARAMETERS(
  {,
    (float)(0.f) rotationSpeed,
    (float)(0.f) sidewaysSpeed,
    (float)(0.f) forwardSpeed,
    //(float)(0.05) kp1,
    //(float)(-0.003) kp2,
    //(float)(0.003) kp3,
    (Vector3f)() error,
  }),
});

class WalkToBallControllerDefenderImpl : public WalkToBallControllerDefenderImplBase
{
  void execute(const WalkToBallControllerDefender& p) override
  {
        // offsetX and offsetY are the gap between the robot's foot and the ball in the front and in the side respectively
        float xOffset = p.offsetX;
        float yOffset = p.offsetY;
        Vector3f error;
        float forwardError, sidewaysError, rotationError;
        Pose2f targetPoint1, targetPoint2, feedbackPoint1, feedbackPoint2;

        // Initialize the Points on the robot's food to get the vector based on the choosen left/right foot
        if (p.useLeftFoot == true) {
            feedbackPoint1 = theLibCheck.rel2Glob(theRobotModel.soleLeft.translation.x(), theRobotModel.soleLeft.translation.y());
            feedbackPoint2 = theLibCheck.rel2Glob( theRobotModel.soleLeft.translation.x()+xOffset, theRobotModel.soleLeft.translation.y()+yOffset );
        } else {
            feedbackPoint1 = theLibCheck.rel2Glob(theRobotModel.soleRight.translation.x(), theRobotModel.soleRight.translation.y());
            feedbackPoint2 = theLibCheck.rel2Glob(theRobotModel.soleRight.translation.x()+xOffset, theRobotModel.soleRight.translation.y()-yOffset );
        }
        
        // Initialize the points to position the robot. First point is the ball and second points is the target optained as a parameter
        // if (theTeamBallModel.position.x() > p.target.translation.x()) {
        targetPoint1 = p.fromTarget;
        targetPoint2 =  p.toTarget;
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

        // Set Nan values as zero error and multiply with the gains
        if (isnan(rotationError)) {rotationSpeed = 0.0;} else {rotationSpeed = p.kp1*rotationError;}    
        if (isnan(sidewaysError)) {sidewaysSpeed = 0.0;} else {sidewaysSpeed = p.kp2*sidewaysError;}
        if (isnan(forwardError)) {forwardSpeed = 0.0;}

        else if (forwardError < 0.0 && forwardError > -500.0) {forwardSpeed = p.kp3*p.gainWhenBallOnSide*forwardError;} else {forwardSpeed = p.kp3*forwardError;}

        // DEBUG Prints
        // std::cout << "Error Norm: " << error.norm() << std::endl;
        // std::cout << rotationError << ", " << rotationSpeed  <<  std::endl;
        // std::cout << forwardError << ", " << forwardSpeed  <<  std::endl;
        // std::cout << sidewaysError << ", " << sidewaysSpeed  <<  std::endl;

        // std::cout << rotationSpeed << ", " << forwardSpeed  << ", " << sidewaysSpeed << std::endl;
        // std::cout << forwardError << ", " << sidewaysError  << ", " << rotationError << std::endl;

        // Walk at relative speed using the computed velocities.

        /*Vector3f NewSpeeds = Vector3f(forwardSpeed,sidewaysSpeed,rotationSpeed);

        /if (NewSpeeds.norm() < p.standThreshold) {*/
        if(std::abs(forwardError)<p.forwardThreshold && std::abs(sidewaysError)<p.sidewaysThreshold && std::abs(rotationError)<p.rotationThreshold){
          theMotionRequest.motion = MotionRequest::stand;
        }
        else {
          theMotionRequest.motion = MotionRequest::walk;
          theMotionRequest.walkRequest.mode = WalkRequest::relativeSpeedMode;
          theMotionRequest.walkRequest.speed = Pose2f(rotationSpeed, forwardSpeed, sidewaysSpeed);
          theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
        }
        theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const WalkToBallControllerDefender&) const override
  {
    return theMotionInfo.motion == MotionRequest::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkToBallControllerDefenderImpl);
