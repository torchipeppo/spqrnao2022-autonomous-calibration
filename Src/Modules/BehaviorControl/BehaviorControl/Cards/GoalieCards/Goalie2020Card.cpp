#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"

//#define SPQR_DEBUG_GOALIE

CARD(Goalie2020Card,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(SpecialAction),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(LibCheck),
  REQUIRES(TeamBallModel),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  USES(TeamData),
  DEFINES_PARAMETERS(
  {,
    (float)(300.f) GOALIE_DISPLACEMENT,
    (bool) case1,
    (bool) case2,
    (bool) case3,
  }),
});

class Goalie2020Card : public Goalie2020CardBase
{ 
  bool preconditions() const override
  {
    SPQR::ConfigurationParameters();

    float myDistanceToBall = theBallModel.estimate.position.norm();
    bool iAmMostNearPlayer = true;
    for(unsigned i = 0; i < theTeamData.teammates.size(); i++){
        if((theTeamData.teammates.at(i).theRobotPose.translation -
                theTeamBallModel.position).norm() < (myDistanceToBall+400.f)){
            iAmMostNearPlayer = false;
        }
    }

    return ( ( ((int)theGameInfo.setPlay != SET_PLAY_GOAL_FREE_KICK && !theLibCheck.isGoalieInKickAwayRange             //conditions to exit from goalieKickAway state in Goalie2020
          && (theBallModel.estimate.position.norm() > 175 && !iAmMostNearPlayer)) ||
          (!theLibCheck.isBallInKickAwayRange && !iAmMostNearPlayer) ) || (theLibCheck.timeSinceBallWasSeen > 1000)); 
  }

  bool postconditions() const override
  {
    Vector2f velocity=theBallModel.estimate.velocity;
    Vector2f position=theBallModel.estimate.position;

    float myDistanceToBall = theBallModel.estimate.position.norm();
    bool iAmMostNearPlayer = true;
    for(unsigned i = 0; i < theTeamData.teammates.size(); i++){
        if((theTeamData.teammates.at(i).theRobotPose.translation -
                theTeamBallModel.position).norm() < (myDistanceToBall+400.f)){
            iAmMostNearPlayer = false;
        }
    }

    return ((velocity.x() < 0 && velocity.norm()/position.norm() >= 0.7) ||                                             // has to dive
    ((int)theGameInfo.setPlay == SET_PLAY_GOAL_FREE_KICK && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber) ||    // goal free kick
    (   (  (case1                                                                                                       // case1: in gotoGoaliePosition, backTrackInPose, mainLoop, or goalieOutOfPoles
            && theBallModel.estimate.velocity.norm() < (float) SPQR::MOVING_BALL_MIN_VELOCITY // norm < 200
            && theLibCheck.isBallInKickAwayRange    // palla < 900
            && theLibCheck.isGoalieInKickAwayRange)  // -4500 < x < -3700  &&  -1100 < y <1100
        || (case2                                                                                                       // case2: in turnToBall
            && theBallModel.estimate.velocity.norm() == 0 // valutare di fare 100
            && theLibCheck.isBallInKickAwayRange
            && theLibCheck.isBallInArea)    
        || (case3                                                                                                       // case3: in mainLoop2
            && theBallModel.estimate.velocity.norm() < (float) SPQR::MOVING_BALL_MIN_VELOCITY // norm < 200
            && theBallModel.estimate.position.norm() < 800
            && iAmMostNearPlayer)
        )
        && theLibCheck.timeSinceBallWasSeen < 800    && theGameInfo.state == STATE_PLAYING 
        && theBallModel.estimate.position.x() != 0.0 && theBallModel.estimate.position.y() != 0.0
    ));
  }

  option
  {
    theActivitySkill(BehaviorStatus::goalie2020);
    
    initial_state(start)
    {
        transition
        {

            if( !theLibCheck.isGoalieInStartingPosition ){
                case1 = true;
                case2 = false;
                case3 = false;
                goto gotoGoaliePosition;
            }

            if(state_time > 500){
                case1 = false;
                case2 = false;
                case3 = false;
                goto turnToOpponentGoal;
            }
        }
        action
        {
            theStandSkill();
            theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
        }
    }

    state(gotoGoaliePosition) // control conditions, too restrictive [comment from previous version]
    {
        transition
        {
            #ifdef SPQR_DEBUG_GOALIE
                std::cerr << "gotogoalieposition" << std::endl;
            #endif

            if(theLibCheck.timeSinceBallWasSeen < 800){

                Vector2f velocity = theBallModel.estimate.velocity;
                Vector2f position = theBallModel.estimate.position;
                Pose2f globalCoordBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());

                if(globalCoordBall.translation.x() > theFieldDimensions.xPosOwnPenaltyArea+200 && 
                  globalCoordBall.translation.x() < -0.55f*SPQR::FIELD_DIMENSION_X && velocity.x()<100)
                    goto goalieOutOfPoles;
            }

            if( theLibCheck.isValueBalanced(theRobotPose.translation.x(), (theFieldDimensions.xPosOwnGroundline + 300) + GOALIE_DISPLACEMENT, SPQR::GOALIE_POSE_X_TOLLERANCE) &&
                    theLibCheck.isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y, SPQR::GOALIE_POSE_Y_TOLLERANCE) ) {
                case1 = false;
                goto turnToOpponentGoal;
            }

            if( theLibCheck.isGoalieInAngle )
                goto backTrackInPose;
        }
        action
        {
            if(std::abs(theLibCheck.angleToTarget( (theFieldDimensions.xPosOwnGroundline + 300) + GOALIE_DISPLACEMENT,
                                                     SPQR::GOALIE_BASE_POSITION_Y)) > 10.f/*Angle::fromDegrees(10.f)*/){
                theLookLeftAndRightSkill();
                
                theWalkToTargetSkill(Pose2f(50.f, 50.f, 50.f),
                         Pose2f(theLibCheck.angleToTarget( (theFieldDimensions.xPosOwnGroundline + 300) + GOALIE_DISPLACEMENT,
                                                             SPQR::GOALIE_BASE_POSITION_Y), 0.f, 0.f));
            }
            else{
                theLookLeftAndRightSkill();

                theWalkToTargetSkill(Pose2f(50.f,50.f,50.f),
                          theLibCheck.glob2Rel( (theFieldDimensions.xPosOwnGroundline + 300) + GOALIE_DISPLACEMENT,
                                                 SPQR::GOALIE_BASE_POSITION_Y));
            }
        }
    }

    state(backTrackInPose)
    {
        transition
        {
            #ifdef SPQR_DEBUG_GOALIE
            std::cerr << "backtrackinpose" << std::endl;
            #endif

            if(theLibCheck.timeSinceBallWasSeen < 800){

                Vector2f velocity = theBallModel.estimate.velocity;
                Vector2f position = theBallModel.estimate.position;
                Pose2f globalCoordBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());

                if(globalCoordBall.translation.x() > theFieldDimensions.xPosOwnPenaltyArea+200 && 
                  globalCoordBall.translation.x() < -0.55f*SPQR::FIELD_DIMENSION_X && velocity.x()<100)
                    goto goalieOutOfPoles;
            }

            #ifdef SPQR_DEBUG_GOALIE
            std::cerr << "backtrack balanced" <<  (theLibCheck.isValueBalanced(theRobotPose.translation.x(), theFieldDimensions.xPosOwnGroundline + 300, SPQR::GOALIE_POSE_X_TOLLERANCE) &&
                                                   theLibCheck.isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y, SPQR::GOALIE_POSE_Y_TOLLERANCE)) << std::endl;
            #endif
            if( theLibCheck.isValueBalanced(theRobotPose.translation.x(), theFieldDimensions.xPosOwnGroundline + 300, SPQR::GOALIE_POSE_X_TOLLERANCE) &&
                    theLibCheck.isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y, SPQR::GOALIE_POSE_Y_TOLLERANCE) )
                goto mainLoop;

            #ifdef SPQR_DEBUG_GOALIE
            std::cerr << "backtrack " << theLibCheck.norm(theRobotPose.translation.x() - theFieldDimensions.xPosOwnGroundline + 300,
                                                                theRobotPose.translation.y() - SPQR::GOALIE_BASE_POSITION_Y) << std::endl;
            #endif

            if( theLibCheck.norm(theRobotPose.translation.x() - theFieldDimensions.xPosOwnGroundline + 300,
                                    theRobotPose.translation.y() - SPQR::GOALIE_BASE_POSITION_Y) > theFieldDimensions.yPosLeftGoal)
                goto gotoGoaliePosition;

            if (!theLibCheck.isGoalieInAngle ) {
                case1 = false;
                goto turnToOpponentGoal;
            }
        }
        action
        {
            if(theLibCheck.timeSinceBallWasSeen < 2000 )
                theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
            else
                theLookLeftAndRightSkill();
            if( !theLibCheck.isValueBalanced(theRobotPose.translation.x(), theFieldDimensions.xPosOwnGroundline + 300, SPQR::GOALIE_POSE_X_TOLLERANCE)
                    || !theLibCheck.isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y, SPQR::GOALIE_POSE_Y_TOLLERANCE) )
                theWalkToTargetSkill(Pose2f(50.f, 50.f, 50.f), theLibCheck.glob2Rel( (theFieldDimensions.xPosOwnGroundline + 300), SPQR::GOALIE_BASE_POSITION_Y));
            else 
                theStandSkill();    //Added to avoid error in SimRobot console
        }
    }


   state(turnToOpponentGoal) // Actually turn to the "right" position according to the ball position.
    {
        transition
        {
            #ifdef SPQR_DEBUG_GOALIE
            std::cerr << "turntoopponentgoal" << std::endl;
            #endif
            if( theLibCheck.isGoalieInAngle ) {
                case1 = true;
                goto mainLoop;
            }
        }
        action
        {
            theLookLeftAndRightSkill();

            if( theRobotPose.rotation >= 0.17f)
                theWalkAtAbsoluteSpeedSkill(Pose2f(-.6f, 0.f, 0.f));
            else if ( theRobotPose.rotation < -0.17f)
                theWalkAtAbsoluteSpeedSkill(Pose2f(.6f, 0.f, 0.f));
            else 
                theStandSkill();   //Added to avoid error in SimRobot console 
        }
    }

    state(mainLoop)
    {
        transition
        {
            #ifdef SPQR_DEBUG_GOALIE
            std::cerr << "mainLoop" << std::endl;
            #endif
            Vector2f velocity = theBallModel.estimate.velocity;
            Vector2f position = theBallModel.estimate.position;


            if(theLibCheck.timeSinceBallWasSeen < 800){

                Pose2f globalCoordBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());

                if(globalCoordBall.translation.x() > theFieldDimensions.xPosOwnPenaltyArea+200 && globalCoordBall.translation.x() < -0.55f*SPQR::FIELD_DIMENSION_X && velocity.x()<100)
                    goto goalieOutOfPoles;
            }

            if( theLibCheck.norm(theRobotPose.translation.x() - theFieldDimensions.xPosOwnGroundline + 300,
                                    theRobotPose.translation.y() - SPQR::GOALIE_BASE_POSITION_Y) > 50 )
                goto backTrackInPose;

            if( theLibCheck.norm(theRobotPose.translation.x() - theFieldDimensions.xPosOwnGroundline + 300,
                                    theRobotPose.translation.y() - SPQR::GOALIE_BASE_POSITION_Y) > 500 )
                goto gotoGoaliePosition;

            if (!theLibCheck.isGoalieInAngle) {
                case1 = false;
                goto turnToOpponentGoal;
            }
        }
        action
        {
            if(theLibCheck.timeSinceBallWasSeen < 2000){
                theStandSkill();
                theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
            }
            else if(theLibCheck.timeSinceBallWasSeen < 5000){
                theStandSkill();
                //theLookAtGlobalBallSkill()
                Pose2f globalBall = theLibCheck.glob2Rel(theTeamBallModel.position.x(),theTeamBallModel.position.y());
                theLookAtPointSkill(Vector3f(globalBall.translation.x(), globalBall.translation.y(), 0));
            }
            else{
                theStandSkill();
                theLookLeftAndRightSkill();
            }
        }
    }

    state(goalieOutOfPoles)
    {
        transition
        {
            #ifdef SPQR_DEBUG_GOALIE
            std::cerr << "goalieOutPoles" << std::endl;
            #endif

            if(std::abs(theRobotPose.translation.x() - theLibCheck.goaliePosition.x()) < 100 &&
               std::abs(theRobotPose.translation.y() - theLibCheck.goaliePosition.y()) < 100 &&
               std::abs(theBallModel.estimate.position.angle()) < 5_deg){
                case1 = false;
                case3= true;
                goto mainLoop2;
            }

            if(theLibCheck.timeSinceBallWasSeen > 1500 || theBallModel.estimate.position.norm() > 2550)
                goto gotoGoaliePosition;

            if((std::abs(theRobotPose.translation.x() - theLibCheck.goaliePosition.x()) < 50) && (std::abs(theRobotPose.translation.y() - theLibCheck.goaliePosition.y()) < 50)){
                case1 = false;
                case2 = true;
                goto turnToBall; // TODO TEST: commented in previous version, see if it causes trouble
            }
        }
        action
        {
            theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
            if (between(theRobotPose.rotation, Angle::fromDegrees(-60.f), Angle::fromDegrees(60.f) ))
                theWalkToTargetSkill(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(),
                                                          (theLibCheck.glob2Rel(theLibCheck.goaliePosition.x(), theLibCheck.goaliePosition.y())).translation.x(),
                                                          (theLibCheck.glob2Rel(theLibCheck.goaliePosition.x(), theLibCheck.goaliePosition.y())).translation.y()));
            else 
                theWalkToTargetSkill(Pose2f(50.f, 50.f, 50.f), Pose2f(0,
                                                          (theLibCheck.glob2Rel(theLibCheck.goaliePosition.x(), theLibCheck.goaliePosition.y())).translation.x(),
                                                          (theLibCheck.glob2Rel(theLibCheck.goaliePosition.x(), theLibCheck.goaliePosition.y())).translation.y()));                
        }
    }

    state(turnToBall)
    {
        transition
        {
            #ifdef SPQR_DEBUG_GOALIE
            std::cerr << "turnToBall" << std::endl;
            #endif

            if(theLibCheck.timeSinceBallWasSeen > 1500 || theBallModel.estimate.position.norm() > 2500) {
                case1 = true;
                case2 = false;
                goto gotoGoaliePosition;
            }
            
            if(std::abs(theBallModel.estimate.position.angle()) < 5_deg){
                case2 = false;
                case3= true;
                goto mainLoop2;
            }
        }
        action
        {
            theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
            theWalkToTargetSkill(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
        }
    }

    state(mainLoop2)
    {
        transition
        {
            #ifdef SPQR_DEBUG_GOALIE
            std::cerr << "mainLoop2" << std::endl;
            #endif          
            Vector2f velocity = theBallModel.estimate.velocity;
            Vector2f position = theBallModel.estimate.position;

            if( theLibCheck.norm(theRobotPose.translation.x() - theLibCheck.goaliePosition.x(),
                                    theRobotPose.translation.y() - theLibCheck.goaliePosition.y()) > 150 ) {
                case1 = true;
                case3 = false;
                goto goalieOutOfPoles;
            }

            if(theLibCheck.timeSinceBallWasSeen > 1800 || theBallModel.estimate.position.norm() > 2550){
                case1 = true;
                case3 = false;
                goto gotoGoaliePosition;
            }

            if(std::abs(theBallModel.estimate.position.angle()) > 5_deg){
                case2 = true;
                case3 = false;
                goto turnToBall;
            }
        }
        action
        {
            if(theLibCheck.timeSinceBallWasSeen < 2000){
                theStandSkill();
                theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
            }
            else if(theLibCheck.timeSinceBallWasSeen < 5000){
                theStandSkill();
                //theLookAtGlobalBallSkill()
                Pose2f globalBall = theLibCheck.glob2Rel(theTeamBallModel.position.x(),theTeamBallModel.position.y());
                theLookAtPointSkill(Vector3f(globalBall.translation.x(), globalBall.translation.y(), 0));
            }
            else{
                theStandSkill();
                theLookLeftAndRightSkill();
            }
        }
    }
  }
};

MAKE_CARD(Goalie2020Card);
