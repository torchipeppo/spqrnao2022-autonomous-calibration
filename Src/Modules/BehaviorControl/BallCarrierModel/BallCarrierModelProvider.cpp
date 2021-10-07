/**
 * @file Modules/BehaviorControl/BallCarrierModel/BallCarrierModelProvider.cpp
 *
 * This module contains all info necessary for the Ball Carrier model
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "BallCarrierModelProvider.h"

BallCarrierModelProvider::BallCarrierModelProvider()
{}

void BallCarrierModelProvider::update(BallCarrierModel& ballCarrierModel)
{
    ballCarrierModel.computeApproachPoint = [&] (float radius, float angle)
    {   
        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation; 
        float dynamicOffsetX = radius * cos(pi + angle);
        float dynamicOffsetY = radius * sin(pi + angle);
        return Pose2f(-angle, ballPositionGlobal.x() + dynamicOffsetX, ballPositionGlobal.y() - dynamicOffsetY);
    };
    
    ballCarrierModel.dynamicApproachPoint = [&] () -> Pose2f
    {
        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation; 
        float ballToTargetAngle = theLibCheck.angleBetweenPoints(ballPositionGlobal, ballCarrierModel.dynamicTarget.translation);
        //Offsets depending on the distance of the robot from the ball
        return ballCarrierModel.computeApproachPoint(ballCarrierModel.dynamicApproachRadius, ballToTargetAngle);
    };

    ballCarrierModel.staticApproachPoint = [&] () -> Pose2f
    {
        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        float ballToTargetAngle = theLibCheck.angleBetweenPoints(ballPositionGlobal, ballCarrierModel.dynamicTarget.translation);
        return ballCarrierModel.computeApproachPoint(ballCarrierModel.staticApproachRadius, ballToTargetAngle);
    };

    ballCarrierModel.escapeApproachPoint = [&] () -> Pose2f
    {
        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        float ballToTargetAngle = theLibCheck.angleBetweenPoints(ballPositionGlobal, ballCarrierModel.nearestObstacleEscapeTarget);
        return ballCarrierModel.computeApproachPoint(ballCarrierModel.staticApproachRadius, ballToTargetAngle);
    };

    //NOTICE: requires GLOBAL obstacle coordinates
    ballCarrierModel.getTangentDirectionToObstacle = [&] (Vector2f obstacle, bool clockwise = false) -> Vector2f
    {
        Vector2f diff = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation - obstacle;
        Vector2f tangent;
        if(clockwise) {
            tangent = Vector2f(-diff.y(), diff.x());
        }
        else {
            tangent = Vector2f(diff.y(), -diff.x());
        }
        tangent.normalize();
        return tangent;
    };

    //NOTICE: requires GLOBAL obstacle coordinates
    ballCarrierModel.getObstacleEscapeTarget = [&] (Vector2f obstacle, float clearanceRadius) -> Vector2f
    {
        //1) Calcolo   angolo tra congiungente palla-dynamic target e verticale (angolo 0°)
        //2) Calcolo   angolo tra congiungente ostacolo-dynamic target e verticale
        //3) Calcolo differenza tra 2) e 1)
        //4) Se differenza positiva -> clockwise = true, else -> false
        //5) Vedere se angleBetweenTwoPoints è corretto 
        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        float ballToTargetAngle = theLibCheck.angleBetweenPoints(globalBall, ballCarrierModel.dynamicTarget.translation);
        float ballToObstacleAngle = theLibCheck.angleBetweenPoints(globalBall, obstacle);
      
        bool clockwise = (bool) (ballToObstacleAngle - ballToTargetAngle < 0);

        Vector2f tangent = ballCarrierModel.getTangentDirectionToObstacle(obstacle, clockwise);
        tangent.x() = tangent.x() * clearanceRadius;
        tangent.y() = tangent.y() * clearanceRadius;
        
        return tangent + globalBall;
    };
    
    ballCarrierModel.nearestObstacleToBall = [&] () -> Vector2f
    {
        Vector2f nearestObstacle;
        float nearestObstacleToBallDistance = -1;
        for(auto obs : theObstacleModel.obstacles)
        {
            float currentObsDistance = theLibCheck.distance(theBallModel.estimate.position, obs.center);
            if(nearestObstacleToBallDistance == -1 || theLibCheck.distance(theBallModel.estimate.position, obs.center) < nearestObstacleToBallDistance)
            {
                nearestObstacle = Vector2f(obs.center.x(), obs.center.y());
                nearestObstacleToBallDistance = currentObsDistance;     
            }
        }
        return nearestObstacle;
    };

    ballCarrierModel.isBallInDangerZone = [&] () -> bool
    {
        Vector2f nearestObstacle = ballCarrierModel.nearestObstacleToBall();
        float nearestObstacleToBallDistance = theLibCheck.distance(theBallModel.estimate.position, nearestObstacle);
        return nearestObstacleToBallDistance<ballCarrierModel.nearestObstacleEscapeRadius;
    };

    ballCarrierModel.isRobotInDangerZone = [&] () -> bool
    {
        Vector2f nearestObstacle = ballCarrierModel.nearestObstacleToBall();
        return nearestObstacle.norm() < ballCarrierModel.nearestObstacleEscapeRadius;
    };

//IDEA usare invece il left and right di un ostacolo: dato che gli ostacoli sono in coordinate locali come il BallModel,
//si può verificare se la palla è "dietro un ostacolo" confrontando la sua posizione con left e right
//ALTRA IDEA invece di verificare se la palla è nella danger zone, verificare se lo staticApproachPoint è nella danger zone

    ballCarrierModel.useEscapeTarget = [&] (Vector2f target) -> bool
    {   
        Vector2f nearestObstacle = ballCarrierModel.nearestObstacleToBall();
        float nearestObstacleToBallDistance = theLibCheck.distance(theBallModel.estimate.position, nearestObstacle);

        //1) The ball is near the obstacle
        return ballCarrierModel.isBallInDangerZone()
                    &&
                    //NOTICE: nearestObstacle is in LOCAL coordinates
                    ballCarrierModel.isRobotInDangerZone() //2) We are near the obstacle
                        &&
                        //3) The ball is nearer to the goal than the obstacle
                        theLibCheck.distance(theLibCheck.rel2Glob(nearestObstacle.x(), nearestObstacle.y()), target) > theLibCheck.distance(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), target)
                            &&
                            //4) The obstacle is nearer to the goal than the robot is (so the obstacle is dangerously near the robot and the ball and between the robot and the ball)            
                            theLibCheck.distance(theLibCheck.rel2Glob(nearestObstacle.x(), nearestObstacle.y()), target) < theLibCheck.distance(theRobotPose, target);
    };

    ballCarrierModel.graphicalDebug = GRAPHICAL_DEBUG;

    ballCarrierModel.ESCAPE_OBSTACLES = ESCAPE_OBSTACLES;
    ballCarrierModel.USE_TRAJECTORY = USE_TRAJECTORY;
    ballCarrierModel.OFFSET_FROM_Y_CENTER_TO_CHANGE_TRAJECTORY = OFFSET_FROM_Y_CENTER_TO_CHANGE_TRAJECTORY;
    
    Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

    if(ONLY_LEFT_PATH)
    {
        if(globalBall.y() < -OFFSET_FROM_Y_CENTER_TO_CHANGE_TRAJECTORY && !ballCarrierModel.usingRightTrajectory)
        {
            ballCarrierModel.usingRightTrajectory = true;
        }
        else if(globalBall.y() > OFFSET_FROM_Y_CENTER_TO_CHANGE_TRAJECTORY && ballCarrierModel.usingRightTrajectory)
        {
            ballCarrierModel.usingRightTrajectory = false;
        }
    }

    ballCarrierModel.TRAJECTORY.clear();
    for(auto trajNode : TRAJECTORY)
    {
        if(ballCarrierModel.usingRightTrajectory)
        {
            trajNode.destination.y() = -trajNode.destination.y();
            ballCarrierModel.TRAJECTORY.push_back(trajNode);
        }
        else
        {
            ballCarrierModel.TRAJECTORY.push_back(trajNode);
        }        
    }
    
    ballCarrierModel.maximumApproachDistance = MAXIMUM_APPROACH_DISTANCE;
    ballCarrierModel.minimumApproachDistance = MINIMUM_APPROACH_DISTANCE;
    ballCarrierModel.nearestObstacleEscapeRadius = OBSTACLE_DANGER_ZONE_RADIUS;
    
    ballCarrierModel.useLongKicksToCarry = USE_LONG_KICKS_TO_CARRY;

    ballCarrierModel.firstTargetWithOneObstacleLeft = Vector2f(FIRST_TARGET_POS_X, FIRST_TARGET_WITH_ONE_OBSTACLE_POS_Y);
    ballCarrierModel.firstTargetWithOneObstacleRight = Vector2f(FIRST_TARGET_POS_X, -FIRST_TARGET_WITH_ONE_OBSTACLE_POS_Y);
    ballCarrierModel.firstTargetWithTwoObstacles = Vector2f(FIRST_TARGET_POS_X, FIRST_TARGET_WITH_TWO_OBSTACLES_POS_Y);

    ballCarrierModel.staticApproachRadius = STATIC_APPROACH_RADIUS;

    ballCarrierModel.xRangeDistanceFromGoalToUseKicks = X_RANGE_DISTANCE_FROM_GOAL_TO_USE_KICKS;
    ballCarrierModel.yRangeDistanceFromGoalToUseKicksInner = Y_RANGE_DISTANCE_FROM_GOAL_TO_USE_KICKS_INNER;
    ballCarrierModel.yRangeDistanceFromGoalToUseKicksOuter = Y_RANGE_DISTANCE_FROM_GOAL_TO_USE_KICKS_OUTER;
    
    ballCarrierModel.obstacleAvoidanceArcAngleStep = pi2/OBSTACLE_AVOIDANCE_ARC_ANGLE_STEP_FACTOR;
    ballCarrierModel.obstacleAvoidancePlan.clear();
    ballCarrierModel.ballPath.clear();


    //If the robot is behind the line after which it is allowed to kick to goal, use the heuristic-based target
    // else use the immediate one
    bool useHeuristic = true;
    if(!ALWAYS_USE_GOAL_TARGET_HEURISTIC && globalBall.x() > theFieldDimensions.xPosOpponentGroundline - ballCarrierModel.xRangeDistanceFromGoalToUseKicks.max)
    {
        useHeuristic = false;
    }
    Vector2f goalTarget = theLibCheck.goalTarget(false, useHeuristic);
    
    Pose2f target;
    if(USE_TRAJECTORY && ballCarrierModel.TRAJECTORY.size()>0)
    {
        int currentTrajNode = 0;
        for(auto trajNode : ballCarrierModel.TRAJECTORY)
        {
            if(theRobotPose.translation.x() < trajNode.reachedAtXCoordinate)
            {
                target = trajNode.destination;
                break;
            }
            currentTrajNode++;   
        }
        if(currentTrajNode == ballCarrierModel.TRAJECTORY.size())
        {
            ballCarrierModel.currentTrajectoryNode = -1;
            //Right now we're just pointing to the goal target without a specific angle
            target = Pose2f(0.0, goalTarget.x(), goalTarget.y());
        }
        else
        {
            ballCarrierModel.currentTrajectoryNode = currentTrajNode;
        }
    }
    else
    {
        //Right now we're just pointing to the goal target without a specific angle
        target = Pose2f(0.0, goalTarget.x(), goalTarget.y());
    }
    //ballCarrierModel.dynamicTarget = Pose2f();
    //Set the goalTarget as a fallback target (if no path is found, a fake path between the ball and the goalTarget will be used)
    ballCarrierModel.dynamicTarget = target;
    
    ballCarrierModel.isFallbackPath = true;
    ballCarrierModel.isTargetOnGoal = true;
    ballCarrierModel.isTargetAPass = false;

    //Find nearest obstacle to ball and its distance
    Vector2f nearestObstacle = ballCarrierModel.nearestObstacleToBall();
    float nearestObstacleToBallDistance = theLibCheck.distance(theBallModel.estimate.position, nearestObstacle);

    //Default obstacle radius is 550 mm
    ballCarrierModel.dynamicGoalPostObstacleRadius = theLibPathPlanner.getDefaultObstacleRadius(Obstacle::Type::goalpost);
    //ballCarrierModel.dynamicUprightRobotObstacleRadius = theLibPathPlanner.getDefaultObstacleRadius(Obstacle::Type::someRobot);
    
    ballCarrierModel.dynamicUprightRobotObstacleRadius = (nearestObstacleToBallDistance == -1 ? BASE_UPRIGHT_ROBOT_OBSTACLE_RADIUS : std::max(MINIMUM_UPRIGHT_ROBOT_OBSTACLE_RADIUS, std::min(BASE_UPRIGHT_ROBOT_OBSTACLE_RADIUS, nearestObstacleToBallDistance)));
    
    //When nearing the goal, decrease the obstacle radius to the very minimum
    if(globalBall.x() > theFieldDimensions.xPosOpponentGroundline - ballCarrierModel.xRangeDistanceFromGoalToUseKicks.max)
    {
        ballCarrierModel.dynamicUprightRobotObstacleRadius = BASE_UPRIGHT_ROBOT_OBSTACLE_RADIUS - theLibCheck.mapToInterval(globalBall.x(), theFieldDimensions.xPosOpponentPenaltyArea - ballCarrierModel.xRangeDistanceFromGoalToUseKicks.max, theFieldDimensions.xPosOpponentPenaltyArea, 0, BASE_UPRIGHT_ROBOT_OBSTACLE_RADIUS - MINIMUM_UPRIGHT_ROBOT_OBSTACLE_RADIUS_IN_KICKING_AREA);
    }

    if(globalBall.x() < OBSTACLE_RADIUS_MULTIPLIER_CHANGE_X_COORDINATE)
    {
        ballCarrierModel.dynamicUprightRobotObstacleRadius *= OBSTACLE_RADIUS_MULTIPLIER.min;
    }
    else
    {
        ballCarrierModel.dynamicUprightRobotObstacleRadius *= OBSTACLE_RADIUS_MULTIPLIER.max;
    }

    ballCarrierModel.dynamicReadyRobotObstacleRadius = theLibPathPlanner.getDefaultObstacleRadius(Obstacle::Type::someRobot);
    ballCarrierModel.dynamicFallenRobotObstacleRadius = theLibPathPlanner.getDefaultObstacleRadius(Obstacle::Type::fallenOpponent);
    ballCarrierModel.dynamicRadiusControlOffset = 100;

    //Compute the perpendicular direction to the vector connecting the nearest obstacle to the ball
    //This direction can be used to 
    ballCarrierModel.nearestObstacleEscapeTarget = ballCarrierModel.getObstacleEscapeTarget(theLibCheck.rel2Glob(nearestObstacle.x(), nearestObstacle.y()).translation, BASE_UPRIGHT_ROBOT_OBSTACLE_RADIUS);

    if(theGameInfo.state == STATE_PLAYING)
    {
        Pose2f speed = Pose2f(0.8f,0.8f,0.8f);
        bool avoidPenaltyArea = false;
        std::vector<PathPlannerUtils::Node> plan = theLibPathPlanner.populatePlanWithCustomObstacleRadius(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), target, speed, avoidPenaltyArea,
                                                                                                            ballCarrierModel.dynamicGoalPostObstacleRadius,
                                                                                                            ballCarrierModel.dynamicUprightRobotObstacleRadius,
                                                                                                            ballCarrierModel.dynamicReadyRobotObstacleRadius,
                                                                                                            ballCarrierModel.dynamicFallenRobotObstacleRadius,
                                                                                                            ballCarrierModel.dynamicRadiusControlOffset);
        
        std::vector<PathPlannerUtils::Edge*> obstacleAvoidancePlan = theLibPathPlanner.createAvoidancePlan(plan);
        std::vector<Vector2f> ballPath = theLibPathPlanner.computePath(plan, ballCarrierModel.obstacleAvoidanceArcAngleStep);
        
        //NOTICE: Use goalTarget as a fallback target
        ballCarrierModel.dynamicTarget = target; 
        
        //Now we create new Node and Edge instances using the BallCarrierModel::Node and ::Edge definitions that are now STREAMABLE
        if(obstacleAvoidancePlan.size() > 0)
        {
            PathPlannerUtils::Edge* edge = obstacleAvoidancePlan[0];
            BallCarrierModel::Node fromNode(edge->fromNode->center, edge->fromNode->radius);
            BallCarrierModel::Node toNode;
            for(int i = 0; i<obstacleAvoidancePlan.size(); i++)
            {
                edge = obstacleAvoidancePlan[i];
                toNode = BallCarrierModel::Node(edge->toNode->center, edge->toNode->radius);
                ballCarrierModel.obstacleAvoidancePlan.push_back(BallCarrierModel::Edge(fromNode, toNode));
                fromNode = toNode;
            }
        }

        if(ballPath.size() > 0)
        {
            BallCarrierModel::Edge edge;
            BallCarrierModel::Node fromNode = BallCarrierModel::Node(ballPath[0], 0.f);
            ballCarrierModel.ballPath.push_back(fromNode);
            for(int i=1; i<ballPath.size(); i++)
            {
                Vector2f position = ballPath[i];
                BallCarrierModel::Node toNode(position, 0.f);
                edge = BallCarrierModel::Edge(fromNode, toNode);
                ballCarrierModel.ballPath.push_back(toNode);
            }
            ballCarrierModel.isFallbackPath = false;
        }
        else
        {
            ballCarrierModel.ballPath.push_back(BallCarrierModel::Node(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation, 0.f));
            ballCarrierModel.ballPath.push_back(BallCarrierModel::Node(target.translation, 0.f));
            ballCarrierModel.isFallbackPath = true;
        }
                                    
        //When the ball is too near an obstacle and the obstacle is between the robot and the ball,
        //use the tanget escape target instead of the ball path
        if(ballCarrierModel.useEscapeTarget(goalTarget))
        {
            //Set the escape target as the dynamic target
            ballCarrierModel.isTargetOnGoal = false;
            ballCarrierModel.isFallbackPath = false;
            
            float angle = theLibCheck.angleToTarget(ballCarrierModel.nearestObstacleEscapeTarget.x(), ballCarrierModel.nearestObstacleEscapeTarget.y());
            ballCarrierModel.dynamicTarget = Pose2f(angle, ballCarrierModel.nearestObstacleEscapeTarget.x(), ballCarrierModel.nearestObstacleEscapeTarget.y());
        }
        else
        {
            //Select the dynamic target (the next target the ball should reach along the path) (the default one is the fallback target that is the current goalTarget)
            if(ballPath.size()==2)
            {
                float angle = theLibCheck.angleToTarget(ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());
                ballCarrierModel.dynamicTarget = Pose2f(angle, ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());

                //If there are only two steps in the path (the ball position and the goalTarget), the target is on the goal line
                ballCarrierModel.isTargetOnGoal = true;
            }
            else
            {
                //Set the dynamicTarget
                float angle = theLibCheck.angleToTarget(ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());
                ballCarrierModel.dynamicTarget = Pose2f(angle, ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());

                //If there are more than two steps in the path, the target is certainly not on the goal line
                ballCarrierModel.isTargetOnGoal = false;
            }
        }
        
        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        
        if(theLibCheck.distance(theRobotPose, ballPositionGlobal) > ballCarrierModel.staticApproachRadius)
        {
            ballCarrierModel.dynamicApproachRadius = ballCarrierModel.staticApproachRadius;
        }
        else if(theLibCheck.distance(theRobotPose, ballPositionGlobal) < ballCarrierModel.minimumApproachDistance)
        {
            ballCarrierModel.dynamicApproachRadius = ballCarrierModel.minimumApproachDistance;
        }
        else if(theLibCheck.distance(theRobotPose, ballPositionGlobal) > ballCarrierModel.minimumApproachDistance 
        && theLibCheck.distance(theRobotPose, ballPositionGlobal) < ballCarrierModel.staticApproachRadius) 
        {
            ballCarrierModel.dynamicApproachRadius = theLibCheck.distance(theRobotPose, ballPositionGlobal);
        }
    }
}

MAKE_MODULE(BallCarrierModelProvider, behaviorControl)
