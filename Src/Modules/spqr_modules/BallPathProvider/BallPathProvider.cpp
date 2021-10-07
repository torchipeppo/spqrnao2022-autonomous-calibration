/**
 * @file zmpProvider.h
 *
 * This file computes zContrast skill vector specification and feedback calculation 
 *
 * @author Akshay Dhonthi
 */

#include "BallPathProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "iostream"

float val = 0.0;
Pose2f oppPos;

Pose2f getPointOnLine(Pose2f targetPoint1, Pose2f targetPoint2, Pose2f feedbackPoint2) {
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

    return pointOnLine;
}

float getErrorTheta(Pose2f targetPoint1, Pose2f targetPoint2, Pose2f feedbackPoint1, Pose2f feedbackPoint2) {
    float m1 = (targetPoint2.translation.y() - targetPoint1.translation.y()) / (targetPoint2.translation.x() - targetPoint1.translation.x());
    float m2 = (feedbackPoint1.translation.y() - feedbackPoint2.translation.y()) / (feedbackPoint1.translation.x()-feedbackPoint2.translation.x());

    return -(atan2( m2-m1, (1+(m1*m2))) * 180)/pi;
}

void BallPathProvider::update(BallPath& ballPath)
{
        // --------------------Get Ball velocity vector------------------------  
        float x = (theTeamBallModel.velocity.x() + theTeamBallModel.position.x());
        float y = (theTeamBallModel.velocity.y() + theTeamBallModel.position.y());

        // --------------------Get Foot Vector------------------------  


        // -------------------------------------------------------------------------------------
        // --------------------Define Four Parameters for the controller------------------------  
        // -------------------------------------------------------------------------------------        
        // Pose2f targetPoint1(theTeamBallModel.position.x(), theTeamBallModel.position.y());
        // Pose2f targetPoint2(-4000, 0);
        std::string cur_state; 
        for(const auto& node : theActivationGraph.graph){
            if (node.option == "BallControllerCard"){
                cur_state = node.state;
            }
        }
        // std::cout << cur_state << std::endl;

        if (cur_state == "coltroller_ID1"){            
            float xOffset = 140.0, yOffset = -50.0;

            Pose2f footL = theLibCheck.rel2Glob(theRobotModel.soleLeft.translation.x(), theRobotModel.soleLeft.translation.y());
            Pose2f footOffL = theLibCheck.rel2Glob( theRobotModel.soleLeft.translation.x()+xOffset, theRobotModel.soleLeft.translation.y()+yOffset );

            Pose2f footR = theLibCheck.rel2Glob(theRobotModel.soleRight.translation.x(), theRobotModel.soleRight.translation.y());
            Pose2f footOffR = theLibCheck.rel2Glob(theRobotModel.soleRight.translation.x()+xOffset, theRobotModel.soleRight.translation.y()+yOffset );

            if (theRobotPose.translation.y() < 0.0) {
                targetPoint1.translation =  theTeamBallModel.position;
                targetPoint2.translation =  Vector2f(-4000.0, 0.0);

                feedbackPoint1 = footL;
                feedbackPoint2 = footOffL;
            } else {
                targetPoint1.translation =  theTeamBallModel.position;
                targetPoint2.translation =  Vector2f(-4000.0, 0.0);

                feedbackPoint1 = footR;
                feedbackPoint2 = footOffR;
            }
        } else if(cur_state == "kickBall_ID2"){
            float xOffset = 80, yOffset = 0.0;

            Pose2f footL = theLibCheck.rel2Glob(theRobotModel.soleLeft.translation.x(), theRobotModel.soleLeft.translation.y());
            Pose2f footOffL = theLibCheck.rel2Glob( theRobotModel.soleLeft.translation.x()+xOffset, theRobotModel.soleLeft.translation.y()+yOffset );

            Pose2f footR = theLibCheck.rel2Glob(theRobotModel.soleRight.translation.x(), theRobotModel.soleRight.translation.y());
            Pose2f footOffR = theLibCheck.rel2Glob(theRobotModel.soleRight.translation.x()+xOffset, theRobotModel.soleRight.translation.y()+yOffset );

            if (theRobotPose.translation.y() < 0.0) {
                targetPoint1.translation =  Vector2f(4000.0, 1000.0);
                targetPoint2.translation = theTeamBallModel.position;
                
                feedbackPoint1 = footL;
                feedbackPoint2 = footOffL;
            } else {
                targetPoint1.translation =  Vector2f(4000.0, -1000.0);
                targetPoint2.translation = theTeamBallModel.position;

                feedbackPoint1 = footR;
                feedbackPoint2 = footOffR;
            }
        }

        // --------------------Get Desired angle------------------------  
        float targetTheta = toDegrees(atan2(targetPoint2.translation.y()-targetPoint1.translation.y(), targetPoint2.translation.x()-targetPoint1.translation.x()));

        // --------------------Get Current orientation of the Robot------------------------  
        float currentTheta = toDegrees(atan2(feedbackPoint1.translation.y()-feedbackPoint2.translation.y(), feedbackPoint1.translation.x()-feedbackPoint2.translation.x()));
        
        // --------------------Calculate error in theta------------------------  
        ballPath.errorTheta = getErrorTheta(targetPoint1, targetPoint2, feedbackPoint1, feedbackPoint2);
        // std::cout << targetTheta << ", " << currentTheta << ", " << ballPath.errorTheta << std::endl;

        // --------------------Get Contact Distance------------------------  
        // Equation of the line in the form of Ax + By + C = 0
        
        float A = -(targetPoint2.translation.y() - targetPoint1.translation.y()) / (targetPoint2.translation.x() - targetPoint1.translation.x());
        float B = 1;
        float C = -(targetPoint1.translation.y() + A*targetPoint1.translation.x());
        // Estimated Point of contact
        float x1 = feedbackPoint2.translation.x();
        float y1 = feedbackPoint2.translation.y();
        // Get Closest point on ball velocity vector
        Pose2f pointOnLine = getPointOnLine(targetPoint1, targetPoint2, feedbackPoint2);
        
        if (theTeamBallModel.position.x() > pointOnLine.translation.x()) {
            ballPath.errorDistX = sqrt(pow((pointOnLine-theTeamBallModel.position).translation.x(), 2)+pow((pointOnLine-theTeamBallModel.position).translation.y(), 2)) - 80.0;
        }else {
            ballPath.errorDistX = -sqrt(pow((pointOnLine-theTeamBallModel.position).translation.x(), 2)+pow((pointOnLine-theTeamBallModel.position).translation.y(), 2)) - 80.0;
        }
        // Calculate perpendicular distance from the point to the line d = |A*x_1 + B*y_1 + C| / (A^2 + B^2)^(1/2)
        ballPath.errorDistY = (A*x1 + B*y1 + C) / sqrt(pow(A,2) + pow(B,2));
        // std::cout << ballPath.errorTheta << ", " << ballPath.errorDistX  << ", " <<  ballPath.errorDistY << std::endl;
        


    if(theOwnTeamInfo.teamNumber == 1 && theRobotInfo.number == 3){
        DEBUG_DRAWING3D("module:BallPathProvider", "field")
        {   
            int height = 40;
            // Draw Desired Vector
            LINE3D("module:BallPathProvider", targetPoint1.translation.x(), targetPoint1.translation.y(), height, targetPoint2.translation.x(), targetPoint2.translation.y(), height, 2, ColorRGBA::red);
            SPHERE3D("module:BallPathProvider", targetPoint1.translation.x(), targetPoint1.translation.y(), height, 15, ColorRGBA(200, 0, 0));
            SPHERE3D("module:BallPathProvider", targetPoint2.translation.x(), targetPoint2.translation.y(), height, 15, ColorRGBA(0, 200, 0));
            
            // Draw Feedback Vector 
            LINE3D("module:BallPathProvider", feedbackPoint1.translation.x(), feedbackPoint1.translation.y(), height, feedbackPoint2.translation.x(), feedbackPoint2.translation.y(), height, 2, ColorRGBA::blue);
            SPHERE3D("module:BallPathProvider", feedbackPoint1.translation.x(), feedbackPoint1.translation.y(), height, 15, ColorRGBA(0, 0, 200));
            SPHERE3D("module:BallPathProvider", feedbackPoint2.translation.x(), feedbackPoint2.translation.y(), height, 15, ColorRGBA(0, 200, 0));
            // Draw Ball Position
            SPHERE3D("module:BallPathProvider", theTeamBallModel.position.x(), theTeamBallModel.position.y(), height, 15, ColorRGBA(0, 200, 0));
            // Draw Ball Velocity Vector
            LINE3D("module:BallPathProvider", theTeamBallModel.position.x(), theTeamBallModel.position.y(), height, x, y, height, 2, ColorRGBA::red);
            // Draw Positional error Point
            SPHERE3D("module:BallPathProvider", pointOnLine.translation.x(), pointOnLine.translation.y(), height, 15, ColorRGBA(200, 0, 0));
        }
    }
}

MAKE_MODULE(BallPathProvider, motionControl)

            
            