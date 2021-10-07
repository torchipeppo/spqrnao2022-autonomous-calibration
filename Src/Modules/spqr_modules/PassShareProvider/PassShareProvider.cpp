#include "PassShareProvider.h"

#include <unistd.h>
#include <iostream>
#include "Representations/BehaviorControl/Libraries/LibCheck.h"


PassShareProvider::PassShareProvider(){
    SPQR::ConfigurationParameters();
}

/*
(float) sharedGoalUtil,
  (int) myNumber,
  (int) passingTo,
  (int) readyReceive,
  (int) readyPass,

*/

void PassShareProvider::update(PassShare& ps) {
  ps.GRAPHICAL_DEBUG = GRAPHICAL_DEBUG;
  ps.MAX_ALLOWED_OPPONENT_DISTANCE_FROM_PASS_TARGET = MAX_ALLOWED_OPPONENT_DISTANCE_FROM_PASS_TARGET;

  Pose2f passTarget = theLibCheck.poseToPass();
  //poseToPass returns a pose anyway, being the center of the enemy goal if no passing line is available
  ps.passTarget = passTarget;
  ps.myNumber = theRobotInfo.number;
  ps.tooManyOpponentsNearTarget = theLibCheck.areThereOpponentsNearby(passTarget.translation, MAX_ALLOWED_OPPONENT_DISTANCE_FROM_PASS_TARGET);
  
  //This decides if we're ready to pass
  if((passTarget.translation.x() != theFieldDimensions.xPosOpponentGroundline && passTarget.translation.y() != 0)
     && !ps.tooManyOpponentsNearTarget) {
    ps.passingTo = theLibCheck.isTargetToPass;
    ps.readyReceive = 1;
    ps.readyPass = 1;
  }
  else
  {
    ps.passingTo = -1;
    ps.readyReceive = 1;
    ps.readyPass = 0;
  }
  
  //updating role
  ps.role=int(theRoleAndContext.myRole);
  // std::cout<<"message role "<<ps.role<<std::endl;
  // ps.sharedGoalUtil = thePossiblePlan.receiveUtil;

  // ps.myNumber = theRobotInfo.number;

  // if(theRole.role == Role::RoleType::goalie)
  //               ps.role = 1;
  //           else if(theRole.role== Role::RoleType::defender)
  //               ps.role = 2;
  //           else if(theRole.role== Role::RoleType::supporter)
  //               ps.role = 3;
  //           else if(theRole.role == Role::RoleType::jolly)
  //               ps.role = 4;
  //           else if(theRole.role == Role::RoleType::striker)
  //               ps.role = 5;
  //      else if(theRole.role == Role::RoleType::searcher_1)
  //               ps.role = 6;
  //           else if(theRole.role == Role::RoleType::searcher_2)
  //               ps.role = 7;
  //           else if(theRole.role == Role::RoleType::searcher_3)
  //               ps.role = 8;
  //           else if(theRole.role == Role::RoleType::searcher_4)
  //               ps.role = 9;
  // if(theRole.role == Role::RoleType::goalie)
  //               // ps.role = 1;
  //               std::cout<<"message role "<<1 <<std::endl;
  //           else if(theRole.role == Role::RoleType::striker)
  //               // ps.role = 2;
  //               std::cout<<"message role "<<2 <<std::endl;
  //           else if(theRole.role== Role::RoleType::defender)
  //               // ps.role = 3;
  //               std::cout<<"message role "<<3 <<std::endl;
  //           else if(theRole.role== Role::RoleType::supporter)
  //               // ps.role = 4;
  //               std::cout<<"message role "<<4 <<std::endl;
  //           else if(theRole.role == Role::RoleType::jolly)
  //               // ps.role = 5;
  //               std::cout<<"message role "<<5 <<std::endl;
  //      else if(theRole.role == Role::RoleType::searcher_1)
  //               // ps.role = 6;
  //               std::cout<<"message role "<<6 <<std::endl;
  //           else if(theRole.role == Role::RoleType::searcher_2)
  //               // ps.role = 7;
  //               std::cout<<"message role "<<7 <<std::endl;
  //           else if(theRole.role == Role::RoleType::searcher_3)
  //               // ps.role = 8;
  //               std::cout<<"message role "<<8 <<std::endl;
  //           else if(theRole.role == Role::RoleType::searcher_4)
  //               // ps.role = 9;
  //               std::cout<<"message role "<<9 <<std::endl;

  // //std::cout<<"Numero "<<ps.myNumber<<" PassUtil "<<ps.sharedGoalUtil<<std::endl;
  // int i;

  // for(i = 0; i < theTeamData.teammates.size(); i++ ){
  //  if(theTeamData.teammates.at(i).thePassShare.sharedGoalUtil > (ps.sharedGoalUtil *1.1)&& theSPQRInfoDWK.IHaveTheBall == 1){
  //    ps.passingTo = theTeamData.teammates.at(i).thePassShare.myNumber;
  //    //std::cout<<"myUtil ="<<ps.sharedGoalUtil<<std::endl;
  //    //std::cout<<"opponent util = "<<theTeamData.teammates.at(i).thePassShare.sharedGoalUtil<<std::endl;

  //  }else{
  //    ps.passingTo = -1;
  //  }
  // }

  // ps.readyPass = 0;
  // if(ps.passingTo != -1){

  //  for(i = 0; i < theTeamData.teammates.size(); i++){
  //    if(theTeamData.teammates.at(i).thePassShare.myNumber == ps.passingTo){
  //      if(theTeamData.teammates.at(i).thePassShare.readyReceive == 1){
  //        ps.readyPass = 1;
  //      }
  //    }
  //  }

  // }

  // ps.readyReceive = 0;
  // for(i = 0; i < theTeamData.teammates.size(); i++){
  //  if(theTeamData.teammates.at(i).thePassShare.passingTo == ps.myNumber){

  //    Pose2f teammatePose = theTeamData.teammates.at(i).theRobotPose;
  //    if(std::abs(theLibCodeRelease.angleToTarget(teammatePose.translation.x(),
  //      teammatePose.translation.y())) < Angle::fromDegrees(5.f) ){
  //      ps.readyReceive = 1;
  //    }

  //  }
  // }

}

MAKE_MODULE(PassShareProvider, modeling)
