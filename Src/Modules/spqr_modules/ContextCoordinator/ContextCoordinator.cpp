/**
* @file ContextCoordinator.cpp
*   This file implements the team coordination module
* @author Francesco Riccio, Emanuele Borzi
*/

#include "ContextCoordinator.h"

#include <unistd.h>
#include <iostream>
#include "Representations/SPQR-Libraries/ConfigFile/ConfigFile.h"


//#include <fstream> // ofstream


#define NORM(x, y) sqrt(x*x + y*y)
//#define UM_VIEW

MAKE_MODULE(ContextCoordinator, behaviorControl)

ContextCoordinator::ContextCoordinator():
    goalie_pose(Pose2f()), defender_pose(Pose2f()), supporter_pose(Pose2f()),
    jolly_pose(Pose2f()), striker_pose(Pose2f()), striker_nokickoff_pose(Pose2f()),
    tmp_role(Role::undefined), myRole(Role::undefined), role_hysteresis_cycle(0),
    startHysteresis(true)
{
    SPQR::ConfigurationParameters();
    //update config file
    configure();
    for(unsigned i = 0; i< theRole.robots_poses.size(); i++) // 5 number of robots     modificato
    {
        std::vector<float> init_uv(theRole.robots_poses.size(),0); // 5 number of roles
        utility_matrix.push_back(init_uv);

        mapped_robots.push_back(false);
        penalized_robots.push_back(fall_down_penalty);
    }

    // role_time.setToNow();
    // stamp.setToNow();
    // role_time.getRealSystemTime();  // modificato
    // stamp.getRealSystemTime();  // modificato
    role_time = Time::getRealSystemTime();
    stamp = Time::getRealSystemTime();

}

void ContextCoordinator::configure()                    // configura la posizione inziale dei giocatori
{
    goalie_pose = Pose2f(goalie_pose_t, goalie_pose_x, goalie_pose_y);
    goalie_pose.translation.x() = goalie_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
    goalie_pose.translation.y() = goalie_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
    defender_pose = Pose2f(defender_pose_t, defender_pose_x, defender_pose_y);
    //defender_pose = Pose2f(defender_pose_t, defender_pose_x, theLibCodeRelease.defenderDynamicY());
    defender_pose.translation.x() = defender_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
    defender_pose.translation.y() = defender_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;

    int ourScore = theOwnTeamInfo.score;
    int theirScore = theOpponentTeamInfo.score;
    //int teamNumber = Global::getSettings().teamNumber;
    /*
    for(auto team : theGameCtrlData.teams){
        if( team.teamNumber == teamNumber ){
            ourScore = team.score;
        }else{
            theirScore = team.score;
        }

    }
    */
    if( ourScore <= theirScore ){
        supporter_pose = Pose2f(supporter_pose_t, supporter_pose_x, supporter_pose_y);
        supporter_pose.translation.x() = supporter_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
        supporter_pose.translation.y() = supporter_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
        jolly_pose = Pose2f(jolly_pose_t, jolly_pose_x, jolly_pose_y);
        jolly_pose.translation.x() = jolly_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
        jolly_pose.translation.y() = jolly_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
    }
    else{
        jolly_pose = Pose2f(supporter_pose_t, supporter_pose_x, supporter_pose_y);
        jolly_pose.translation.x() = supporter_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
        jolly_pose.translation.y() = supporter_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
        supporter_pose = Pose2f(jolly_pose_t, jolly_pose_x, jolly_pose_y);
        supporter_pose.translation.x() = jolly_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
        supporter_pose.translation.y() = jolly_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
    }



    striker_pose = Pose2f(striker_pose_t, striker_pose_x, striker_pose_y);
    striker_pose.translation.x() = striker_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
    striker_pose.translation.y() = striker_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
    striker_nokickoff_pose = Pose2f(striker_pose_t, striker_nokickoff_pose_x, striker_pose_y);
    striker_nokickoff_pose.translation.y() = striker_nokickoff_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
    striker_nokickoff_pose.translation.y() = striker_nokickoff_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
}

bool ContextCoordinator::isInCurrentContext(Role::RoleType currentRole)  // in base al numero del ruolo del giocatore torna true o false
{
    if (((int) currentRole) == 0 || ((int) currentRole) == 14)
        return false;
    //playing context
    if((theRole.current_context == Role::playing) && (((int) currentRole) < 6) && (((int) currentRole) > 0 ))
        return true;
    //search_for_ball context
    if(theRole.current_context == Role::search_for_ball &&

            ( ((int) currentRole >5 && (int) currentRole < 10)))
        return true;

    return false;
}

float ContextCoordinator::getUtilityOrientationValue(Pose2f target, Pose2f robot) const   // ritorna l'angolo con cui sono orientato
{                                                                                       // contributo da 0 a 125circa in utilitymatrix

    //Pose2f relativePosition = glob2Rel(x,y);

    Vector2f result;
    float theta = 0;
    float tempX = target.translation.x() - robot.translation.x();
    float tempY = target.translation.y() - robot.translation.y();

    result.x() = (float)(tempX * cos(robot.rotation) + tempY * sin(robot.rotation));
    result.y() = (float)(-tempX * sin(robot.rotation) + tempY * cos(robot.rotation));

    Pose2f a = Pose2f(theta , result.x(),result.y());


    float angle =  (atan2f(a.translation.y(), a.translation.x()));

    //float angle = theLibCodeRelease.angleToTarget(target.translation.x(), target.translation.y());
    //std::cout<< " targetAngle  "<< angle <<std::endl;

    /*if( NORM(vx,vy) < 500.f)
        return 200;
    else
        return (M_PI - Vector2f(vx,vy).angle())/M_PI;   //M_PI è pigreco in c++*/

    //if(theRobotInfo.number == 2 && target.translation.x() == 4500.f)

    return (float)((M_PI - std::abs(angle))/M_PI);
}

float ContextCoordinator::getUtilityRoleHistory(int j) const
{
    //from the time since this particular role has been mapped   --- per quanto tempo ho fatto quel ruolo
    uint time;
    // time = (int)(PTracking::Timestamp() - role_time).getSeconds();    ------modificato
    time = (Time::getTimeSince(role_time) ); //theFrameInfo.time tempo attuale in millisecondi
    time = time / 1000;  //cosi torna in secondi

//    if(tmp_role == j)
//    {
//        if(time*50 <= 500)
//            return time*50;
//        else return 500;
//    }

//    return 0;

        if(tmp_role == j)
        {
            if(time*50 <= 500)
            {
                //if(theRobotInfo.number == 2)
                    //std::cout<<"  primo return  = "<<time*25<<std::endl;
                return time/10;
            }
            else
            {
                //if(theRobotInfo.number == 2)
                  //  std::cout<<"  secondo return  = "<<250<<std::endl;
                return 1;
            }
        }

            //if(theRobotInfo.number == 2)
            //std::cout<<"  terzo return  = "<<0<<std::endl;
        return 0;

}

int ContextCoordinator::biasWeight(int role) const            //TODO NORMALIZE
{
    if(role == theRobotInfo.number && role != 5) return 500.f;
    else return 0.f;
}

float ContextCoordinator::getTemmateOntheBallValue(Pose2f target, Pose2f robotPos) const
{
    //if(theRobotInfo.number == 2){
    for(const auto& obs : theObstaclesFieldPercept.obstacles) {
        if(obs.type == ObstaclesFieldPercept::ownPlayer){
            if(theLibCheck.distance(obs.center, theLibCheck.glob2Rel(target.translation.x(), target.translation.y()))    <   theLibCheck.distance(robotPos, target)){
                //std::cout<<"distanza obs-tar  "<<theLibCodeRelease.distance(obs.center, theLibCodeRelease.glob2Rel(target.translation.x(), target.translation.y()))<<std::endl;
                //std::cout<<"distanza mia-tar  "<<theLibCodeRelease.distance(robotPos, target)<<std::endl;
                return 1.f;

            }
        }
    }
    //}
    return 0.f;
}

bool ContextCoordinator::isMaxRoleScore(int c)     // controlla la matrice di utilità e decide se il giocatore come parametro c è il migliore per quel ruolo
{                                                  // e controlla il caso in cui si ha gia qul ruolo
    //std::cout<<"ismaxRoleScore  "<< c <<std::endl;
    unsigned int max_r = 0;
    for(unsigned int r=0; r<utility_matrix.size(); ++r)
    {
        if(mapped_robots.at(r))
        {
        //    std::cout<<"r già mappato"<< r <<std::endl;
            continue;
        }
        if( utility_matrix.at(max_r).at(c-1) < utility_matrix.at(r).at(c-1) )
            max_r  = r;
    }

    if(max_r != 0)
    {
        //std::cout<<"mappo r  "<< max_r <<std::endl;
        mapped_robots.at(max_r) = true;
    }
    if( (int) max_r+1 == theRobotInfo.number)
    {
        //std::cout<<"ritorna true  "<< r <<std::endl;
        return true;
    }
    return false;
}


bool ContextCoordinator::isRobotPenalized(int r)             // controlla se il robot è in  stato penalized
{
    for(unsigned int t=0; t<theTeamData.teammates.size(); ++t)
    {
        if(theTeamData.teammates.at(t).number == r){
            return theTeamData.teammates.at(t).status != Teammate::PLAYING;
        }
    }

    return false;
}

Vector2f ContextCoordinator::rel2Glob(float x, float y) const
{
    Vector2f result;
    float rho = (float)sqrt((x * x) + (y * y));

    result.x() = (float)(theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x))));
    result.y() = (float)(theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x))));

    return Vector2f(result.x(),result.y());
}

void ContextCoordinator::computePlayingRoleAssignment()     // assegno il ruolo durante il playing
{
    for(unsigned int r=0; r<utility_matrix.size(); ++r){
        mapped_robots.at(r) = false;
    }

    if( isMaxRoleScore((int) Role::striker) )
        tmp_role = Role::striker;
    else if( isMaxRoleScore((int) Role::defender) /*&& theOwnTeamInfo.teamColor == TEAM_RED*/ ) //ptl
        tmp_role = Role::defender;
    else if( isMaxRoleScore((int) Role::jolly) )
        tmp_role = Role::jolly;
    else if( isMaxRoleScore((int) Role::supporter) )
        tmp_role = Role::supporter;
    else
        tmp_role = Role::undefined;

#ifdef DEEP_DEBUG_CONTEXT_COORD
    SPQR_INFO("tmp: "<<tmp_role); /// when no role has been selected the tmp role assigns the undefined enum.
    for(unsigned int r=0;r<mapped_robots.size(); ++r)
        SPQR_INFO("mapped robot "<<mapped_robots.at(r));
#endif
}

void ContextCoordinator::computeSearchRoleAssignment()   // assegno i ruoli quando bisogna cercare la palla
{
    for(unsigned int r=0; r<utility_matrix.size(); ++r)
        mapped_robots.at(r) = false;


    if( isMaxRoleScore((int) Role::striker) )
        tmp_role = Role::searcher_3;
    else if( isMaxRoleScore((int) Role::defender) )
        tmp_role = Role::searcher_1;
    else if( isMaxRoleScore((int) Role::supporter) )
        tmp_role = Role::searcher_2;
    else if( isMaxRoleScore((int) Role::jolly) )
        tmp_role = Role::searcher_4;
    else
        tmp_role = Role::undefined;

#ifdef DEEP_DEBUG_CONTEXT_COORD
    SPQR_INFO("tmp: "<<tmp_role); /// when no role has been selected the tmp role assigns the undefined enum.
    for(unsigned int r=0;r<mapped_robots.size(); ++r)
        SPQR_INFO("mapped robot "<<mapped_robots.at(r));
#endif
}

void ContextCoordinator::computeUtilityMatrix(const std::vector<Pose2f>& targets)
{

#ifdef COORDINATION_TODO
    /*
    * test for guarantee consistent state. TODO: check validity of vector (mean?) and do not consider entry r=0 (goalie)
    */

    bool ready = false, late = false;
    unsigned now = SystemCall::getCurrentSystemTime(), diff = 9999;
    int best_time = 0;
    for(int t = 0; t<10; ++t)
    {
        unsigned diff_tmp = now - theTeamData.coordinationBuffer.references[t];//( ( theTeamData.coordinationBuffer.getMax(t) - theTeamData.coordinationBuffer.getMin(t) ) / 2);
        if( diff_tmp < 3000 ) // vector not so distant in time
        {

            if( //theTeamData.coordinationBuffer.informationMatrix[r][t].ts != 0 && // is it just started ? - maybe this check can be erase
                    theTeamData.coordinationBuffer.fallHere( t, 3000 )) // values not distant more than 'range'
            {
                ready = true;
                if( diff_tmp < diff && diff_tmp > 0)
                {
                    diff = diff_tmp;
                    best_time = t;
                }
            }
        }
    }

    if( !ready && (SystemCall::getCurrentSystemTime() - last_utility_computation) > /*threshold for computation*/ 3000) // too late, I must compute something - now it is large for testing
    {
        SPQR_INFO( "LATE!");
        late = true;
    }


    if(ready)
    {
        for(unsigned int r=1; r<5; ++r) // how many robots are alive ?
        {
            //            SPQR_SUCC(theTeamData.coordinationBuffer.informationMatrix[r][best_time].pose.translation.x() << ", "
            //                      << theTeamData.coordinationBuffer.informationMatrix[r][best_time].pose.translation.y());
            if(theTeamData.coordinationBuffer.informationMatrix[r][best_time].pose.translation.x() == .0f) continue;
            for(unsigned int j=1; j<targets.size(); ++j)
            {
                penalty = 0;
                if(isRobotPenalized(r)) penalty = 1;

                utility_matrix.at(r).at(j) = (1-penalty) /** theTeamData.coordinationBuffer.informationMatrix[r][best_time].validity */ * (
                            + translation_weight * (1000*(SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD -
                                                          norm(targets.at(j).translation.x()-theTeamData.coordinationBuffer.informationMatrix[r][best_time].pose.translation.x(),
                                                               targets.at(j).translation.y()-theTeamData.coordinationBuffer.informationMatrix[r][best_time].pose.translation.y()))/
                                                    SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD)
                            + bias_weight * biasWeight(j)
                            + history_weight * getUtilityRoleHistory(j)
                            + orientation_weight * getUtilityOrientationValue(targets.at(j).translation.x(), targets.at(j).translation.y()));
            }
        }
        last_utility_computation = SystemCall::getCurrentSystemTime();
    }
    else if(late) // previous utility computation
    {
        //for(unsigned int r=1; r<theRole.robots_poses.size(); ++r)
        for(unsigned int r=1; r<theTeamData.numberOfActiveTeammates; ++r)   // modificato
        {
            //if(theRole.robots_poses.at(r).translation.x() == .0f) continue;
            if(theTeamData.teammates.at(r).theRobotPose.translation.x() == .0f) continue;  // modificato
            for(unsigned int j=1; j<targets.size(); ++j)
            {
                penalty = 0;
                if(isRobotPenalized(r)) penalty = 1;

                utility_matrix.at(r).at(j) = (1-penalty) * theRobotPose.validity * (
                            + translation_weight * (1000*(SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD -
                                                          //norm(targets.at(j).translation.x()-theRole.robots_poses.at(r).translation.x(),
                                                          //     targets.at(j).translation.y()-theRole.robots_poses.at(r).translation.y()))/
                                                          norm(targets.at(j).translation.x()-theTeamData.teammates.at(r).theRobotPose.translation.x(),    // modificato
                                                          targets.at(j).translation.y()-theTeamData.teammates.at(r).theRobotPose.translation.y()))/
                                                   SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD)
                            + bias_weight * biasWeight(j)
                            + history_weight * getUtilityRoleHistory(j)
                            + orientation_weight * getUtilityOrientationValue(targets.at(j).translation.x(), targets.at(j).translation.y()));
            }
        }
        last_utility_computation = SystemCall::getCurrentSystemTime();
    }

#endif

    float temmate_on_the_ball = 0.2f;

    for(unsigned int r=1; r<theRole.robots_poses.size(); ++r) // how many robots are alive ?
    {
        for(unsigned int j=1; j<targets.size(); ++j)
        {

            if(j == 1)
                temmate_on_the_ball = 0.2f;
            else
                temmate_on_the_ball = 0.f;
            /*
            if (j == 1)  //striker
            {
                translation_weight = 0.6f;
                history_weight = 0.1f;
                target_orientation_weight = 0.15f;
                opp_goal_orientation_weight = 0.15f;
            }
            else if (j ==2) //defender
            {
                translation_weight = 0.7f;
                history_weight = 1;
                target_orientation_weight = 0;
                opp_goal_orientation_weight = 1;
            }
            if (j == 3) //jolly
            {
                translation_weight = 0.7f;
                history_weight = 1.5;
                target_orientation_weight = 1;
                opp_goal_orientation_weight = 0;
            }
            else if (j ==4)     //supporter
            {
                translation_weight = 0.7f;
                history_weight = 0;
                target_orientation_weight = 0;
                opp_goal_orientation_weight = 0;
            }
            */

            if(theRole.robots_poses.at(r).translation.x() == .0f) continue; //check
            int penalty = 0;
            if(isRobotPenalized(r+1)) penalty = 1;  //modificato r+1
//            if(theRobotInfo.number == 2 && theOwnTeamInfo.teamColor == TEAM_BLUE) penalty = 1; // ptl bug
            //std::cout<<"peso histiry   "<< opp_goal_orientation_weight * getUtilityOrientationValue(Pose2f(4500.f, 0.f),theRole.robots_poses.at(r)) <<std::endl;
            utility_matrix.at(r).at(j) = ((1-penalty) /** theTeammateData.coordinationBuffer.informationMatrix[r][best_time].validity */ *  (
                        + translation_weight * ((SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD -
                                                      norm(targets.at(j).translation.x()-theRole.robots_poses.at(r).translation.x(),
                                                           targets.at(j).translation.y()-theRole.robots_poses.at(r).translation.y()))/
                                                SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD)
                        //+ bias_weight * biasWeight(j)  //default 0
                        + history_weight * getUtilityRoleHistory(j)  //default 1
                        + target_orientation_weight * getUtilityOrientationValue(targets.at(j),theRole.robots_poses.at(r))  //orient respect the target
                        + opp_goal_orientation_weight * getUtilityOrientationValue(Pose2f(theFieldDimensions.xPosOpponentGroundline, 0.f),theRole.robots_poses.at(r)) //orient respect opponent goal
                        //- temmate_on_the_ball * getTemmateOntheBallValue(targets.at(j), theRole.robots_poses.at(r))                        
                        ));
        }
    }
}


void ContextCoordinator::updateRobotRole(Role& role)
{
    /// WARNING: handle cases like: role.role == 0
    if( (role.role == Role::undefined && tmp_role != Role::undefined) ||
            //role.role == 0 ||
            !isInCurrentContext(role.role) )
    {
        //if(theRobotInfo.number == 2)
            //std::cout<<"coordination NO  role.role = "<< role.role<< "  |  tmprole= "<<tmp_role<<"  is incurr  "<<isInCurrentContext(role.role)<<std::endl;
        role.lastRole = role.role;
        role.role = tmp_role;
        myRole = tmp_role;
    }
    else
    {
        if(role.role != tmp_role && startHysteresis == true)
        {
            //std::cout << "aumenta isteresi   "<<  role_hysteresis_cycle<<std::endl;
            role_hysteresis_cycle = theFrameInfo.time;
            //++role_hysteresis_cycle;
            startHysteresis = false;
            //std::cout << " isteresi   "<< hysteresis <<std::endl;
        }

        if ( role_hysteresis_cycle != 0 && theFrameInfo.getTimeSince(role_hysteresis_cycle) > time_hysteresis)

        {
            //if(theRobotInfo.number == 2)
                //std::cout<<"coordination Hister role.role = "<< role.role<< "  |  tmprole= "<<tmp_role<< "   cycle = "<<role_hysteresis_cycle<<std::endl;

            role.lastRole = role.role;
            role.role = tmp_role;
            myRole = tmp_role;
            role_hysteresis_cycle = 0;
            // role_time.setToNow();
            // role_time.getCurrentSystemTime();  // modificato
            role_time = Time::getRealSystemTime();
            role_hysteresis_cycle = 0;
            startHysteresis =true;
        }
    }
    //role.utility_vector = utility_matrix;

    //if(theRobotInfo.number == 2)
        //std::cout<<"ME INTERESSA  "<<  role.lastRole<< "  "<<role.role<<std::endl;
    /*
    if(theRobotInfo.number == 2 && role_hysteresis_cycle == 99)
        if(tmp_role != role.role)
            for(int i = 0; i<utility_matrix.size();i++)
                for(int j = 0; j < utility_matrix.at(i).size();j++)
                    std::cout<<"robot    -  "<<i<<"  =  "<<utility_matrix.at(i).at(j)<<std::endl;*/

}

void ContextCoordinator::updatePlayingRoleSpace(Role& role)
{


    if(theRobotInfo.number == 1)
    {
        role.role = Role::goalie;
        myRole = Role::goalie;
    }
    else
    {
        /// define targets: the order is crazy important
        std::vector<Pose2f> targets;
        targets.push_back(goalie_pose);
        if(ballSeen){
            targets.push_back(Pose2f(rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y())));
        } else {
            targets.push_back(Pose2f(theTeamBallModel.position.x(), theTeamBallModel.position.y()));//VINCENZO
        }

        defender_pose = Pose2f(defender_pose.translation.x(), theLibCheck.defenderDynamicY());       //VINCENZO
        targets.push_back((Pose2f) defender_pose);

        /*
        if(theTeamBallModel.position.y()>0)
            targets.push_back(Pose2f(2000.f, -2000.f));
        else
            targets.push_back(Pose2f(2000.f, 2000.f));
        */
        
        supporter_pose = theLibCheck.getSupporterPosition(); //VINCENZO
        targets.push_back(supporter_pose);       //VINCENZO

        jolly_pose = Pose2f(theLibCheck.getJollyPosition().x(), theLibCheck.getJollyPosition().y()); // TODO VINCENZO
        targets.push_back(jolly_pose);           //VINCENZO

//        targets.push_back((Pose2f) supporter_pose);
//        targets.push_back((Pose2f) jolly_pose);



#if defined(DEBUG_AND_WRITE_ROLE_PERSISTENCE)
        Pose2f posa_palla = Pose2f(rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
#endif

        /// compute the utility matrix
        computeUtilityMatrix(targets);

//        for(int i = 0; i<utility_matrix.size();i++)
//            for(int j = 0; j < utility_matrix.at(i).size();j++)
//                std::cout<<"posizione  "<<i<<"  -  "<<j<<"  =  "<<utility_matrix.at(i).at(j)<<std::endl;

        /// map the current configuration in the 'role space'
        computePlayingRoleAssignment();
        /// update spqr robot role
        updateRobotRole(role);

        /*
        * PLOTTING
        */
#if defined(DEBUG_AND_WRITE_ROLE_PERSISTENCE)
        Vector2f my_pos = Vector2f(
                    //theRole.robots_poses.at( theRobotInfo.number-1 ).translation.x(),
                    //theRole.robots_poses.at( theRobotInfo.number-1 ).translation.y()
                    theTeamData.teammates.at(theRobotInfo.number-1).theRobotPose.translation.x(),  // modificato sbagliato5
                    theTeamData.teammates.at(theRobotInfo.number-1).theRobotPose.translation.y()
                    );
        Vector2f my_role_pos;
        std::stringstream to_plot;
        switch(role.role)
        {
        case Role::goalie:
            my_role_pos = Vector2f( goalie_pose.translation.x(), goalie_pose.translation.y() );
            to_plot << "Goalie";
            break;
        case Role::defender:
            my_role_pos = Vector2f( defender_pose.translation.x(), defender_pose.translation.y() );
            to_plot << "Defender";
            break;
        case Role::jolly:
            my_role_pos = Vector2f( jolly_pose.translation.x(), jolly_pose.translation.y() );
            to_plot << "Jolly";
            break;
        case Role::supporter:
            my_role_pos = Vector2f( supporter_pose.translation.x(), supporter_pose.translation.y() );
            to_plot << "Supporter";
            break;
        case Role::striker:
            my_role_pos = Vector2f( posa_palla.translation.x(), posa_palla.translation.y() );
            to_plot << "Striker";
            break;
        case Role::undefined:
            my_role_pos = Vector2f( 0,0 );
            to_plot << "undefined";
            break;
        }

        if( role.role == Role::undefined )
            to_plot << " <> MY POSITION: " << my_pos.x << "," << my_role_pos.y <<
                       " <> MY ROLE POSITION: " << 0 << "," << 0 <<
                       " <> DISTANCE: " << 9000.0;
        else
            to_plot << " <> MY POSITION: " << my_pos.x << "," << my_pos.y <<
                       " <> MY ROLE POSITION: " << my_role_pos.x << "," << my_role_pos.y <<
                       " <> DISTANCE: " << (my_pos - my_role_pos).absFloat();

        //SPQR_INFO(to_plot.str());

        std::ofstream myfile;
        std::stringstream ss;
        ss << "log_roles/log_roles_" << theRobotInfo.number << (theOwnTeamInfo.teamColor == TEAM_BLUE ? "_(BLUE)" : "_(RED)" ) << ".txt";
        myfile.open(ss.str(),std::ofstream::ate | std::ofstream::app);
        myfile << to_plot.str() << "\n";
        myfile.close();
#endif

    }
}

void ContextCoordinator::updateSearchRoleSpace(Role& role)
{
    if(theRobotInfo.number == 1)
    {
        role.role = Role::goalie;
        myRole = Role::goalie;
    }
    else
    {
        /// define targets
        //         std::vector<Pose2f> targets;
        //        targets.push_back(goalie_pose);
        //        targets.push_back(defender_pose);
        //        //for(unsigned int i=0; i<theSpqrDWKcombiner.unexplored_clusters_centroids.size(); ++i)
        //        //    targets.push_back(theSpqrDWKcombiner.unexplored_clusters_centroids.at(i));
        //        for(unsigned int i=0; i<unexplored_clusters_centroids.size(); ++i){
        //            targets.push_back(unexplored_clusters_centroids.at(i));    //modificato

        std::vector<Pose2f> targets;
        targets.push_back(goalie_pose);
        targets.push_back(Pose2f(theTeamBallModel.position.x(), theTeamBallModel.position.y()));

        //targets.push_back(defender_pose);
        //targets.push_back(supporter_pose);

        targets.push_back((Pose2f)theLibCheck.defenderPosition);
        targets.push_back((Pose2f)theLibCheck.supporterPosition);

        float searcherYpose;
        if( theTeamBallModel.position.y() > 0)
            searcherYpose = -( (SPQR::FIELD_DIMENSION_Y)/2 );
        else
            searcherYpose = (SPQR::FIELD_DIMENSION_Y)/2 ;


        targets.push_back(Pose2f(0.62f*SPQR::FIELD_DIMENSION_X, searcherYpose));

        /// compute the utility matrix

        computeUtilityMatrix(targets);

        /// map the current configuration in the 'role space'
        computeSearchRoleAssignment();

        /// update spqr robot role
        updateRobotRole(role);
    }
}

void ContextCoordinator::updateRobotPoses(Role& role)
{
    role.robots_poses.at(theRobotInfo.number-1) = Pose2f(theRobotPose.rotation,theRobotPose.translation.x(),theRobotPose.translation.y());
    if(!theTeamData.teammates.size()) return;

    for(unsigned int i=0; i<theTeamData.teammates.size(); ++i)
    {
        //theTeamData.teammates.at(i).thePassShare.role;

        Pose2f tmp_pose = Pose2f(theTeamData.teammates.at(i).theRobotPose.rotation,
                                 theTeamData.teammates.at(i).theRobotPose.translation.x(),
                                 theTeamData.teammates.at(i).theRobotPose.translation.y());

        role.robots_poses.at(theTeamData.teammates.at(i).number-1) = tmp_pose;
    }
}

void ContextCoordinator::update(Role& role)
{

    if(theGameInfo.state == STATE_PLAYING)
    {
        updateRobotPoses(role);


        if ( theFrameInfo.time - theBallModel.timeWhenLastSeen < time_when_last_seen)
        {

            ballSeen=true;
        }
        else if ( theTeamBallModel.isValid)
        {
            teamBall=true;
            ballSeen=false;
        }
        else
        {
            ballSeen=false;
            teamBall=false;
        }

//        if(!ballSeen && !teamBall){
//            if (prev_status == Role::playing){
//                if(flag){
//                    stamp = Time::getRealSystemTime();
//                    flag = false;
//                }
//                if (Time::getRealSystemTime() > stamp + 5000 ){
//                    current_status = Role::search_for_ball;
//                    prev_status = Role::search_for_ball;
//                    flag = true;
//                }
//            }
//            else
//                current_status = Role::search_for_ball;
//        }

//        else{
//            current_status = Role::playing;
//            prev_status = Role::playing;
//        }

        if(!ballSeen && !teamBall)
        {
            prev_status = current_status;
            current_status = Role::search_for_ball;
        }

        else
        {
            prev_status = current_status;
            current_status = Role::playing;
        }
        role.current_context = current_status;

        /// playing context
        if(current_status == Role::playing){
            updatePlayingRoleSpace(role);
        }
        /// search for ball context
        else if(current_status == Role::search_for_ball && (prev_status==Role::no_context || prev_status==Role::playing)){
            updateSearchRoleSpace(role);

            }

        /// assign keeper role   ----------------- Goal Keeper
        if(theRobotInfo.number == 1)
        {
            role.role = Role::goalie;
            myRole = Role::goalie;
        }
    }

#ifdef DEBUG_CONTEXT_COORD
    if(theGameInfo.state == STATE_PLAYING && theRobotInfo.number != 1)
        //        if(theGameInfo.state == STATE_PLAYING && (theRobotInfo.number == 2 || theRobotInfo.number == 5) )
    {
        // if ((PTracking::Timestamp() - stamp).getMs() > 3000)
        if ((Time::getTimeSince(stamp)) > 3000)    // modificato è gia in millisecondi
        {
            //for(unsigned int i=0; i<unexplored_clusters_centroids.size(); ++i)
            //    SPQR_INFO("Centroid "<<i<<": "<<unexplored_clusters_centroids.at(i).x<<", "<<
            //              unexplored_clusters_centroids.at(i).y);
            for(unsigned int i=0; i<unexplored_clusters_centroids.size(); ++i)  // modificato
                SPQR_INFO("Centroid "<<i<<": "<<unexplored_clusters_centroids.at(i).x<<", "<<
                          unexplored_clusters_centroids.at(i).y);
        }
    }
#endif

#ifdef UM_VIEW
    //        if(theGameInfo.state == STATE_PLAYING && (theRobotInfo.number == 5) )
    if(theGameInfo.state == STATE_PLAYING)
    {
        if( theRobotInfo.number != 1 )
        {
            // if ((PTracking::Timestamp() - stamp).getMs() > 3000)
            if ((Time::getTimeSince(stamp)) > 3000) // modificato
            {
                //std::cout<<"Context: "; CONTEXT(theRole.current_context);
                std::cerr<<"Spqr Role mapped to robot ["<<theRobotInfo.number<<"] =>  "<< (int) role.role<<std::endl;
                //SPQR_ROLE((int) role.role);
                for(unsigned int i=0; i<utility_matrix.size(); ++i)
                {
                    for(unsigned int j=0; j<utility_matrix.at(i).size(); ++j)
                    {
                        if(utility_matrix.at(i).at(j) >= 850) std::cout<<"\033[0;32;1m"<<"["<<utility_matrix.at(i).at(j)<<"]--- \033[0m";
                        else if(utility_matrix.at(i).at(j) < 100) std::cout<<"\033[22;31;1m"<<"["<<utility_matrix.at(i).at(j)<<"]--- \033[0m";
                        else std::cout<<"\033[22;34;1m"<<"["<<utility_matrix.at(i).at(j)<<"]--- \033[0m";
                    }

                    std::cout<<std::endl;
                }
                std::cout<<std::endl;

                // stamp.setToNow();
                //stamp.getCurrentSystemTime();    // modificato
                stamp = Time::getRealSystemTime();

            }
        }
    }
#endif
}
