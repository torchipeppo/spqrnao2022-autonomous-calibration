#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/RoboCupGameControlData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/OurDefinitions.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"



// #include "Timestamp.h" // nuovo ma non funge

// #include <SPQR-Libraries/PTracking/Core/Processors/Processor.h>
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <mutex>


MODULE(ContextCoordinator, //TODO need to change Forget and Change as streamable and sand to all also the role toghether with the timestamp.. byebye
{,
 REQUIRES(GameInfo),
 REQUIRES(OpponentTeamInfo),
 REQUIRES(OwnTeamInfo),
 REQUIRES(RobotInfo),
 REQUIRES(RobotPose),
 REQUIRES(BallModel),
 REQUIRES(FrameInfo),
 REQUIRES(FallDownState),
 REQUIRES(TeamData),
 REQUIRES(TeamBallModel),
 REQUIRES(FieldDimensions),
 REQUIRES(ObstaclesFieldPercept),
 REQUIRES(LibCheck),
 USES(Role),
// USES(UtilityShare),
// USES(RoboCupGameControlData),
 PROVIDES(Role),

 LOADS_PARAMETERS(
 {,
  //(int) dead_robot_time_threshold,
  (int) time_hysteresis,
  (int) fall_down_penalty,
  (float) target_orientation_weight,
  (float) opp_goal_orientation_weight,
  (float) translation_weight,
  //(float) striker_weight,
  (float) history_weight,
  (float) bias_weight,
  (float) goalie_pose_x,
  (float) goalie_pose_y,
  (float) goalie_pose_t,
  (float) defender_pose_x,
  (float) defender_pose_y,
  (float) defender_pose_t,
  (float) supporter_pose_x,
  (float) supporter_pose_y,
  (float) supporter_pose_t,
  (float) jolly_pose_x,
  (float) jolly_pose_y,
  (float) jolly_pose_t,
  (float) striker_pose_x,
  (float) striker_nokickoff_pose_x,
  (float) striker_pose_y,
  (float) striker_pose_t,
  (float) threshold_striker,
//  (int) request_timeout,
//  (int) request_timeout_others,
//  (int) countdown_threshold,
  (unsigned int) time_when_last_seen,
 }),
       });

class ContextCoordinator: public ContextCoordinatorBase
{

private:

    /** Utilities */
  float norm(float x, float y){ return (float)sqrt(x*x + y*y); }
    float sign(float x){ if (x >= 0) return 1.f; else return -1.f; }
    Vector2f rel2Glob(float x, float y) const;



    unsigned last_utility_computation;
    bool ballSeen;
    bool teamBall;

public:

    // PTracking::Timestamp stamp, role_time;     // attenzione role_time e timestamp
    unsigned stamp, role_time, context_time;
    bool flag = true;
    std::vector<Vector2f> explored_clusters_centroids;    // aggiunti per togliere spqrdwk
    std::vector<Vector2f> unexplored_clusters_centroids;

    //    int dead_robot_time_threshold;
    //    int time_hysteresis;
    //    int fall_down_penalty;
    //    int penalty;

    //    float orientation_weight, translation_weight,
    //    striker_weight, history_weight, bias_weight;

    Pose2f goalie_pose, defender_pose, supporter_pose,
    jolly_pose, striker_pose, striker_nokickoff_pose;

    std::vector<std::vector<float> > utility_matrix;
    Role::RoleType tmp_role, myRole;

    Role::Context prev_status = Role::no_context;

    Role::Context current_status = Role::no_context;


    int role_hysteresis_cycle;
    bool startHysteresis;
    std::vector<bool> mapped_robots;
    std::vector<int> penalized_robots;

    void configure();

    bool isInCurrentContext(Role::RoleType currentRole);
    float getUtilityOrientationValue(Pose2f target, Pose2f robot) const;
    float getUtilityRoleHistory(int j) const;
    float getTemmateOntheBallValue(Pose2f target, Pose2f robotPos) const;
    int biasWeight(int role) const;
    bool isMaxRoleScore(int c);
    bool isRobotPenalized(int r);
    void computePlayingRoleAssignment();
    void computeSearchRoleAssignment();

    void computeUtilityMatrix(const std::vector<Pose2f>& targets);

    void updateRobotRole(Role& role);
    void updateRobotPoses(Role& role);

    void updatePlayingRoleSpace(Role& role);
    void updateSearchRoleSpace(Role& role);

    ContextCoordinator();
    void update(Role& role);

    //void update(ContextCoordination& contextCoordination);

};
