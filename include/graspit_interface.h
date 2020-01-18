#ifndef _GRASPIT_INTERFACE_H_
#define _GRASPIT_INTERFACE_H_

// ROS includes
#ifndef Q_MOC_RUN
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

// GraspIt! includes
#include <graspit/EGPlanner/searchState.h>
#include <graspit/EGPlanner/simAnnPlanner.h>
#include <graspit/plugin.h>
// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl_conversions/pcl_conversions.h>

// QT
#include <QFile>
#include <QLabel>
#include <QObject>
// Message includes
#include "scene_completion_msgs/CompleteSceneAction.h"
#include <graspit_interface/SearchSpace.h>
// Service includes
#include <graspit_interface/AddObject.h>
#include <graspit_interface/ApproachToContact.h>
#include <graspit_interface/AutoGrasp.h>
#include <graspit_interface/AutoOpen.h>
#include <graspit_interface/ClearWorld.h>
#include <graspit_interface/ComputeEnergy.h>
#include <graspit_interface/ComputeQuality.h>
#include <graspit_interface/DynamicAutoGraspComplete.h>
#include <graspit_interface/FindInitialContact.h>
#include <graspit_interface/ForceRobotDOF.h>
#include <graspit_interface/GetBodies.h>
#include <graspit_interface/GetBody.h>
#include <graspit_interface/GetDynamics.h>
#include <graspit_interface/GetGraspableBodies.h>
#include <graspit_interface/GetGraspableBody.h>
#include <graspit_interface/GetRobot.h>
#include <graspit_interface/GetRobots.h>
#include <graspit_interface/ImportGraspableBody.h>
#include <graspit_interface/ImportObstacle.h>
#include <graspit_interface/ImportRobot.h>
#include <graspit_interface/LoadWorld.h>
#include <graspit_interface/MoveDOFToContacts.h>
#include <graspit_interface/Robot.h>
#include <graspit_interface/SaveImage.h>
#include <graspit_interface/SaveWorld.h>
#include <graspit_interface/SetBodyPose.h>
#include <graspit_interface/SetDynamics.h>
#include <graspit_interface/SetGraspableBodyPose.h>
#include <graspit_interface/SetRobotDesiredDOF.h>
#include <graspit_interface/SetRobotPose.h>
#include <graspit_interface/ToggleAllCollisions.h>

#include <graspit_msgs/ObjectInfo.h>
// ActionServer includes
#include <graspit_interface/ExecuteXBestGraspsAction.h>
#include <graspit_interface/GetSegmentedMeshedSceneAction.h>
#include <graspit_interface/PlanGraspsAction.h>
#include <graspit_interface/PlanXBestGraspsAction.h>
#include <graspit_interface/SimulationExperimentAction.h>
#endif

#define AXIS_SCALE 1000
typedef Eigen::Vector3d position;
typedef Eigen::Vector3d vec3;
typedef Eigen::Quaterniond Quaternion;

namespace GraspitInterface {

class GraspitInterface : public QObject, public Plugin {

  Q_OBJECT

private:
  bool render_graphics = true;
  bool evaluate_on_shape_samples = false;
  std::string file_path_to_ground_truth_meshes;
  std::string file_path_to_shape_completed_meshes;
  std::string file_path_to_save_location;

  std::string grasp_planning_method;
  int number_of_best_grasps_to_return;
  bool rank_grasps_on_samples;

  std::vector<int> indices_of_best_epsilon_grasps;
  std::vector<int> indices_of_best_volume_grasps;
  std::vector<sensor_msgs::JointState> gripper_joint_state_for_all_grasps;
  std::vector<geometry_msgs::Pose> gripper_pose_for_all_grasps;

  std::string mean_mesh_file_path;
  geometry_msgs::Vector3 mesh_offset;

  ros::NodeHandle *nh;

  // Service declarations
  ros::ServiceServer add_object_srv;

  ros::ServiceServer get_robot_srv;
  ros::ServiceServer get_graspable_body_srv;
  ros::ServiceServer get_body_srv;

  ros::ServiceServer get_robots_srv;
  ros::ServiceServer get_graspable_bodies_srv;
  ros::ServiceServer get_bodies_srv;

  ros::ServiceServer set_robot_pose_srv;
  ros::ServiceServer set_body_pose_srv;
  ros::ServiceServer set_graspable_body_pose_srv;

  ros::ServiceServer get_dynamics_srv;
  ros::ServiceServer set_dynamics_srv;

  ros::ServiceServer auto_grasp_srv;
  ros::ServiceServer auto_open_srv;
  ros::ServiceServer force_robot_dof_srv;
  ros::ServiceServer move_dof_to_contacts_srv;
  ros::ServiceServer set_robot_desired_dof_srv;

  ros::ServiceServer import_robot_srv;
  ros::ServiceServer import_obstacle_srv;
  ros::ServiceServer import_graspable_body_srv;

  ros::ServiceServer clear_world_srv;
  ros::ServiceServer load_world_srv;
  ros::ServiceServer save_world_srv;

  ros::ServiceServer save_image_srv;
  ros::ServiceServer toggle_all_collisions_srv;

  ros::ServiceServer compute_quality_srv;
  ros::ServiceServer compute_energy_srv;

  ros::ServiceServer approach_to_contact_srv;
  ros::ServiceServer find_initial_contact_srv;

  ros::ServiceServer dynamic_auto_grasp_complete_srv;

  // ActionServer declarations
  actionlib::SimpleActionServer<graspit_interface::PlanGraspsAction>
      *plan_grasps_action_server;
  graspit_interface::PlanGraspsFeedback feedback_;
  graspit_interface::PlanGraspsResult result_;
  graspit_interface::PlanGraspsGoal goal;

  actionlib::SimpleActionClient<graspit_interface::ExecuteXBestGraspsAction>
      *execute_x_best_grasps_action_client;

  actionlib::SimpleActionClient<graspit_interface::PlanXBestGraspsAction>
      *plan_x_best_grasps_action_client;

  actionlib::SimpleActionClient<graspit_interface::SimulationExperimentAction>
      *simulation_experiment_action_client;

  actionlib::SimpleActionClient<scene_completion_msgs::CompleteSceneAction>
      *shape_complete_scene_action_client;

  ros::Publisher meshed_scene_repub;
  ros::Subscriber meshed_scene_sub;

  ros::ServiceClient clear_world_client;
  ros::ServiceClient add_objects_client;
  ros::ServiceClient get_graspable_bodies_client;
  ros::ServiceClient save_world_client;
  ros::ServiceClient add_object_client;

  GraspPlanningState *m_hand_object_state;
  SimAnnPlanner *m_planner;

  QLabel *scene_completion_time;
  QLabel *grasp_planning_time;

  Body *selected_body;

  void visualizeGrasp(const sensor_msgs::JointState &gripper_joints,
                      const geometry_msgs::Pose &gripper_pose);
  void addToWorld(const QString modelname, const QString object_name,
                  const transf object_pose);
  void addObject(graspit_msgs::ObjectInfo object);
  bool first_time_in_main_loop;

  // Service callbacks
  bool addObjectCB(graspit_interface::AddObject::Request &request,
                   graspit_interface::AddObject::Response &response);

  bool getRobotCB(graspit_interface::GetRobot::Request &request,
                  graspit_interface::GetRobot::Response &response);

  bool
  getGraspableBodyCB(graspit_interface::GetGraspableBody::Request &request,
                     graspit_interface::GetGraspableBody::Response &response);

  bool getBodyCB(graspit_interface::GetBody::Request &request,
                 graspit_interface::GetBody::Response &response);

  bool getRobotsCB(graspit_interface::GetRobots::Request &request,
                   graspit_interface::GetRobots::Response &response);

  bool getGraspableBodiesCB(
      graspit_interface::GetGraspableBodies::Request &request,
      graspit_interface::GetGraspableBodies::Response &response);

  bool getBodiesCB(graspit_interface::GetBodies::Request &request,
                   graspit_interface::GetBodies::Response &response);

  bool setRobotPoseCB(graspit_interface::SetRobotPose::Request &request,
                      graspit_interface::SetRobotPose::Response &response);

  bool setGraspableBodyPoseCB(
      graspit_interface::SetGraspableBodyPose::Request &request,
      graspit_interface::SetGraspableBodyPose::Response &response);

  bool setBodyPoseCB(graspit_interface::SetBodyPose::Request &request,
                     graspit_interface::SetBodyPose::Response &response);

  bool getDynamicsCB(graspit_interface::GetDynamics::Request &request,
                     graspit_interface::GetDynamics::Response &response);

  bool setDynamicsCB(graspit_interface::SetDynamics::Request &request,
                     graspit_interface::SetDynamics::Response &response);

  bool autoGraspCB(graspit_interface::AutoGrasp::Request &request,
                   graspit_interface::AutoGrasp::Response &response);

  bool autoOpenCB(graspit_interface::AutoOpen::Request &request,
                  graspit_interface::AutoOpen::Response &response);

  bool forceRobotDOFCB(graspit_interface::ForceRobotDOF::Request &request,
                       graspit_interface::ForceRobotDOF::Response &response);

  bool
  moveDOFToContactsCB(graspit_interface::MoveDOFToContacts::Request &request,
                      graspit_interface::MoveDOFToContacts::Response &response);

  bool setRobotDesiredDOFCB(
      graspit_interface::SetRobotDesiredDOF::Request &request,
      graspit_interface::SetRobotDesiredDOF::Response &response);

  bool importRobotCB(graspit_interface::ImportRobot::Request &request,
                     graspit_interface::ImportRobot::Response &response);

  bool importObstacleCB(graspit_interface::ImportObstacle::Request &request,
                        graspit_interface::ImportObstacle::Response &response);

  bool importGraspableBodyCB(
      graspit_interface::ImportGraspableBody::Request &request,
      graspit_interface::ImportGraspableBody::Response &response);

  bool loadWorldCB(graspit_interface::LoadWorld::Request &request,
                   graspit_interface::LoadWorld::Response &response);

  bool saveWorldCB(graspit_interface::SaveWorld::Request &request,
                   graspit_interface::SaveWorld::Response &response);

  bool clearWorldCB(graspit_interface::ClearWorld::Request &request,
                    graspit_interface::ClearWorld::Response &response);

  bool saveImageCB(graspit_interface::SaveImage::Request &request,
                   graspit_interface::SaveImage::Response &response);

  bool toggleAllCollisionsCB(
      graspit_interface::ToggleAllCollisions::Request &request,
      graspit_interface::ToggleAllCollisions::Response &response);

  bool computeQualityCB(graspit_interface::ComputeQuality::Request &request,
                        graspit_interface::ComputeQuality::Response &response);

  bool computeEnergyCB(graspit_interface::ComputeEnergy::Request &request,
                       graspit_interface::ComputeEnergy::Response &response);

  bool
  approachToContactCB(graspit_interface::ApproachToContact::Request &request,
                      graspit_interface::ApproachToContact::Response &response);

  bool findInitialContactCB(
      graspit_interface::FindInitialContact::Request &request,
      graspit_interface::FindInitialContact::Response &response);

  bool dynamicAutoGraspCompleteCB(
      graspit_interface::DynamicAutoGraspComplete::Request &request,
      graspit_interface::DynamicAutoGraspComplete::Response &response);

  // ActionServer callbacks
  void PlanGraspsCB(const graspit_interface::PlanGraspsGoalConstPtr &goal);

  void graspPlanningStateToROSMsg(const GraspPlanningState *gps,
                                  graspit_interface::Grasp &g, Hand *mHand);

  void addMesh(QString mesh_index, shape_msgs::Mesh mesh,
               geometry_msgs::Vector3 offset, const bool is_mean = true);

  void saveMesh(const QString filepath, const QString filename,
                const pcl::PolygonMesh *pcl_mesh, const bool is_mean = false);
  // ActionClient Feedback
  void executeXBestGraspsFeedback(
      const graspit_interface::ExecuteXBestGraspsFeedbackConstPtr &feedback);

  // Convenience functions for converting between pose types:
  inline geometry_msgs::Pose transfToRosMsg(transf pose) {
    geometry_msgs::Pose ret;
    ret.position.x = pose.translation().x() / AXIS_SCALE;
    ret.position.y = pose.translation().y() / AXIS_SCALE;
    ret.position.z = pose.translation().z() / AXIS_SCALE;
    ret.orientation.w = pose.rotation().w();
    ret.orientation.x = pose.rotation().x();
    ret.orientation.y = pose.rotation().y();
    ret.orientation.z = pose.rotation().z();
    return ret;
  }
  inline transf rosMsgToTransf(geometry_msgs::Pose pose) {
    Quaternion q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                 pose.orientation.z);
    vec3 p(pose.position.x * AXIS_SCALE, pose.position.y * AXIS_SCALE,
           pose.position.z * AXIS_SCALE);
    transf ret(q, p);
    return ret;
  }

public:
  GraspitInterface() {}
  ~GraspitInterface() {}

  virtual int init(int argc, char **argv);

  virtual int mainLoop();
  void receivedMeshedSceneCB(
      const actionlib::SimpleClientGoalState &state,
      const scene_completion_msgs::CompleteSceneResultConstPtr &result);
  void getSegmentedMeshesCB(
      const scene_completion_msgs::CompleteSceneResultConstPtr &result);
  void planXBestGraspsCB(
      const actionlib::SimpleClientGoalState &state,
      const graspit_interface::PlanXBestGraspsResultConstPtr &result);
  void executeXBestGraspsCB(
      const actionlib::SimpleClientGoalState &state,
      const graspit_interface::ExecuteXBestGraspsResultConstPtr &result);
  void simulationExperimentCB(
      const actionlib::SimpleClientGoalState &state,
      const graspit_interface::SimulationExperimentResultConstPtr &result);

public Q_SLOTS:

  void runPlannerInMainThread();
  void processPlannerResultsInMainThread();
  void buildFeedbackInMainThread();

Q_SIGNALS:

  void emitRunPlannerInMainThread();
  void emitProcessPlannerResultsInMainThread();
  void emitBuildFeedbackInMainThread();

public slots:
  void onShapeCompleteSceneButtonPressed();
  void onGenerateGraspsButtonPressed();
  void onExecuteHighestRankedGraspButtonPressed();
  void onSimulationExperimentButtonPressed();
  void onBrowseButtonPressed(int);
  void onRunButtonPressed();
  void onEvaluateOnShapeSamplesRadioButtonPressed();
};

} // namespace GraspitInterface

#endif
