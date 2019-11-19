#include "graspit_interface.h"
#include <QtGui>
#include <graspit/graspitCore.h>
#include <graspit/ivmgr.h>
#include <graspit/robot.h>
#include <graspit/world.h>
#include <string>
#include <thread>

#include "graspit/EGPlanner/search.h"
#include <graspit/EGPlanner/egPlanner.h>
#include <graspit/EGPlanner/energy/searchEnergy.h>
#include <graspit/EGPlanner/energy/searchEnergyFactory.h>
#include <graspit/EGPlanner/guidedPlanner.h>
#include <graspit/EGPlanner/searchState.h>
#include <graspit/EGPlanner/simAnnParams.h>
#include <graspit/EGPlanner/simAnnPlanner.h>
#include <graspit/bodySensor.h>
#include <graspit/cmdline/cmdline.h>
#include <graspit/grasp.h>
#include <graspit/quality/qualEpsilon.h>
#include <graspit/quality/qualVolume.h>
#include <graspit/quality/quality.h>
using namespace graspit_msgs;
namespace GraspitInterface {
void meshMsgToVerticesTriangles(const shape_msgs::Mesh &mesh,
                                std::vector<position> *vertices,
                                std::vector<int> *triangles,
                                pcl::PolygonMesh &pcl_mesh) {
  pcl::PointCloud<pcl::PointXYZ> *pcl_temp =
      new pcl::PointCloud<pcl::PointXYZ>();

  for (int i = 0; i < mesh.vertices.size(); i++) {
    geometry_msgs::Point p = mesh.vertices.at(i);
    vertices->push_back(position(p.x, p.y, p.z));
    pcl::PointXYZ *point = new pcl::PointXYZ(p.x, p.y, p.z);
    pcl_temp->push_back(*point);
  }

  pcl::toPCLPointCloud2(*pcl_temp, pcl_mesh.cloud);

  for (int i = 0; i < mesh.triangles.size(); i++) {
    shape_msgs::MeshTriangle t = mesh.triangles.at(i);
    triangles->push_back(t.vertex_indices.at(0));
    triangles->push_back(t.vertex_indices.at(1));
    triangles->push_back(t.vertex_indices.at(2));

    pcl::Vertices *p = new pcl::Vertices;
    p->vertices.push_back(t.vertex_indices.at(0));
    p->vertices.push_back(t.vertex_indices.at(1));
    p->vertices.push_back(t.vertex_indices.at(2));
    pcl_mesh.polygons.push_back(*p);
  }
}
void verticesToMeshMsg(shape_msgs::Mesh &mesh,
                       std::vector<position> *vertices) {

  for (int i = 0; i < vertices->size(); i++) {
    position p = vertices->at(i);
    geometry_msgs::Point gp;
    gp.x = p.x();
    gp.y = p.y();
    gp.z = p.z();
    mesh.vertices.push_back(gp);
  }
}
int GraspitInterface::init(int argc, char **argv) {
  ros::init(argc, argv, "graspit_interface_node");

  const std::string node_name_help = "print Ros Node name\n";

  cmdline::parser *parser = new cmdline::parser();

  parser->add<std::string>("node_name", 'n', node_name_help, false);
  parser->parse(argc, argv);

  std::string node_name = "";

  if (parser->exist("node_name")) {
    node_name = parser->get<std::string>("node_name");
  }

  nh = new ros::NodeHandle(node_name);
  nh->getParam("/Render", render_graphics);
  nh->getParam("/Grasp_Planner", grasp_planning_method);
  nh->getParam("/Number_of_Best_Grasp", number_of_best_grasps_to_return);
  nh->getParam("/Rank_Grasps_on_Samples", rank_grasps_on_samples);
  add_object_srv =
      nh->advertiseService("addObject", &GraspitInterface::addObjectCB, this);

  get_robot_srv =
      nh->advertiseService("getRobot", &GraspitInterface::getRobotCB, this);
  get_graspable_body_srv = nh->advertiseService(
      "getGraspableBody", &GraspitInterface::getGraspableBodyCB, this);
  get_body_srv =
      nh->advertiseService("getBody", &GraspitInterface::getBodyCB, this);
  get_robots_srv =
      nh->advertiseService("getRobots", &GraspitInterface::getRobotsCB, this);
  get_graspable_bodies_srv = nh->advertiseService(
      "getGraspableBodies", &GraspitInterface::getGraspableBodiesCB, this);
  get_bodies_srv =
      nh->advertiseService("getBodies", &GraspitInterface::getBodiesCB, this);
  set_robot_pose_srv = nh->advertiseService(
      "setRobotPose", &GraspitInterface::setRobotPoseCB, this);
  set_body_pose_srv = nh->advertiseService(
      "setBodyPose", &GraspitInterface::setBodyPoseCB, this);
  set_graspable_body_pose_srv = nh->advertiseService(
      "setGraspableBodyPose", &GraspitInterface::setGraspableBodyPoseCB, this);

  get_dynamics_srv = nh->advertiseService(
      "getDynamics", &GraspitInterface::getDynamicsCB, this);
  set_dynamics_srv = nh->advertiseService(
      "setDynamics", &GraspitInterface::setDynamicsCB, this);

  auto_grasp_srv =
      nh->advertiseService("autoGrasp", &GraspitInterface::autoGraspCB, this);
  auto_open_srv =
      nh->advertiseService("autoOpen", &GraspitInterface::autoOpenCB, this);

  force_robot_dof_srv = nh->advertiseService(
      "forceRobotDof", &GraspitInterface::forceRobotDOFCB, this);
  move_dof_to_contacts_srv = nh->advertiseService(
      "moveDOFToContacts", &GraspitInterface::moveDOFToContactsCB, this);
  set_robot_desired_dof_srv = nh->advertiseService(
      "setRobotDesiredDOF", &GraspitInterface::setRobotDesiredDOFCB, this);

  import_robot_srv = nh->advertiseService(
      "importRobot", &GraspitInterface::importRobotCB, this);
  import_obstacle_srv = nh->advertiseService(
      "importObstacle", &GraspitInterface::importObstacleCB, this);
  import_graspable_body_srv = nh->advertiseService(
      "importGraspableBody", &GraspitInterface::importGraspableBodyCB, this);

  // Shape completion stuff
  shape_complete_scene_action_client = new actionlib::SimpleActionClient<
      scene_completion_msgs::CompleteSceneAction>(
      "/scene_completion/SceneCompletion", true);
  plan_x_best_grasps_action_client = new actionlib::SimpleActionClient<
      graspit_interface::PlanXBestGraspsAction>("/plan_best_grasp", true);
  execute_x_best_grasps_action_client = new actionlib::SimpleActionClient<
      graspit_interface::ExecuteXBestGraspsAction>("/execute_best_grasps",
                                                   true);
  simulation_experiment_action_client = new actionlib::SimpleActionClient<
      graspit_interface::SimulationExperimentAction>("/simulation_experiment",
                                                     true);

  meshed_scene_repub =
      nh->advertise<scene_completion_msgs::CompleteSceneResult>(
          "/get_segmented_meshed_scene", 1);
  meshed_scene_sub =
      nh->subscribe("/get_segmented_meshed_scene", 10,
                    &GraspitInterface::getSegmentedMeshesCB, this);
  clear_world_srv =
      nh->advertiseService("clearWorld", &GraspitInterface::clearWorldCB, this);
  load_world_srv =
      nh->advertiseService("loadWorld", &GraspitInterface::loadWorldCB, this);
  save_world_srv =
      nh->advertiseService("saveWorld", &GraspitInterface::saveWorldCB, this);

  save_image_srv =
      nh->advertiseService("saveImage", &GraspitInterface::saveImageCB, this);
  toggle_all_collisions_srv = nh->advertiseService(
      "toggleAllCollisions", &GraspitInterface::toggleAllCollisionsCB, this);

  compute_quality_srv = nh->advertiseService(
      "computeQuality", &GraspitInterface::computeQualityCB, this);
  compute_energy_srv = nh->advertiseService(
      "computeEnergy", &GraspitInterface::computeEnergyCB, this);

  approach_to_contact_srv = nh->advertiseService(
      "approachToContact", &GraspitInterface::approachToContactCB, this);
  find_initial_contact_srv = nh->advertiseService(
      "findInitialContact", &GraspitInterface::findInitialContactCB, this);
  dynamic_auto_grasp_complete_srv =
      nh->advertiseService("dynamicAutoGraspComplete",
                           &GraspitInterface::dynamicAutoGraspCompleteCB, this);

  plan_grasps_action_server =
      new actionlib::SimpleActionServer<graspit_interface::PlanGraspsAction>(
          *nh, "planGrasps",
          boost::bind(&GraspitInterface::PlanGraspsCB, this, _1), false);
  plan_grasps_action_server->start();

  first_time_in_main_loop = true;

  m_planner = NULL;
  m_hand_object_state = NULL;
  clear_world_client =
      nh->serviceClient<graspit_interface::ClearWorld>("clearWorld");
  add_objects_client =
      nh->serviceClient<graspit_interface::ImportGraspableBody>(
          "importGraspableBody");
  get_graspable_bodies_client =
      nh->serviceClient<graspit_interface::GetGraspableBodies>(
          "getGraspableBodies");
  save_world_client =
      nh->serviceClient<graspit_interface::SaveWorld>("saveWorld");
  add_object_client =
      nh->serviceClient<graspit_interface::AddObject>("addObject");
  ROS_INFO("MAKING SHAPE COMPLETION UI");
  QPushButton *shape_complete_scene_button =
      new QPushButton("Scene Completion");
  QPushButton *generate_grasps_button = new QPushButton("Generate Grasps");
  QPushButton *execute_highest_ranked_grasp_button =
      new QPushButton("Execute Best Grasp");
  QPushButton *simulation_experiment_button =
      new QPushButton("Do Simulation Experiment");

  shape_complete_scene_button->setDefault(true);
  generate_grasps_button->setDefault(true);
  execute_highest_ranked_grasp_button->setDefault(true);
  simulation_experiment_button->setDefault(true);

  QWidget *shape_completion_control_box = new QWidget();

  scene_completion_time = new QLabel(tr("Scene Completion Time:"));
  grasp_planning_time = new QLabel(tr("Grasp Planning Time:"));

  QGridLayout *main_layout = new QGridLayout;

  main_layout->addWidget(shape_complete_scene_button, 0, 0);
  main_layout->addWidget(generate_grasps_button, 1, 0);
  main_layout->addWidget(execute_highest_ranked_grasp_button, 2, 0);
  main_layout->addWidget(simulation_experiment_button, 3, 0);
  main_layout->addWidget(scene_completion_time, 4, 0);
  main_layout->addWidget(grasp_planning_time, 5, 0);
  shape_completion_control_box->setLayout(main_layout);

  shape_completion_control_box->show();

  QObject::connect(shape_complete_scene_button, SIGNAL(clicked()), this,
                   SLOT(onShapeCompleteSceneButtonPressed()));
  QObject::connect(generate_grasps_button, SIGNAL(clicked()), this,
                   SLOT(onGenerateGraspsButtonPressed()));
  QObject::connect(execute_highest_ranked_grasp_button, SIGNAL(clicked()), this,
                   SLOT(onExecuteHighestRankedGraspButtonPressed()));
  QObject::connect(simulation_experiment_button, SIGNAL(clicked()), this,
                   SLOT(onSimulationExperimentButtonPressed()));

  ROS_INFO("GraspIt interface successfully initialized!");

  return 0;
}

int GraspitInterface::mainLoop() {
  if (first_time_in_main_loop) {
    // Planner Must be started by mainthread, so it cannot be
    // Started inside the callback for the action server.  I need to connect
    // these here So that when the signal is emitted, the slot function is
    // executed by the correct thread.
    QObject::connect(this, SIGNAL(emitRunPlannerInMainThread()), this,
                     SLOT(runPlannerInMainThread()),
                     Qt::BlockingQueuedConnection);
    QObject::connect(this, SIGNAL(emitProcessPlannerResultsInMainThread()),
                     this, SLOT(processPlannerResultsInMainThread()),
                     Qt::BlockingQueuedConnection);
    QObject::connect(this, SIGNAL(emitBuildFeedbackInMainThread()), this,
                     SLOT(buildFeedbackInMainThread()),
                     Qt::BlockingQueuedConnection);
    first_time_in_main_loop = false;
    ROS_INFO("Planner Signal/Slots connected");
  }

  if (ros::ok()) {
    ros::spinOnce();
  } else {
    ROS_INFO("Exit main loop");
    graspitCore->exitMainLoop();
  }
  return 0;
}

void GraspitInterface::onShapeCompleteSceneButtonPressed() {
  ROS_INFO("onShapeCompleteSceneButtonPressed\n");

  ROS_INFO("Emptying World");
  graspitCore->emptyWorld();

  scene_completion_msgs::CompleteSceneGoal goal;

  ROS_INFO("About to send goal");
  shape_complete_scene_action_client->sendGoal(
      goal, boost::bind(&GraspitInterface::receivedMeshedSceneCB, this, _1, _2),
      actionlib::SimpleActionClient<
          scene_completion_msgs::CompleteSceneAction>::SimpleActiveCallback(),
      actionlib::SimpleActionClient<
          scene_completion_msgs::CompleteSceneAction>::
          SimpleFeedbackCallback());
}

void GraspitInterface::getSegmentedMeshesCB(
    const scene_completion_msgs::CompleteSceneResultConstPtr &result) {

  ROS_INFO("Entering getSegmentedMeshesCB");
  if (QThread::currentThread() != QApplication::instance()->thread()) {
    meshed_scene_repub.publish(result);
  } else {
    ROS_INFO("Received %lu mean meshes and %lu sampled meshes",
             result->mean_meshes.size(), result->sample_meshes.size());
    ROS_INFO("Adding only mean meshes to the scene");
    for (int i = 0; i < result->mean_meshes.size(); i++) {
      int new_mesh_index = graspitCore->getWorld()->getNumGB();
      QString mesh_idx_1 = QString::number(new_mesh_index);
      geometry_msgs::Vector3 offset;
      offset.x = result->poses.at(i).pose.position.x;
      offset.y = result->poses.at(i).pose.position.y;
      offset.z = result->poses.at(i).pose.position.z;
      mesh_offset.x = offset.x;
      mesh_offset.y = offset.y;
      mesh_offset.z = offset.z;

      addMesh(mesh_idx_1, result->mean_meshes.at(i), offset);
      for (int j = 0; j < result->sample_meshes.size(); j++) {
        addMesh(mesh_idx_1 + QString::number(j), result->sample_meshes.at(j),
                offset, false);
      }
    }
  }

  ROS_INFO("Successfully received meshed scene");
}

void GraspitInterface::receivedMeshedSceneCB(
    const actionlib::SimpleClientGoalState &state,
    const scene_completion_msgs::CompleteSceneResultConstPtr &result)
// const graspit_interface::GetSegmentedMeshedSceneResultConstPtr& result)
{
  ROS_INFO("Sucessfully recieved meshed scene");
  getSegmentedMeshesCB(result);

  // TODO: Return the time it takes to complete the mesh
  // scene_segmentation_time->setText(QString("Scene Segmentation Time (ms): ")
  //+ QString::number(result->completion_time));
}

void GraspitInterface::onGenerateGraspsButtonPressed() {
  ROS_INFO("onGenerateGraspsButtonPressed\n");
  ROS_INFO("Generate grasps with the %s planner and returning the %d best "
           "grasps",
           grasp_planning_method.c_str(), number_of_best_grasps_to_return);

  graspit_interface::PlanXBestGraspsGoal goal;
  goal.filename = mean_mesh_file_path;
  goal.mesh_offset = mesh_offset;
  goal.rank_grasps_on_samples = rank_grasps_on_samples;
  goal.number_of_top_x_grasps_to_return = number_of_best_grasps_to_return;
  goal.planner = grasp_planning_method;
  ROS_INFO("About to send goal");
  plan_x_best_grasps_action_client->sendGoal(
      goal, boost::bind(&GraspitInterface::planXBestGraspsCB, this, _1, _2),
      actionlib::SimpleActionClient<
          graspit_interface::PlanXBestGraspsAction>::SimpleActiveCallback(),
      actionlib::SimpleActionClient<
          graspit_interface::PlanXBestGraspsAction>::SimpleFeedbackCallback());
}

void GraspitInterface::planXBestGraspsCB(
    const actionlib::SimpleClientGoalState &state,
    const graspit_interface::PlanXBestGraspsResultConstPtr &result) {
  ROS_INFO("Entering planBestGraspCB");
  ROS_INFO("%d", result->success);
  indices_of_best_epsilon_grasps = result->top_x_epsilon_quality_grasps;
  indices_of_best_volume_grasps = result->top_x_volume_quality_grasps;
  gripper_pose_for_all_grasps = result->hand_poses;
  gripper_joint_state_for_all_grasps = result->hand_joint_states;
}

void GraspitInterface::onSimulationExperimentButtonPressed() {
  ROS_INFO("onSimulationExperimentButtonPressed\n");

  QWidget *simulation_experiment_control_box = new QWidget();

  QGridLayout *main_layout = new QGridLayout;

  QRadioButton *evaluate_on_shape_samples_radio_button =
      new QRadioButton(tr("Evaluate on shape samples"));
  QPushButton *ground_truth_meshes_button =
      new QPushButton(tr("&Folder containing ground truth meshes"));
  QPushButton *shape_completed_meshes_button =
      new QPushButton(tr("&Folder containing shape completed meshes"));
  QPushButton *storage_folder_button =
      new QPushButton(tr("&Folder where to save the results"));
  QPushButton *run_button = new QPushButton(tr("&Run"));

  QSignalMapper *signal_mapper = new QSignalMapper(this);
  signal_mapper->setMapping(ground_truth_meshes_button, 0);
  signal_mapper->setMapping(shape_completed_meshes_button, 1);
  signal_mapper->setMapping(storage_folder_button, 2);

  QObject::connect(ground_truth_meshes_button, SIGNAL(clicked()), signal_mapper,
                   SLOT(map()));
  QObject::connect(shape_completed_meshes_button, SIGNAL(clicked()),
                   signal_mapper, SLOT(map()));
  QObject::connect(storage_folder_button, SIGNAL(clicked()), signal_mapper,
                   SLOT(map()));
  QObject::connect(signal_mapper, SIGNAL(mapped(int)), this,
                   SLOT(onBrowseButtonPressed(int)));
  QObject::connect(evaluate_on_shape_samples_radio_button, SIGNAL(clicked()),
                   this, SLOT(onEvaluateOnShapeSamplesRadioButtonPressed()));
  QObject::connect(run_button, SIGNAL(clicked()), this,
                   SLOT(onRunButtonPressed()));

  evaluate_on_shape_samples_radio_button->setChecked(false);
  main_layout->addWidget(evaluate_on_shape_samples_radio_button);
  main_layout->addWidget(ground_truth_meshes_button);
  main_layout->addWidget(shape_completed_meshes_button);
  main_layout->addWidget(storage_folder_button);
  main_layout->addWidget(run_button);
  simulation_experiment_control_box->setLayout(main_layout);

  simulation_experiment_control_box->show();
}

void GraspitInterface::onEvaluateOnShapeSamplesRadioButtonPressed() {
  evaluate_on_shape_samples = !evaluate_on_shape_samples;
}

void GraspitInterface::onRunButtonPressed() {
  graspit_interface::SimulationExperimentGoal goal;
  goal.path_to_ground_truth_meshes = file_path_to_ground_truth_meshes;
  goal.path_to_shape_completed_meshes = file_path_to_shape_completed_meshes;
  goal.path_to_save_location = file_path_to_save_location;
  goal.evaluate_on_shape_samples = evaluate_on_shape_samples;
  ROS_INFO("About to send goal");
  simulation_experiment_action_client->sendGoal(
      goal,
      boost::bind(&GraspitInterface::simulationExperimentCB, this, _1, _2),
      actionlib::SimpleActionClient<
          graspit_interface::SimulationExperimentAction>::
          SimpleActiveCallback(),
      actionlib::SimpleActionClient<
          graspit_interface::SimulationExperimentAction>::
          SimpleFeedbackCallback());
}

void GraspitInterface::simulationExperimentCB(
    const actionlib::SimpleClientGoalState &state,
    const graspit_interface::SimulationExperimentResultConstPtr &result) {
  ROS_INFO("Entering planBestGraspCB");
  ROS_INFO("%d", result->success);
}

void GraspitInterface::onBrowseButtonPressed(int id) {
  QString directory = QDir::toNativeSeparators(
      QFileDialog::getExistingDirectory(0, tr("Find Files"), "~"));
  if (id == 0)
    file_path_to_ground_truth_meshes = directory.toStdString();
  else if (id == 1)
    file_path_to_shape_completed_meshes = directory.toStdString();
  else if (id == 2)
    file_path_to_save_location = directory.toStdString();
}

void GraspitInterface::onExecuteHighestRankedGraspButtonPressed() {
  ROS_INFO("onExecuteHighestRankedGraspButtonPressed\n");
  graspit_interface::ExecuteXBestGraspsGoal goal;
  goal.indices_of_best_epsilon_grasps = indices_of_best_epsilon_grasps;
  goal.indices_of_best_volume_grasps = indices_of_best_volume_grasps;
  goal.gripper_pose_for_all_grasps = gripper_pose_for_all_grasps;
  goal.gripper_joint_state_for_all_grasps = gripper_joint_state_for_all_grasps;

  ROS_INFO(
      "Calling the execute_best_grasp service which executes the best grasp on "
      "the robot. This service needs to be implemented by the user");
  ROS_INFO("About to send goal");
  execute_x_best_grasps_action_client->sendGoal(
      goal, boost::bind(&GraspitInterface::executeXBestGraspsCB, this, _1, _2),
      actionlib::SimpleActionClient<
          graspit_interface::ExecuteXBestGraspsAction>::SimpleActiveCallback(),
      boost::bind(&GraspitInterface::executeXBestGraspsFeedback, this, _1));
}

void GraspitInterface::executeXBestGraspsCB(
    const actionlib::SimpleClientGoalState &state,
    const graspit_interface::ExecuteXBestGraspsResultConstPtr &result) {
  ROS_INFO("Entering ExecuteXBestGraspsCB");
  ROS_INFO("%d", result->success);
  visualizeGrasp(gripper_joint_state_for_all_grasps[result->final_grasp_index],
                 gripper_pose_for_all_grasps[result->final_grasp_index]);
}

void GraspitInterface::executeXBestGraspsFeedback(
    const graspit_interface::ExecuteXBestGraspsFeedbackConstPtr &feedback) {
  ROS_INFO("Entering executeXBestGraspsFeedback");
  int current_grasp_index = feedback->current_grasp;
  visualizeGrasp(gripper_joint_state_for_all_grasps[current_grasp_index],
                 gripper_pose_for_all_grasps[current_grasp_index]);
}

void GraspitInterface::visualizeGrasp(
    const sensor_msgs::JointState &gripper_joints,
    const geometry_msgs::Pose &gripper_pose) {
  ROS_INFO("Visualizing grasps on mean mesh");
  // TODO: visualize the grasp
}

void GraspitInterface::addObject(graspit_msgs::ObjectInfo object) {
  ROS_INFO("Entering addObject function");
  QString model_name(QString::fromStdString(object.model_name));
  QString object_name(QString::fromStdString(object.object_name));
  transf object_pose = transf(Quaternion(object.object_pose.orientation.w,
                                         object.object_pose.orientation.x,
                                         object.object_pose.orientation.y,
                                         object.object_pose.orientation.z),
                              vec3(object.object_pose.position.x * AXIS_SCALE,
                                   object.object_pose.position.y * AXIS_SCALE,
                                   object.object_pose.position.z * AXIS_SCALE));
  ROS_INFO("Adding Model %s", model_name.toStdString().c_str());
  ROS_INFO("Adding Model %s", object_name.toStdString().c_str());
  addToWorld(model_name, object_name, object_pose);
}

void GraspitInterface::addToWorld(const QString model_name,
                                  const QString object_name,
                                  const transf object_pose) {
  QString model_filename = model_name + QString(".xml");
  ROS_INFO("model filename: %s", model_filename.toStdString().c_str());

  QString body_file =
      QString(getenv("GRASPIT")) + "/" + "models/objects/" + model_filename;
  Body *b = graspitCore->getWorld()->importBody("GraspableBody", body_file);
  if (!b) {
    QString body_file = QString(getenv("GRASPIT")) + "/" +
                        "models/object_database/" + model_filename;
    b = graspitCore->getWorld()->importBody("GraspableBody", body_file);
  }
  if (b) {
    b->setTran(object_pose);
    b->setName(object_name);
    b->setObjectName(object_name);
  }
}

bool GraspitInterface::addObjectCB(
    graspit_interface::AddObject::Request &request,
    graspit_interface::AddObject::Response &response) {

  ROS_INFO("Adding objects");
  for (int i = 0; i < request.meshes.size(); i++) {
    int new_mesh_index = graspitCore->getWorld()->getNumGB();
    geometry_msgs::Vector3 offset;
    offset.x = request.poses.at(i).pose.position.x;
    offset.y = request.poses.at(i).pose.position.y;
    offset.z = request.poses.at(i).pose.position.z;
    addMesh(QString::number(new_mesh_index), request.meshes.at(i), offset);
  }
  response.result = response.RESULT_SUCCESS;
  return true;
}
void GraspitInterface::addMesh(QString mesh_index, shape_msgs::Mesh mesh,
                               geometry_msgs::Vector3 offset,
                               const bool is_mean) {
  std::vector<int> triangles;
  std::vector<position> vertices;
  pcl::PolygonMesh *pcl_mesh = new pcl::PolygonMesh;
  meshMsgToVerticesTriangles(mesh, &vertices, &triangles, *pcl_mesh);

  QString full_file_path =
      QString(getenv("GRASPIT")) + QString("/models/objects/");
  if (!is_mean) {
    QString mesh_name = QString("mesh_sample_") + mesh_index;
    ROS_INFO("This sampled mesh is only saved to an xml file.\n");
    saveMesh(full_file_path, mesh_name, pcl_mesh);
  } else {
    GraspableBody *b = new GraspableBody(graspitCore->getWorld());
    if (b->loadGeometryMemory(vertices, triangles) == SUCCESS) {
      ROS_INFO("setting mesh name\n");
      QString mesh_name = QString("mesh_mean_") + mesh_index;
      b->setName(mesh_name);

      ROS_INFO("About to add IVMAT\n");
      b->addIVMat(true);

      ROS_INFO("About to add to IVC\n");
      b->addToIvc();
      SoSeparator *IVGeomRoot = b->getIVGeomRoot();
      SoTransform *IVScaleTran = new SoTransform;

      IVScaleTran->scaleFactor.setValue(AXIS_SCALE, AXIS_SCALE, AXIS_SCALE);
      IVGeomRoot->insertChild(IVScaleTran, 0);

      ROS_INFO("Adding mean mesh to main world\n");
      graspitCore->getWorld()->addBody(b);
      ROS_INFO("Added Mesh: %s", b->getName().toStdString().c_str());

      b->setFilename(QString("models/objects/") + b->getName() + ".xml");

      saveMesh(full_file_path, b->getName(), pcl_mesh, true);
    } else {
      ROS_INFO("Failed to load mesh geometry; Mesh: %s",
               b->getName().toStdString().c_str());
    }
  }
}

void GraspitInterface::saveMesh(const QString file_path, const QString filename,
                                const pcl::PolygonMesh *pcl_mesh,
                                const bool is_mean) {

  QString mesh_filename = filename + QString(".ply");
  QString mesh_full_file_path = file_path + mesh_filename;
  if (is_mean) {
    mean_mesh_file_path = (file_path + filename).toStdString();
  }

  ROS_INFO("Saving in mesh file: %s",
           mesh_full_file_path.toStdString().c_str());

  pcl::io::savePLYFileBinary(mesh_full_file_path.toStdString().c_str(),
                             *pcl_mesh);

  QString xml_full_file_path = file_path + filename + QString(".xml");
  ROS_INFO("Saving in xml file: %s", xml_full_file_path.toStdString().c_str());
  ROS_INFO("saving .xml");
  QFile file(xml_full_file_path);
  file.open(QIODevice::WriteOnly | QIODevice::Text);
  ROS_INFO("opened .xml");
  QTextStream stream(&file);
  ROS_INFO("about to stream .xml");

  stream << "<?xml version=\"1.0\"?>\n";
  stream << "<root>\n";

  stream << "	<geometryScaling>1000</geometryScaling> \n";
  stream << "	<geometryFile type=\"ply\">";
  stream << mesh_filename.toStdString().c_str();
  stream << "</geometryFile>\n";
  stream << "</root>";

  ROS_INFO("wrote .xml");
  file.close();
  ROS_INFO("close .xml");
}

bool GraspitInterface::getRobotCB(
    graspit_interface::GetRobot::Request &request,
    graspit_interface::GetRobot::Response &response) {
  if (graspitCore->getWorld()->getNumRobots() <= request.id) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  } else {
    Robot *r = graspitCore->getWorld()->getRobot(request.id);
    transf t = r->getTran();

    geometry_msgs::Pose pose = geometry_msgs::Pose();

    pose.position.x = t.translation().x() / AXIS_SCALE;
    pose.position.y = t.translation().y() / AXIS_SCALE;
    ;
    pose.position.z = t.translation().z() / AXIS_SCALE;
    ;
    pose.orientation.w = t.rotation().w();
    pose.orientation.x = t.rotation().x();
    pose.orientation.y = t.rotation().y();
    pose.orientation.z = t.rotation().z();

    response.robot.pose = pose;

    // Info for all contacts with this robot:
    std::list<Contact *> contacts = r->getContacts();
    for (std::list<Contact *>::iterator it = contacts.begin();
         it != contacts.end(); it++) {
      graspit_interface::Contact c;
      c.body1 = (*it)->getBody1()->getName().toStdString();
      c.body2 = (*it)->getBody2()->getName().toStdString();

      transf contactInWorldFrame = (*it)->getBody1Tran() % (*it)->getFrame();
      c.ps.header.frame_id = "world";
      c.ps.pose.position.x = contactInWorldFrame.translation().x() / AXIS_SCALE;
      c.ps.pose.position.y = contactInWorldFrame.translation().y() / AXIS_SCALE;
      c.ps.pose.position.z = contactInWorldFrame.translation().z() / AXIS_SCALE;
      c.ps.pose.orientation.w = contactInWorldFrame.rotation().w();
      c.ps.pose.orientation.x = contactInWorldFrame.rotation().x();
      c.ps.pose.orientation.y = contactInWorldFrame.rotation().y();
      c.ps.pose.orientation.z = contactInWorldFrame.rotation().z();
      c.cof = (*it)->getCof();
      response.robot.contacts.push_back(c);
    }

    double *joint_vals = new double[r->getNumJoints()];
    r->getJointValues(joint_vals);

    sensor_msgs::JointState robot_joint_state = sensor_msgs::JointState();
    for (int i = 0; i < r->getNumJoints(); i++) {

      std::string joint_name;
      std::ostringstream convert;
      convert << i;
      joint_name = convert.str();

      robot_joint_state.name.push_back(joint_name);
      robot_joint_state.position.push_back(joint_vals[i]);
      robot_joint_state.velocity.push_back(0);
      robot_joint_state.effort.push_back(0);
    }
    response.robot.joints.push_back(robot_joint_state);

    for (int i = 0; i < r->getNumDOF(); i++) {
      response.robot.dofs.push_back(r->getDOF(i)->getVal());
    }

    for (int i = 0; i < r->getNumSensors(); i++) {

      BodySensor *s = r->getSensor(i);
      transf t = s->getSensorTran();

      geometry_msgs::PoseStamped pose_stamped = geometry_msgs::PoseStamped();

      pose_stamped.header.frame_id = "world";
      pose_stamped.pose.position.x = t.translation().x() / AXIS_SCALE;
      pose_stamped.pose.position.y = t.translation().y() / AXIS_SCALE;
      pose_stamped.pose.position.z = t.translation().z() / AXIS_SCALE;
      pose_stamped.pose.orientation.w = t.rotation().w();
      pose_stamped.pose.orientation.x = t.rotation().x();
      pose_stamped.pose.orientation.y = t.rotation().y();
      pose_stamped.pose.orientation.z = t.rotation().z();

      response.robot.tactile.sensor_poses.push_back(pose_stamped);
      response.robot.tactile.sensor_forces.push_back(s->getNormalForce());
    }

    return true;
  }
  return true;
}

bool GraspitInterface::getGraspableBodyCB(
    graspit_interface::GetGraspableBody::Request &request,
    graspit_interface::GetGraspableBody::Response &response) {
  if (graspitCore->getWorld()->getNumGB() <= request.id) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  } else {
    GraspableBody *b = graspitCore->getWorld()->getGB(request.id);
    transf t = b->getTran();
    geometry_msgs::Pose pose = geometry_msgs::Pose();

    pose.position.x = t.translation().x() / AXIS_SCALE;
    pose.position.y = t.translation().y() / AXIS_SCALE;
    ;
    pose.position.z = t.translation().z() / AXIS_SCALE;
    ;
    pose.orientation.w = t.rotation().w();
    pose.orientation.x = t.rotation().x();
    pose.orientation.y = t.rotation().y();
    pose.orientation.z = t.rotation().z();

    response.graspable_body.pose = pose;
    return true;
  }
  return true;
}

bool GraspitInterface::getBodyCB(
    graspit_interface::GetBody::Request &request,
    graspit_interface::GetBody::Response &response) {
  if (graspitCore->getWorld()->getNumBodies() <= request.id) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  } else {
    Body *b = graspitCore->getWorld()->getBody(request.id);
    transf t = b->getTran();

    geometry_msgs::Pose pose = geometry_msgs::Pose();

    pose.position.x = t.translation().x() / AXIS_SCALE;
    pose.position.y = t.translation().y() / AXIS_SCALE;
    pose.position.z = t.translation().z() / AXIS_SCALE;
    pose.orientation.w = t.rotation().w();
    pose.orientation.x = t.rotation().x();
    pose.orientation.y = t.rotation().y();
    pose.orientation.z = t.rotation().z();

    response.body.pose = pose;

    return true;
  }
  return true;
}

bool GraspitInterface::getRobotsCB(
    graspit_interface::GetRobots::Request &request,
    graspit_interface::GetRobots::Response &response) {
  for (int i = 0; i < graspitCore->getWorld()->getNumRobots(); i++) {
    response.ids.push_back(i);
  }
  return true;
}

bool GraspitInterface::getGraspableBodiesCB(
    graspit_interface::GetGraspableBodies::Request &request,
    graspit_interface::GetGraspableBodies::Response &response) {
  for (int i = 0; i < graspitCore->getWorld()->getNumGB(); i++) {
    response.ids.push_back(i);
  }
  return true;
}

bool GraspitInterface::getBodiesCB(
    graspit_interface::GetBodies::Request &request,
    graspit_interface::GetBodies::Response &response) {
  for (int i = 0; i < graspitCore->getWorld()->getNumBodies(); i++) {
    response.ids.push_back(i);
  }
  return true;
}

bool GraspitInterface::setRobotPoseCB(
    graspit_interface::SetRobotPose::Request &request,
    graspit_interface::SetRobotPose::Response &response) {
  if (graspitCore->getWorld()->getNumRobots() <= request.id) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  } else {
    vec3 new_translation(request.pose.position.x * AXIS_SCALE,
                         request.pose.position.y * AXIS_SCALE,
                         request.pose.position.z * AXIS_SCALE);

    Quaternion new_rotation(
        request.pose.orientation.w, request.pose.orientation.x,
        request.pose.orientation.y, request.pose.orientation.z);

    transf new_transform(new_rotation, new_translation);

    graspitCore->getWorld()->getRobot(request.id)->setTran(new_transform);
    return true;
  }
}

bool GraspitInterface::setGraspableBodyPoseCB(
    graspit_interface::SetGraspableBodyPose::Request &request,
    graspit_interface::SetGraspableBodyPose::Response &response) {
  if (graspitCore->getWorld()->getNumGB() <= request.id) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  } else {

    vec3 new_translation(request.pose.position.x * AXIS_SCALE,
                         request.pose.position.y * AXIS_SCALE,
                         request.pose.position.z * AXIS_SCALE);

    Quaternion new_rotation(
        request.pose.orientation.w, request.pose.orientation.x,
        request.pose.orientation.y, request.pose.orientation.z);

    transf new_transform(new_rotation, new_translation);

    graspitCore->getWorld()->getGB(request.id)->setTran(new_transform);
    return true;
  }
}

bool GraspitInterface::setBodyPoseCB(
    graspit_interface::SetBodyPose::Request &request,
    graspit_interface::SetBodyPose::Response &response) {
  if (graspitCore->getWorld()->getNumBodies() <= request.id) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  } else {

    vec3 new_translation(request.pose.position.x * AXIS_SCALE,
                         request.pose.position.y * AXIS_SCALE,
                         request.pose.position.z * AXIS_SCALE);

    Quaternion new_rotation(
        request.pose.orientation.w, request.pose.orientation.x,
        request.pose.orientation.y, request.pose.orientation.z);

    transf new_transform(new_rotation, new_translation);

    graspitCore->getWorld()->getBody(request.id)->setTran(new_transform);
    return true;
  }
}

bool GraspitInterface::getDynamicsCB(
    graspit_interface::GetDynamics::Request &request,
    graspit_interface::GetDynamics::Response &response) {
  response.dynamicsEnabled = graspitCore->getWorld()->dynamicsAreOn();
  return true;
}

bool GraspitInterface::setDynamicsCB(
    graspit_interface::SetDynamics::Request &request,
    graspit_interface::SetDynamics::Response &response) {
  if (request.enableDynamics && (!graspitCore->getWorld()->dynamicsAreOn())) {
    graspitCore->getWorld()->turnOnDynamics();
    ROS_INFO("Turning Dynamics On");
  } else if ((!request.enableDynamics) &&
             graspitCore->getWorld()->dynamicsAreOn()) {
    graspitCore->getWorld()->turnOffDynamics();
    ROS_INFO("Turning Dynamics Off");
  }
  return true;
}

bool GraspitInterface::autoGraspCB(
    graspit_interface::AutoGrasp::Request &request,
    graspit_interface::AutoGrasp::Response &response) {
  if (graspitCore->getWorld()->getNumRobots() <= request.id) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  } else {
    graspitCore->getWorld()
        ->getHand(request.id)
        ->autoGrasp(render_graphics, 1.0, false);
    graspitCore->getWorld()->updateGrasps();
  }
  return true;
}

bool GraspitInterface::autoOpenCB(
    graspit_interface::AutoOpen::Request &request,
    graspit_interface::AutoOpen::Response &response) {
  if (graspitCore->getWorld()->getNumRobots() <= request.id) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  } else {
    graspitCore->getWorld()
        ->getHand(request.id)
        ->autoGrasp(render_graphics, -1.0, false);
    graspitCore->getWorld()->updateGrasps();
  }
  return true;
}

bool GraspitInterface::forceRobotDOFCB(
    graspit_interface::ForceRobotDOF::Request &request,
    graspit_interface::ForceRobotDOF::Response &response) {
  if (graspitCore->getWorld()->getNumRobots() <= request.id) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  } else if (graspitCore->getWorld()->dynamicsAreOn()) {
    response.result = response.RESULT_DYNAMICS_MODE_ENABLED;
    return true;
  } else {
    graspitCore->getWorld()
        ->getHand(request.id)
        ->forceDOFVals(request.dofs.data());
    response.result = response.RESULT_SUCCESS;
    return true;
  }
}

bool GraspitInterface::moveDOFToContactsCB(
    graspit_interface::MoveDOFToContacts::Request &request,
    graspit_interface::MoveDOFToContacts::Response &response) {
  if (graspitCore->getWorld()->getNumRobots() <= request.id) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  } else if (graspitCore->getWorld()->dynamicsAreOn()) {
    response.result = response.RESULT_DYNAMICS_MODE_ENABLED;
    return true;
  } else {
    graspitCore->getWorld()
        ->getHand(request.id)
        ->moveDOFToContacts(request.dofs.data(), request.desired_steps.data(),
                            request.stopAtContact, render_graphics);
    response.result = response.RESULT_SUCCESS;
    return true;
  }
}

bool GraspitInterface::setRobotDesiredDOFCB(
    graspit_interface::SetRobotDesiredDOF::Request &request,
    graspit_interface::SetRobotDesiredDOF::Response &response) {
  if (graspitCore->getWorld()->getNumRobots() <= request.id) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  } else if (!graspitCore->getWorld()->dynamicsAreOn()) {
    response.result = response.RESULT_DYNAMICS_MODE_DISABLED;
    return true;
  } else {
    for (int i = 0;
         i < graspitCore->getWorld()->getHand(request.id)->getNumDOF(); i++) {
      graspitCore->getWorld()
          ->getHand(request.id)
          ->getDOF(i)
          ->setDesiredVelocity(request.dof_velocities.data()[i]);
    }
    graspitCore->getWorld()
        ->getHand(request.id)
        ->setDesiredDOFVals(request.dofs.data());
    response.result = response.RESULT_SUCCESS;
    return true;
  }
}

bool GraspitInterface::importRobotCB(
    graspit_interface::ImportRobot::Request &request,
    graspit_interface::ImportRobot::Response &response) {
  QString filename = QString(getenv("GRASPIT")) + QString("/models/robots/") +
                     QString(request.filename.data()) + QString("/") +
                     QString(request.filename.data()) + QString(".xml");

  ROS_INFO("Loading %s", filename.toStdString().c_str());

  Robot *r = graspitCore->getWorld()->importRobot(filename);
  if (r == NULL) {
    response.result = response.RESULT_FAILURE;
    return true;
  }
  vec3 new_translation(request.pose.position.x * AXIS_SCALE,
                       request.pose.position.y * AXIS_SCALE,
                       request.pose.position.z * AXIS_SCALE);

  Quaternion new_rotation(
      request.pose.orientation.w, request.pose.orientation.x,
      request.pose.orientation.y, request.pose.orientation.z);

  transf new_transform(new_rotation, new_translation);
  r->setTran(new_transform);
  r->setRenderGeometry(render_graphics);
  return true;
}

bool GraspitInterface::importObstacleCB(
    graspit_interface::ImportObstacle::Request &request,
    graspit_interface::ImportObstacle::Response &response) {
  QString filename = QString(getenv("GRASPIT")) +
                     QString("/models/obstacles/") +
                     QString(request.filename.data()) + QString(".xml");

  ROS_INFO("Loading %s", filename.toStdString().c_str());

  Body *b = graspitCore->getWorld()->importBody(QString("Body"), filename);
  if (b == NULL) {
    // Now try to load using unaltered file_path from request.
    b = graspitCore->getWorld()->importBody(QString("Body"),
                                            QString(request.filename.data()));
    if (b == NULL) {
      response.result = response.RESULT_FAILURE;
      return true;
    }
  }

  vec3 new_translation(request.pose.position.x * AXIS_SCALE,
                       request.pose.position.y * AXIS_SCALE,
                       request.pose.position.z * AXIS_SCALE);

  Quaternion new_rotation(
      request.pose.orientation.w, request.pose.orientation.x,
      request.pose.orientation.y, request.pose.orientation.z);

  transf new_transform(new_rotation, new_translation);
  b->setTran(new_transform);
  return true;
}

bool GraspitInterface::importGraspableBodyCB(
    graspit_interface::ImportGraspableBody::Request &request,
    graspit_interface::ImportGraspableBody::Response &response) {
  QString filename = QString(getenv("GRASPIT")) + QString("/models/objects/") +
                     QString(request.filename.data()) + QString(".xml");

  ROS_INFO("Loading %s", filename.toStdString().c_str());
  // First try to load from Graspit Directory
  Body *b =
      graspitCore->getWorld()->importBody(QString("GraspableBody"), filename);
  if (b == NULL) {
    // Now try to load using unaltered file_path from request.
    b = graspitCore->getWorld()->importBody(QString("GraspableBody"),
                                            QString(request.filename.data()));
    if (b == NULL) {
      response.result = response.RESULT_FAILURE;
      return true;
    }
  }

  vec3 new_translation(request.pose.position.x * AXIS_SCALE,
                       request.pose.position.y * AXIS_SCALE,
                       request.pose.position.z * AXIS_SCALE);

  Quaternion new_rotation(
      request.pose.orientation.w, request.pose.orientation.x,
      request.pose.orientation.y, request.pose.orientation.z);

  transf new_transform(new_rotation, new_translation);
  b->setTran(new_transform);

  return true;
}

bool GraspitInterface::loadWorldCB(
    graspit_interface::LoadWorld::Request &request,
    graspit_interface::LoadWorld::Response &response) {
  QString filename = QString(getenv("GRASPIT")) + QString("/worlds/") +
                     QString(request.filename.data()) + QString(".xml");

  ROS_INFO("Loading World: %s", filename.toStdString().c_str());
  int result = graspitCore->getWorld()->load(filename);
  if (result == FAILURE) {
    response.result = response.RESULT_FAILURE;
    return true;
  }
  return true;
}

bool GraspitInterface::saveWorldCB(
    graspit_interface::SaveWorld::Request &request,
    graspit_interface::SaveWorld::Response &response) {
  QString filename = QString(getenv("GRASPIT")) + QString("/worlds/") +
                     QString(request.filename.data()) + QString(".xml");

  ROS_INFO("Saving World: %s", filename.toStdString().c_str());
  int result = graspitCore->getWorld()->save(filename);
  if (result == FAILURE) {
    response.result = response.RESULT_FAILURE;
    return true;
  }
  return true;
}

bool GraspitInterface::clearWorldCB(
    graspit_interface::ClearWorld::Request &request,
    graspit_interface::ClearWorld::Response &response) {
  ROS_INFO("Emptying World");
  graspitCore->emptyWorld();

  return true;
}

bool GraspitInterface::saveImageCB(
    graspit_interface::SaveImage::Request &request,
    graspit_interface::SaveImage::Response &response) {
  QString filename = QString(getenv("GRASPIT")) + QString("/images/") +
                     QString(request.filename.data()) + QString(".jpg");

  ROS_INFO("Saving Image: %s", filename.toStdString().c_str());
  graspitCore->getIVmgr()->saveImage(filename);
  return true;
}

bool GraspitInterface::toggleAllCollisionsCB(
    graspit_interface::ToggleAllCollisions::Request &request,
    graspit_interface::ToggleAllCollisions::Response &response) {
  graspitCore->getWorld()->toggleAllCollisions(request.enableCollisions);
  // if(request.enableCollisions)
  //{
  //    ROS_INFO("Collision Detection is On, objects cannot interpentrate");
  //}
  // else
  //{
  //    ROS_INFO("Collision Detection is Off, objects can interpentrate");
  //}
  return true;
}

bool GraspitInterface::computeQualityCB(
    graspit_interface::ComputeQuality::Request &request,
    graspit_interface::ComputeQuality::Response &response) {
  CollisionReport col_report;
  // first test whether the hand is in collision now
  int num_cols = graspitCore->getWorld()->getCollisionReport(&col_report);
  // if it is in collision, then there should be no reason to calculate the
  // quality
  if (num_cols > 0) {
    response.result = response.RESULT_COLLISION;
    response.epsilon = -1.0;
    response.volume = -1.0;
    return true;
  }
  Hand *m_hand = graspitCore->getWorld()->getHand(request.id);
  if (m_hand == NULL) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  }

  // if there is no collision, then begin computation

  QualVolume m_vol_qual(m_hand->getGrasp(), ("Volume"), "L1 Norm");
  QualEpsilon m_eps_qual(m_hand->getGrasp(), ("Epsilon"), "L1 Norm");

  graspitCore->getWorld()->findAllContacts();
  graspitCore->getWorld()->updateGrasps();

  response.epsilon = m_eps_qual.evaluate();
  response.volume = m_vol_qual.evaluate();

  return true;
}

bool GraspitInterface::computeEnergyCB(
    graspit_interface::ComputeEnergy::Request &request,
    graspit_interface::ComputeEnergy::Response &response) {
  Hand *m_hand = graspitCore->getWorld()->getHand(request.handId);
  if (m_hand == NULL) {
    response.result = response.RESULT_INVALID_HAND_ID;
    ROS_INFO("Planning Hand is NULL");
    return true;
  }
  GraspableBody *m_object =
      graspitCore->getWorld()->getGB(request.graspableBodyId);
  if (m_object == NULL) {
    ROS_INFO("Planning Object is NULL");
    response.result = response.RESULT_INVALID_BODY_ID;
    return true;
  }

  graspitCore->getWorld()->findAllContacts();
  graspitCore->getWorld()->updateGrasps();

  std::vector<std::string> energy_types =
      SearchEnergyFactory::getInstance()->getAllRegisteredEnergy();
  if (std::find(energy_types.begin(), energy_types.end(), request.energyType) ==
      energy_types.end()) {
    ROS_INFO_STREAM("Invalid Energy Type " << request.energyType << std::endl);
    response.result = response.RESULT_INVALID_ENERGY_TYPE;
    return true;
  }

  SearchEnergy *se =
      SearchEnergyFactory::getInstance()->createEnergy(request.energyType);

  bool is_legal;
  double state_energy;
  se->analyzeCurrentPosture(m_hand, m_object, is_legal, state_energy);
  response.isLegal = is_legal;
  response.energy = state_energy;

  return true;
}

bool GraspitInterface::approachToContactCB(
    graspit_interface::ApproachToContact::Request &request,
    graspit_interface::ApproachToContact::Response &response) {
  Hand *m_hand = graspitCore->getWorld()->getHand(request.id);
  if (m_hand == NULL) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  }
  m_hand->approachToContact(request.moveDist, request.oneStep);
  return true;
}

bool GraspitInterface::findInitialContactCB(
    graspit_interface::FindInitialContact::Request &request,
    graspit_interface::FindInitialContact::Response &response) {
  Hand *m_hand = graspitCore->getWorld()->getHand(request.id);
  if (m_hand == NULL) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  }

  m_hand->findInitialContact(request.moveDist);
  return true;
}

bool GraspitInterface::dynamicAutoGraspCompleteCB(
    graspit_interface::DynamicAutoGraspComplete::Request &request,
    graspit_interface::DynamicAutoGraspComplete::Response &response) {
  Hand *m_hand = graspitCore->getWorld()->getCurrentHand();
  if (m_hand == NULL) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  }
  response.GraspComplete = m_hand->dynamicAutograspComplete();
  return true;
}

void GraspitInterface::PlanGraspsCB(
    const graspit_interface::PlanGraspsGoalConstPtr &_goal) {
  goal = *_goal;
  ROS_INFO("About to Call emit runPlannerInMainLoop();");
  emit emitRunPlannerInMainThread();

  ROS_INFO("Waiting For Planner to Finish");
  int last_feedback_step;
  while (m_planner->isActive()) {
    if (_goal->feedback_num_steps < 1) {
      continue;
    }
    int current_feedback_step = m_planner->getCurrentStep();
    if (current_feedback_step == m_planner->getStartingStep()) {
      continue;
    }
    if ((current_feedback_step % _goal->feedback_num_steps == 0) &&
        (current_feedback_step != last_feedback_step)) {
      ROS_INFO("Curret Planner Step: %d", m_planner->getCurrentStep());
      ROS_INFO("Curret Num Grasps: %d", m_planner->getListSize());

      // collect grasps
      emit emitBuildFeedbackInMainThread();
      plan_grasps_action_server->publishFeedback(feedback_);
      last_feedback_step = current_feedback_step;
    }
  }

  ROS_INFO("About to Call emit emitProcessPlannerResultsInMainThread();");
  emit emitProcessPlannerResultsInMainThread();

  plan_grasps_action_server->setSucceeded(result_);
  ROS_INFO("Action ServerCB Finished");
}

void GraspitInterface::buildFeedbackInMainThread() {
  feedback_.current_step = m_planner->getCurrentStep();

  Hand *m_hand = graspitCore->getWorld()->getCurrentHand();

  feedback_.grasps.clear();
  feedback_.energies.clear();
  for (int i = 0; i < m_planner->getListSize(); i++) {
    const GraspPlanningState *gps = m_planner->getGrasp(i);
    graspit_interface::Grasp g;
    graspPlanningStateToROSMsg(gps, g, m_hand);

    feedback_.grasps.push_back(g);
    feedback_.energies.push_back(gps->getEnergy());
    feedback_.search_energy = goal.search_energy;
  }
}

void GraspitInterface::runPlannerInMainThread() {
  ROS_INFO("Planner Starting in Mainloop");
  ROS_INFO("Getting Hand");
  Hand *m_hand = graspitCore->getWorld()->getCurrentHand();
  if (m_hand == NULL) {
    ROS_INFO("Planning Hand is NULL");
  }
  GraspableBody *m_object = graspitCore->getWorld()->getGB(0);
  if (m_object == NULL) {
    ROS_INFO("Planning Object is NULL");
  }

  ROS_INFO("Initing m_hand_object_state");
  m_hand_object_state = new GraspPlanningState(m_hand);
  m_hand_object_state->setObject(m_object);

  switch (goal.search_space.type) {
  case graspit_interface::SearchSpace::SPACE_COMPLETE: {
    m_hand_object_state->setPositionType(SPACE_COMPLETE);
    m_hand_object_state->setRefTran(m_object->getTran());
    break;
  }
  case graspit_interface::SearchSpace::SPACE_AXIS_ANGLE: {
    m_hand_object_state->setPositionType(SPACE_AXIS_ANGLE);
    m_hand_object_state->setRefTran(m_object->getTran());
    break;
  }
  case graspit_interface::SearchSpace::SPACE_ELLIPSOID: {
    m_hand_object_state->setPositionType(SPACE_ELLIPSOID);
    m_hand_object_state->setRefTran(m_object->getTran());
    break;
  }
  case graspit_interface::SearchSpace::SPACE_APPROACH: {
    m_hand_object_state->setPositionType(SPACE_APPROACH);
    m_hand_object_state->setRefTran(m_hand->getTran());
    break;
  }
  default: {
    ROS_INFO("Invalid Search Space Type");
    // return;
  }
  }

  ROS_INFO("Initing m_hand_object_state");
  m_hand_object_state->reset();

  ROS_INFO("Initing m_planner");

  switch (goal.planner.type) {
  case graspit_interface::Planner::SIM_ANN: {
    m_planner = new SimAnnPlanner(m_hand);
    ROS_INFO("Using graspit_interface::Planner::SIM_ANN ");
    break;
  }
  case graspit_interface::Planner::MULTI_THREADED: {
    m_planner = new GuidedPlanner(m_hand);
    ROS_INFO("Using graspit_interface::Planner::MULTI_THREADED ");
    break;
  }
  default: {
    ROS_INFO("Invalid Planner Type");
    // return;
  }
  }

  m_planner->setEnergyType(goal.search_energy);

  if (!render_graphics)
    m_planner->setRenderType(RENDER_NEVER);
  if (goal.sim_ann_params.set_custom_params) {
    ROS_INFO("Switching SimAnn Annealing parameters to your custom defined "
             "values!!! ");
    SimAnnParams sim_ann_params;
    sim_ann_params.YC = goal.sim_ann_params.YC;
    sim_ann_params.HC = goal.sim_ann_params.HC;
    sim_ann_params.YDIMS = goal.sim_ann_params.YDIMS;
    sim_ann_params.NBR_ADJ = goal.sim_ann_params.NBR_ADJ;
    sim_ann_params.ERR_ADJ = goal.sim_ann_params.ERR_ADJ;
    sim_ann_params.DEF_T0 = goal.sim_ann_params.DEF_T0;
    sim_ann_params.DEF_K0 = goal.sim_ann_params.DEF_K0;
    m_planner->setAnnealingParameters(sim_ann_params);
  }

  switch (goal.search_contact.type) {
  case graspit_interface::SearchContact::CONTACT_PRESET: {
    m_planner->setContactType(CONTACT_PRESET);
    ROS_INFO("Using graspit_interface::SearchContact::CONTACT_PRESET ");
    break;
  }
  case graspit_interface::SearchContact::CONTACT_LIVE: {
    m_planner->setContactType(CONTACT_LIVE);
    ROS_INFO("Using graspit_interface::SearchContact::CONTACT_LIVE ");
    break;
  }
  default: {
    ROS_INFO("Invalid Search Contact Type");
    // return;
  }
  }

  ROS_INFO("Setting Planner Model State");
  m_planner->setModelState(m_hand_object_state);
  int max_steps = goal.max_steps;
  if (max_steps == 0) {
    max_steps = 70000;
  }
  ROS_INFO("Setting Planner Max Steps %d", max_steps);
  m_planner->setMaxSteps(max_steps);

  ROS_INFO("resetting Planner");
  m_planner->resetPlanner();

  ROS_INFO("Starting Planner");
  m_planner->startPlanner();
}

void GraspitInterface::graspPlanningStateToROSMsg(const GraspPlanningState *gps,
                                                  graspit_interface::Grasp &g,
                                                  Hand *m_hand) {
  gps->execute(m_hand);
  m_hand->autoGrasp(false, 1.0, false);

  geometry_msgs::Pose pose;
  transf t = m_hand->getTran();
  pose.position.x = t.translation().x() / AXIS_SCALE;
  pose.position.y = t.translation().y() / AXIS_SCALE;
  pose.position.z = t.translation().z() / AXIS_SCALE;
  pose.orientation.w = t.rotation().w();
  pose.orientation.x = t.rotation().x();
  pose.orientation.y = t.rotation().y();
  pose.orientation.z = t.rotation().z();

  geometry_msgs::Vector3Stamped approach_direction;
  vec3 approach_in_hand = m_hand->getApproachTran() * vec3(0, 0, 1);
  approach_in_hand.normalize();
  approach_direction.vector.x = approach_in_hand.x();
  approach_direction.vector.y = approach_in_hand.y();
  approach_direction.vector.z = approach_in_hand.z();
  approach_direction.header.frame_id = m_hand->getName().toStdString();

  g.graspable_body_id = goal.graspable_body_id;

  double dof[m_hand->getNumDOF()];
  m_hand->getDOFVals(dof);
  for (int i = 0; i < m_hand->getNumDOF(); ++i) {
    g.dofs.push_back(dof[i]);
  }

  g.pose = pose;
  m_hand->getGrasp()->update();
  QualVolume m_vol_qual(m_hand->getGrasp(), ("Volume"), "L1 Norm");
  QualEpsilon m_eps_qual(m_hand->getGrasp(), ("Epsilon"), "L1 Norm");

  graspitCore->getWorld()->findAllContacts();
  graspitCore->getWorld()->updateGrasps();

  g.epsilon_quality = m_eps_qual.evaluate();
  g.volume_quality = m_vol_qual.evaluate();
  g.approach_direction = approach_direction;
}

void GraspitInterface::processPlannerResultsInMainThread() {
  Hand *m_hand = graspitCore->getWorld()->getCurrentHand();
  if (m_hand == NULL) {
    ROS_INFO("Planning Hand is NULL");
  }
  GraspableBody *m_object = graspitCore->getWorld()->getGB(0);
  if (m_object == NULL) {
    ROS_INFO("Planning Object is NULL");
  }

  result_.grasps.clear();
  result_.energies.clear();

  ROS_INFO("Publishing Result");
  for (int i = 0; i < m_planner->getListSize(); i++) {
    const GraspPlanningState *gps = m_planner->getGrasp(i);
    graspit_interface::Grasp g;
    graspPlanningStateToROSMsg(gps, g, m_hand);

    result_.grasps.push_back(g);
    result_.energies.push_back(gps->getEnergy());
    result_.search_energy = goal.search_energy;
  }

  if (m_planner->getListSize() > 0) {
    ROS_INFO("Showing Grasp 0");
    m_planner->showGrasp(0);
  }

  if (m_hand_object_state != NULL) {
    delete m_hand_object_state;
    m_hand_object_state = NULL;
  }

  if (m_planner != NULL) {
    delete m_planner;
    m_planner = NULL;
  }
}

} // namespace GraspitInterface
