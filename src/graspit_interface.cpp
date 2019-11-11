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
  nh->getParam("/Shape_Completion_Method", shape_completion_method);
  addObject_srv =
      nh->advertiseService("addObject", &GraspitInterface::addObjectCB, this);

  getRobot_srv =
      nh->advertiseService("getRobot", &GraspitInterface::getRobotCB, this);
  getGraspableBody_srv = nh->advertiseService(
      "getGraspableBody", &GraspitInterface::getGraspableBodyCB, this);
  getBody_srv =
      nh->advertiseService("getBody", &GraspitInterface::getBodyCB, this);
  getRobots_srv =
      nh->advertiseService("getRobots", &GraspitInterface::getRobotsCB, this);
  getGraspableBodies_srv = nh->advertiseService(
      "getGraspableBodies", &GraspitInterface::getGraspableBodiesCB, this);
  getBodies_srv =
      nh->advertiseService("getBodies", &GraspitInterface::getBodiesCB, this);
  setRobotPose_srv = nh->advertiseService(
      "setRobotPose", &GraspitInterface::setRobotPoseCB, this);
  setBodyPose_srv = nh->advertiseService(
      "setBodyPose", &GraspitInterface::setBodyPoseCB, this);
  setGraspableBodyPose_srv = nh->advertiseService(
      "setGraspableBodyPose", &GraspitInterface::setGraspableBodyPoseCB, this);

  getDynamics_srv = nh->advertiseService(
      "getDynamics", &GraspitInterface::getDynamicsCB, this);
  setDynamics_srv = nh->advertiseService(
      "setDynamics", &GraspitInterface::setDynamicsCB, this);

  autoGrasp_srv =
      nh->advertiseService("autoGrasp", &GraspitInterface::autoGraspCB, this);
  autoOpen_srv =
      nh->advertiseService("autoOpen", &GraspitInterface::autoOpenCB, this);

  forceRobotDOF_srv = nh->advertiseService(
      "forceRobotDof", &GraspitInterface::forceRobotDOFCB, this);
  moveDOFToContacts_srv = nh->advertiseService(
      "moveDOFToContacts", &GraspitInterface::moveDOFToContactsCB, this);
  setRobotDesiredDOF_srv = nh->advertiseService(
      "setRobotDesiredDOF", &GraspitInterface::setRobotDesiredDOFCB, this);

  importRobot_srv = nh->advertiseService(
      "importRobot", &GraspitInterface::importRobotCB, this);
  importObstacle_srv = nh->advertiseService(
      "importObstacle", &GraspitInterface::importObstacleCB, this);
  importGraspableBody_srv = nh->advertiseService(
      "importGraspableBody", &GraspitInterface::importGraspableBodyCB, this);

  // Shape completion stuff
  shape_complete_scene_action_client = new actionlib::SimpleActionClient<
      scene_completion_msgs::CompleteSceneAction>(
      "/scene_completion/SceneCompletion", true);
  plan_x_best_grasps_action_client = new actionlib::SimpleActionClient<
      graspit_interface::PlanXBestGraspsAction>("/planBestGrasp", true);

  meshed_scene_repub =
      nh->advertise<scene_completion_msgs::CompleteSceneResult>(
          "/get_segmented_meshed_scene", 1);
  meshed_scene_sub =
      nh->subscribe("/get_segmented_meshed_scene", 10,
                    &GraspitInterface::getSegmentedMeshesCB, this);
  clearWorld_srv =
      nh->advertiseService("clearWorld", &GraspitInterface::clearWorldCB, this);
  loadWorld_srv =
      nh->advertiseService("loadWorld", &GraspitInterface::loadWorldCB, this);
  saveWorld_srv =
      nh->advertiseService("saveWorld", &GraspitInterface::saveWorldCB, this);

  saveImage_srv =
      nh->advertiseService("saveImage", &GraspitInterface::saveImageCB, this);
  toggleAllCollisions_srv = nh->advertiseService(
      "toggleAllCollisions", &GraspitInterface::toggleAllCollisionsCB, this);

  computeQuality_srv = nh->advertiseService(
      "computeQuality", &GraspitInterface::computeQualityCB, this);
  computeEnergy_srv = nh->advertiseService(
      "computeEnergy", &GraspitInterface::computeEnergyCB, this);

  approachToContact_srv = nh->advertiseService(
      "approachToContact", &GraspitInterface::approachToContactCB, this);
  findInitialContact_srv = nh->advertiseService(
      "findInitialContact", &GraspitInterface::findInitialContactCB, this);
  dynamicAutoGraspComplete_srv =
      nh->advertiseService("dynamicAutoGraspComplete",
                           &GraspitInterface::dynamicAutoGraspCompleteCB, this);

  plan_grasps_as =
      new actionlib::SimpleActionServer<graspit_interface::PlanGraspsAction>(
          *nh, "planGrasps",
          boost::bind(&GraspitInterface::PlanGraspsCB, this, _1), false);
  plan_grasps_as->start();

  firstTimeInMainLoop = true;

  mPlanner = NULL;
  mHandObjectState = NULL;
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
  srand(1);
  ROS_INFO("MAKING SHAPE COMPLETION UI");
  QPushButton *shapeCompleteSceneButton = new QPushButton("Scene Completion");
  QPushButton *generateGraspButton = new QPushButton("Generate Grasps");

  shapeCompleteSceneButton->setDefault(true);
  generateGraspButton->setDefault(true);

  QWidget *shapeCompletionControlBox = new QWidget();

  scene_completion_time = new QLabel(tr("Scene Completion Time:"));
  grasp_planning_time = new QLabel(tr("Grasp Planning Time:"));

  QGridLayout *mainLayout = new QGridLayout;

  mainLayout->addWidget(shapeCompleteSceneButton, 0, 0);
  mainLayout->addWidget(generateGraspButton, 1, 0);
  mainLayout->addWidget(scene_completion_time, 2, 0);
  mainLayout->addWidget(grasp_planning_time, 3, 0);
  shapeCompletionControlBox->setLayout(mainLayout);

  // shapeCompletionControlBox->resize(QSize(200,100));

  shapeCompletionControlBox->show();

  QObject::connect(shapeCompleteSceneButton, SIGNAL(clicked()), this,
                   SLOT(onShapeCompleteSceneButtonPressed()));
  QObject::connect(generateGraspButton, SIGNAL(clicked()), this,
                   SLOT(onGenerateGraspButtonPressed()));
  ROS_INFO("GraspIt interface successfully initialized!");

  return 0;
}

int GraspitInterface::mainLoop() {
  if (firstTimeInMainLoop) {
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
    firstTimeInMainLoop = false;
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

void GraspitInterface::onGenerateGraspButtonPressed() {
  ROS_INFO("onGenerateGraspButtonPressed\n");
  graspit_interface::PlanXBestGraspsGoal goal;
  goal.filename = mean_mesh_file_path;
  goal.meshOffset = mesh_offset;
  ROS_INFO("Generate grasps with method %s, planner %s, and number of "
           "best grasps %d",
           shape_completion_method.c_str(), grasp_planning_method.c_str(),
           number_of_best_grasps_to_return);
  goal.method = shape_completion_method;
  goal.numBestGrasps = number_of_best_grasps_to_return;
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
  indices_of_best_epsilon_grasps = result->bestEpsGrasps;
  indices_of_best_volume_grasps = result->bestVolGrasps;
  gripper_pose_for_all_grasps = result->handPoses;
  gripper_joint_state_for_all_grasps = result->handJointStates;
}

void GraspitInterface::getSegmentedMeshesCB(
    const scene_completion_msgs::CompleteSceneResultConstPtr &result) {

  ROS_INFO("Entering getSegmentedMeshesCB");
  if (QThread::currentThread() != QApplication::instance()->thread()) {
    meshed_scene_repub.publish(result);
  } else {
    ROS_INFO("Recieved %lu mean meshes and %lu sampled meshes",
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

  ROS_INFO("Sucessfully recieved meshed scene");
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

void GraspitInterface::addObject(graspit_msgs::ObjectInfo object) {
  ROS_INFO("Entering addObject function");
  QString modelName(QString::fromStdString(object.model_name));
  QString objectName(QString::fromStdString(object.object_name));
  transf object_pose = transf(Quaternion(object.object_pose.orientation.w,
                                         object.object_pose.orientation.x,
                                         object.object_pose.orientation.y,
                                         object.object_pose.orientation.z),
                              vec3(object.object_pose.position.x * AXIS_SCALE,
                                   object.object_pose.position.y * AXIS_SCALE,
                                   object.object_pose.position.z * AXIS_SCALE));
  ROS_INFO("Adding Model %s", modelName.toStdString().c_str());
  ROS_INFO("Adding Model %s", objectName.toStdString().c_str());
  addToWorld(modelName, objectName, object_pose);
}

void GraspitInterface::addToWorld(const QString modelname,
                                  const QString object_name,
                                  const transf object_pose) {
  QString model_filename = modelname + QString(".xml");
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

  QString fullfilepath =
      QString(getenv("GRASPIT")) + QString("/models/objects/");
  if (!is_mean) {
    QString meshname = QString("mesh_sample_") + mesh_index;
    ROS_INFO("This sampled mesh is only saved to an xml file.\n");
    saveMesh(fullfilepath, meshname, pcl_mesh);
  } else {
    GraspableBody *b = new GraspableBody(graspitCore->getWorld());
    if (b->loadGeometryMemory(vertices, triangles) == SUCCESS) {
      ROS_INFO("setting mesh name\n");
      QString meshname = QString("mesh_mean_") + mesh_index;
      b->setName(meshname);

      ROS_INFO("About to add IVMAT\n");
      b->addIVMat(true);

      ROS_INFO("About to add to IVC\n");
      b->addToIvc();
      SoSeparator *IVGeomRoot = b->getIVGeomRoot();
      SoTransform *IVScaleTran = new SoTransform;

      IVScaleTran->scaleFactor.setValue(AXIS_SCALE, AXIS_SCALE, AXIS_SCALE);
      IVGeomRoot->insertChild(IVScaleTran, 0);

      // ROS_INFO("offsetting mesh\n");
      // transf *t =	new transf(mat3::Identity(), vec3(offset.x, offset.y,
      // offset.z)); b->setTran(*t);
      ROS_INFO("Adding mean mesh to main world\n");
      graspitCore->getWorld()->addBody(b);
      ROS_INFO("Added Mesh: %s", b->getName().toStdString().c_str());

      b->setFilename(QString("models/objects/") + b->getName() + ".xml");

      saveMesh(fullfilepath, b->getName(), pcl_mesh, true);
    } else {
      ROS_INFO("Failed to load mesh geometry; Mesh: %s",
               b->getName().toStdString().c_str());
    }
  }
}

void GraspitInterface::saveMesh(const QString filepath, const QString filename,
                                const pcl::PolygonMesh *pcl_mesh,
                                const bool is_mean) {

  QString mesh_filename = filename + QString(".ply");
  QString mesh_fullfilepath = filepath + mesh_filename;
  if (is_mean) {
    mean_mesh_file_path = (filepath + filename).toStdString();
  }

  ROS_INFO("Saving in mesh file: %s", mesh_fullfilepath.toStdString().c_str());

  pcl::io::savePLYFileBinary(mesh_fullfilepath.toStdString().c_str(),
                             *pcl_mesh);

  QString xml_fullfilepath = filepath + filename + QString(".xml");
  ROS_INFO("Saving in xml file: %s", xml_fullfilepath.toStdString().c_str());
  ROS_INFO("saving .xml");
  QFile file(xml_fullfilepath);
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
      ;
      c.ps.pose.position.z = contactInWorldFrame.translation().z() / AXIS_SCALE;
      ;
      c.ps.pose.orientation.w = contactInWorldFrame.rotation().w();
      c.ps.pose.orientation.x = contactInWorldFrame.rotation().x();
      c.ps.pose.orientation.y = contactInWorldFrame.rotation().y();
      c.ps.pose.orientation.z = contactInWorldFrame.rotation().z();
      c.cof = (*it)->getCof();
      response.robot.contacts.push_back(c);
    }

    double *jointVals = new double[r->getNumJoints()];
    r->getJointValues(jointVals);

    sensor_msgs::JointState robot_joint_state = sensor_msgs::JointState();
    for (int i = 0; i < r->getNumJoints(); i++) {

      std::string joint_name;
      std::ostringstream convert;
      convert << i;
      joint_name = convert.str();

      robot_joint_state.name.push_back(joint_name);
      robot_joint_state.position.push_back(jointVals[i]);
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

      geometry_msgs::PoseStamped poseStamped = geometry_msgs::PoseStamped();

      poseStamped.header.frame_id = "world";
      poseStamped.pose.position.x = t.translation().x() / AXIS_SCALE;
      poseStamped.pose.position.y = t.translation().y() / AXIS_SCALE;
      ;
      poseStamped.pose.position.z = t.translation().z() / AXIS_SCALE;
      ;
      poseStamped.pose.orientation.w = t.rotation().w();
      poseStamped.pose.orientation.x = t.rotation().x();
      poseStamped.pose.orientation.y = t.rotation().y();
      poseStamped.pose.orientation.z = t.rotation().z();

      response.robot.tactile.sensor_poses.push_back(poseStamped);
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
    ;
    pose.position.z = t.translation().z() / AXIS_SCALE;
    ;
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
    vec3 newTranslation(request.pose.position.x * AXIS_SCALE,
                        request.pose.position.y * AXIS_SCALE,
                        request.pose.position.z * AXIS_SCALE);

    Quaternion newRotation(
        request.pose.orientation.w, request.pose.orientation.x,
        request.pose.orientation.y, request.pose.orientation.z);

    transf newTransform(newRotation, newTranslation);

    graspitCore->getWorld()->getRobot(request.id)->setTran(newTransform);
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

    vec3 newTranslation(request.pose.position.x * AXIS_SCALE,
                        request.pose.position.y * AXIS_SCALE,
                        request.pose.position.z * AXIS_SCALE);

    Quaternion newRotation(
        request.pose.orientation.w, request.pose.orientation.x,
        request.pose.orientation.y, request.pose.orientation.z);

    transf newTransform(newRotation, newTranslation);

    graspitCore->getWorld()->getGB(request.id)->setTran(newTransform);
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

    vec3 newTranslation(request.pose.position.x * AXIS_SCALE,
                        request.pose.position.y * AXIS_SCALE,
                        request.pose.position.z * AXIS_SCALE);

    Quaternion newRotation(
        request.pose.orientation.w, request.pose.orientation.x,
        request.pose.orientation.y, request.pose.orientation.z);

    transf newTransform(newRotation, newTranslation);

    graspitCore->getWorld()->getBody(request.id)->setTran(newTransform);
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
  vec3 newTranslation(request.pose.position.x * AXIS_SCALE,
                      request.pose.position.y * AXIS_SCALE,
                      request.pose.position.z * AXIS_SCALE);

  Quaternion newRotation(request.pose.orientation.w, request.pose.orientation.x,
                         request.pose.orientation.y,
                         request.pose.orientation.z);

  transf newTransform(newRotation, newTranslation);
  r->setTran(newTransform);
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
    // Now try to load using unaltered filepath from request.
    b = graspitCore->getWorld()->importBody(QString("Body"),
                                            QString(request.filename.data()));
    if (b == NULL) {
      response.result = response.RESULT_FAILURE;
      return true;
    }
  }

  vec3 newTranslation(request.pose.position.x * AXIS_SCALE,
                      request.pose.position.y * AXIS_SCALE,
                      request.pose.position.z * AXIS_SCALE);

  Quaternion newRotation(request.pose.orientation.w, request.pose.orientation.x,
                         request.pose.orientation.y,
                         request.pose.orientation.z);

  transf newTransform(newRotation, newTranslation);
  b->setTran(newTransform);
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
    // Now try to load using unaltered filepath from request.
    b = graspitCore->getWorld()->importBody(QString("GraspableBody"),
                                            QString(request.filename.data()));
    if (b == NULL) {
      response.result = response.RESULT_FAILURE;
      return true;
    }
  }

  vec3 newTranslation(request.pose.position.x * AXIS_SCALE,
                      request.pose.position.y * AXIS_SCALE,
                      request.pose.position.z * AXIS_SCALE);

  Quaternion newRotation(request.pose.orientation.w, request.pose.orientation.x,
                         request.pose.orientation.y,
                         request.pose.orientation.z);

  transf newTransform(newRotation, newTranslation);
  b->setTran(newTransform);

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
  CollisionReport colReport;
  // first test whether the hand is in collision now
  int numCols = graspitCore->getWorld()->getCollisionReport(&colReport);
  // if it is in collision, then there should be no reason to calculate the
  // quality
  if (numCols > 0) {
    response.result = response.RESULT_COLLISION;
    response.epsilon = -1.0;
    response.volume = -1.0;
    return true;
  }
  Hand *mHand = graspitCore->getWorld()->getHand(request.id);
  if (mHand == NULL) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  }

  // if there is no collision, then begin computation

  QualVolume mVolQual(mHand->getGrasp(), ("Volume"), "L1 Norm");
  QualEpsilon mEpsQual(mHand->getGrasp(), ("Epsilon"), "L1 Norm");

  graspitCore->getWorld()->findAllContacts();
  graspitCore->getWorld()->updateGrasps();

  response.epsilon = mEpsQual.evaluate();
  response.volume = mVolQual.evaluate();

  return true;
}

bool GraspitInterface::computeEnergyCB(
    graspit_interface::ComputeEnergy::Request &request,
    graspit_interface::ComputeEnergy::Response &response) {
  Hand *mHand = graspitCore->getWorld()->getHand(request.handId);
  if (mHand == NULL) {
    response.result = response.RESULT_INVALID_HAND_ID;
    ROS_INFO("Planning Hand is NULL");
    return true;
  }
  GraspableBody *mObject =
      graspitCore->getWorld()->getGB(request.graspableBodyId);
  if (mObject == NULL) {
    ROS_INFO("Planning Object is NULL");
    response.result = response.RESULT_INVALID_BODY_ID;
    return true;
  }

  graspitCore->getWorld()->findAllContacts();
  graspitCore->getWorld()->updateGrasps();

  std::vector<std::string> energyTypes =
      SearchEnergyFactory::getInstance()->getAllRegisteredEnergy();
  if (std::find(energyTypes.begin(), energyTypes.end(), request.energyType) ==
      energyTypes.end()) {
    ROS_INFO_STREAM("Invalid Energy Type " << request.energyType << std::endl);
    response.result = response.RESULT_INVALID_ENERGY_TYPE;
    return true;
  }

  SearchEnergy *se =
      SearchEnergyFactory::getInstance()->createEnergy(request.energyType);

  bool isLegal;
  double stateEnergy;
  se->analyzeCurrentPosture(mHand, mObject, isLegal, stateEnergy);
  response.isLegal = isLegal;
  response.energy = stateEnergy;

  return true;
}

bool GraspitInterface::approachToContactCB(
    graspit_interface::ApproachToContact::Request &request,
    graspit_interface::ApproachToContact::Response &response) {
  Hand *mHand = graspitCore->getWorld()->getHand(request.id);
  if (mHand == NULL) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  }
  mHand->approachToContact(request.moveDist, request.oneStep);
  return true;
}

bool GraspitInterface::findInitialContactCB(
    graspit_interface::FindInitialContact::Request &request,
    graspit_interface::FindInitialContact::Response &response) {
  Hand *mHand = graspitCore->getWorld()->getHand(request.id);
  if (mHand == NULL) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  }

  mHand->findInitialContact(request.moveDist);
  return true;
}

bool GraspitInterface::dynamicAutoGraspCompleteCB(
    graspit_interface::DynamicAutoGraspComplete::Request &request,
    graspit_interface::DynamicAutoGraspComplete::Response &response) {
  Hand *mHand = graspitCore->getWorld()->getCurrentHand();
  if (mHand == NULL) {
    response.result = response.RESULT_INVALID_ID;
    return true;
  }
  response.GraspComplete = mHand->dynamicAutograspComplete();
  return true;
}

void GraspitInterface::PlanGraspsCB(
    const graspit_interface::PlanGraspsGoalConstPtr &_goal) {
  goal = *_goal;
  ROS_INFO("About to Call emit runPlannerInMainLoop();");
  emit emitRunPlannerInMainThread();

  ROS_INFO("Waiting For Planner to Finish");
  int last_feedback_step;
  while (mPlanner->isActive()) {
    if (_goal->feedback_num_steps < 1) {
      continue;
    }
    int current_feedback_step = mPlanner->getCurrentStep();
    if (current_feedback_step == mPlanner->getStartingStep()) {
      continue;
    }
    if ((current_feedback_step % _goal->feedback_num_steps == 0) &&
        (current_feedback_step != last_feedback_step)) {
      ROS_INFO("Curret Planner Step: %d", mPlanner->getCurrentStep());
      ROS_INFO("Curret Num Grasps: %d", mPlanner->getListSize());

      // collect grasps
      emit emitBuildFeedbackInMainThread();
      plan_grasps_as->publishFeedback(feedback_);
      last_feedback_step = current_feedback_step;
    }
  }

  ROS_INFO("About to Call emit emitProcessPlannerResultsInMainThread();");
  emit emitProcessPlannerResultsInMainThread();

  plan_grasps_as->setSucceeded(result_);
  ROS_INFO("Action ServerCB Finished");
}

void GraspitInterface::buildFeedbackInMainThread() {
  feedback_.current_step = mPlanner->getCurrentStep();

  Hand *mHand = graspitCore->getWorld()->getCurrentHand();

  feedback_.grasps.clear();
  feedback_.energies.clear();
  for (int i = 0; i < mPlanner->getListSize(); i++) {
    const GraspPlanningState *gps = mPlanner->getGrasp(i);
    graspit_interface::Grasp g;
    graspPlanningStateToROSMsg(gps, g, mHand);

    feedback_.grasps.push_back(g);
    feedback_.energies.push_back(gps->getEnergy());
    feedback_.search_energy = goal.search_energy;
  }
}

void GraspitInterface::runPlannerInMainThread() {
  ROS_INFO("Planner Starting in Mainloop");
  ROS_INFO("Getting Hand");
  Hand *mHand = graspitCore->getWorld()->getCurrentHand();
  if (mHand == NULL) {
    ROS_INFO("Planning Hand is NULL");
  }
  GraspableBody *mObject = graspitCore->getWorld()->getGB(0);
  if (mObject == NULL) {
    ROS_INFO("Planning Object is NULL");
  }

  ROS_INFO("Initing mHandObjectState");
  mHandObjectState = new GraspPlanningState(mHand);
  mHandObjectState->setObject(mObject);

  switch (goal.search_space.type) {
  case graspit_interface::SearchSpace::SPACE_COMPLETE: {
    mHandObjectState->setPositionType(SPACE_COMPLETE);
    mHandObjectState->setRefTran(mObject->getTran());
    break;
  }
  case graspit_interface::SearchSpace::SPACE_AXIS_ANGLE: {
    mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
    mHandObjectState->setRefTran(mObject->getTran());
    break;
  }
  case graspit_interface::SearchSpace::SPACE_ELLIPSOID: {
    mHandObjectState->setPositionType(SPACE_ELLIPSOID);
    mHandObjectState->setRefTran(mObject->getTran());
    break;
  }
  case graspit_interface::SearchSpace::SPACE_APPROACH: {
    mHandObjectState->setPositionType(SPACE_APPROACH);
    mHandObjectState->setRefTran(mHand->getTran());
    break;
  }
  default: {
    ROS_INFO("Invalid Search Space Type");
    // return;
  }
  }

  ROS_INFO("Initing mHandObjectState");
  mHandObjectState->reset();

  ROS_INFO("Initing mPlanner");

  switch (goal.planner.type) {
  case graspit_interface::Planner::SIM_ANN: {
    mPlanner = new SimAnnPlanner(mHand);
    ROS_INFO("Using graspit_interface::Planner::SIM_ANN ");
    break;
  }
  case graspit_interface::Planner::MULTI_THREADED: {
    mPlanner = new GuidedPlanner(mHand);
    ROS_INFO("Using graspit_interface::Planner::MULTI_THREADED ");
    break;
  }
  default: {
    ROS_INFO("Invalid Planner Type");
    // return;
  }
  }

  mPlanner->setEnergyType(goal.search_energy);

  if (!render_graphics)
    mPlanner->setRenderType(RENDER_NEVER);
  if (goal.sim_ann_params.set_custom_params) {
    ROS_INFO("Switching SimAnn Annealing parameters to your custom defined "
             "values!!! ");
    SimAnnParams simAnnParams;
    simAnnParams.YC = goal.sim_ann_params.YC;
    simAnnParams.HC = goal.sim_ann_params.HC;
    simAnnParams.YDIMS = goal.sim_ann_params.YDIMS;
    simAnnParams.NBR_ADJ = goal.sim_ann_params.NBR_ADJ;
    simAnnParams.ERR_ADJ = goal.sim_ann_params.ERR_ADJ;
    simAnnParams.DEF_T0 = goal.sim_ann_params.DEF_T0;
    simAnnParams.DEF_K0 = goal.sim_ann_params.DEF_K0;
    mPlanner->setAnnealingParameters(simAnnParams);
  }

  switch (goal.search_contact.type) {
  case graspit_interface::SearchContact::CONTACT_PRESET: {
    mPlanner->setContactType(CONTACT_PRESET);
    ROS_INFO("Using graspit_interface::SearchContact::CONTACT_PRESET ");
    break;
  }
  case graspit_interface::SearchContact::CONTACT_LIVE: {
    mPlanner->setContactType(CONTACT_LIVE);
    ROS_INFO("Using graspit_interface::SearchContact::CONTACT_LIVE ");
    break;
  }
  default: {
    ROS_INFO("Invalid Search Contact Type");
    // return;
  }
  }

  ROS_INFO("Setting Planner Model State");
  mPlanner->setModelState(mHandObjectState);
  int max_steps = goal.max_steps;
  if (max_steps == 0) {
    max_steps = 70000;
  }
  ROS_INFO("Setting Planner Max Steps %d", max_steps);
  mPlanner->setMaxSteps(max_steps);

  ROS_INFO("resetting Planner");
  mPlanner->resetPlanner();

  ROS_INFO("Starting Planner");
  mPlanner->startPlanner();
}

void GraspitInterface::graspPlanningStateToROSMsg(const GraspPlanningState *gps,
                                                  graspit_interface::Grasp &g,
                                                  Hand *mHand) {
  gps->execute(mHand);
  mHand->autoGrasp(false, 1.0, false);

  geometry_msgs::Pose pose;
  transf t = mHand->getTran();
  pose.position.x = t.translation().x() / AXIS_SCALE;
  pose.position.y = t.translation().y() / AXIS_SCALE;
  ;
  pose.position.z = t.translation().z() / AXIS_SCALE;
  ;
  pose.orientation.w = t.rotation().w();
  pose.orientation.x = t.rotation().x();
  pose.orientation.y = t.rotation().y();
  pose.orientation.z = t.rotation().z();

  geometry_msgs::Vector3Stamped approach_direction;
  vec3 approachInHand = mHand->getApproachTran() * vec3(0, 0, 1);
  approachInHand.normalize();
  approach_direction.vector.x = approachInHand.x();
  approach_direction.vector.y = approachInHand.y();
  approach_direction.vector.z = approachInHand.z();
  approach_direction.header.frame_id = mHand->getName().toStdString();

  g.graspable_body_id = goal.graspable_body_id;

  double dof[mHand->getNumDOF()];
  mHand->getDOFVals(dof);
  for (int i = 0; i < mHand->getNumDOF(); ++i) {
    g.dofs.push_back(dof[i]);
  }

  g.pose = pose;
  mHand->getGrasp()->update();
  QualVolume mVolQual(mHand->getGrasp(), ("Volume"), "L1 Norm");
  QualEpsilon mEpsQual(mHand->getGrasp(), ("Epsilon"), "L1 Norm");

  graspitCore->getWorld()->findAllContacts();
  graspitCore->getWorld()->updateGrasps();

  g.epsilon_quality = mEpsQual.evaluate();
  g.volume_quality = mVolQual.evaluate();
  g.approach_direction = approach_direction;
}

void GraspitInterface::processPlannerResultsInMainThread() {
  Hand *mHand = graspitCore->getWorld()->getCurrentHand();
  if (mHand == NULL) {
    ROS_INFO("Planning Hand is NULL");
  }
  GraspableBody *mObject = graspitCore->getWorld()->getGB(0);
  if (mObject == NULL) {
    ROS_INFO("Planning Object is NULL");
  }

  result_.grasps.clear();
  result_.energies.clear();

  ROS_INFO("Publishing Result");
  for (int i = 0; i < mPlanner->getListSize(); i++) {
    const GraspPlanningState *gps = mPlanner->getGrasp(i);
    graspit_interface::Grasp g;
    graspPlanningStateToROSMsg(gps, g, mHand);

    result_.grasps.push_back(g);
    result_.energies.push_back(gps->getEnergy());
    result_.search_energy = goal.search_energy;
  }

  if (mPlanner->getListSize() > 0) {
    ROS_INFO("Showing Grasp 0");
    mPlanner->showGrasp(0);
  }

  if (mHandObjectState != NULL) {
    delete mHandObjectState;
    mHandObjectState = NULL;
  }

  if (mPlanner != NULL) {
    delete mPlanner;
    mPlanner = NULL;
  }
}

} // namespace GraspitInterface
