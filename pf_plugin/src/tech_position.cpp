#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QVariant>
#include <QMessageBox>

#include "config.hpp"
#include "tech_position.hpp"

namespace pf_plugin {

TechPosition::TechPosition()
  : moving_position_node_( nullptr )
  , current_position_property_( nullptr )
{
  shortcut_key_ = 'l';
  tech_position_publisher_ = nh_.advertise<std_msgs::String>(CONFIG::TECH_POSES_TOPIC, 1);

  graph_nodes_subscriber_ = nh_.subscribe(CONFIG::NODES_POSES_TOPIC, 1, &TechPosition::loadGraphNodesPositions, this);

  path_nodes_subscriber_ = nh_.subscribe(CONFIG::CURRENT_PATH_NODES_TOPIC, 1, &TechPosition::loadPathNodesPositions, this);
}

void TechPosition::loadGraphNodesPositions(const std_msgs::String::ConstPtr& msg){

  for( unsigned i = 0; i < graph_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( graph_nodes_[ i ]);
  }

  graph_nodes_.clear();

  QJsonDocument doc = QJsonDocument::fromJson(msg->data.c_str());
  QJsonObject obj = doc.object();

  QJsonArray _nodes = obj["nodes"].toArray();

  for (auto item : _nodes) {

      auto _obj = item.toObject();

      Ogre::Vector3 position;

      position.x = _obj["x"].toDouble();
      position.y = _obj["y"].toDouble();
      position.z = _obj["z"].toDouble();

      makeNodePosition(position);
  }
}

void TechPosition::loadPathNodesPositions(const std_msgs::String::ConstPtr& msg){
  for( unsigned i = 0; i < path_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( path_nodes_[ i ]);
  }

  path_nodes_.clear();

  QJsonDocument doc = QJsonDocument::fromJson(msg->data.c_str());
  QJsonObject obj = doc.object();

  QJsonArray _nodes = obj["path"].toArray();

  for (auto item : _nodes) {

      auto _obj = item.toObject();

      Ogre::Vector3 position;

      position.x = _obj["x"].toDouble();
      position.y = _obj["y"].toDouble();
      position.z = _obj["z"].toDouble();

      makePathPosition(position);
  }
}

TechPosition::~TechPosition() {
  for( unsigned i = 0; i < position_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( position_nodes_[ i ]);
  }
}

void TechPosition::onInitialize()
{
  position_resource_ = "package://pf_plugin/media/tech_position.dae";

  if( rviz::loadMeshFromResource( position_resource_ ).isNull() ) {
    ROS_ERROR( "TechPosition: failed to load model resource '%s'.", position_resource_.c_str() );
    return;
  }

  moving_position_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( position_resource_ );
  moving_position_node_->attachObject( entity );
  moving_position_node_->setVisible( false );

  node_resource_ = "package://pf_plugin/media/node_position.dae";

  if( rviz::loadMeshFromResource( node_resource_ ).isNull() ) {
    ROS_ERROR( "TechPosition: failed to load model resource '%s'.", node_resource_.c_str() );
    return;
  }

  path_resource_ = "package://pf_plugin/media/path_position.dae";

  if( rviz::loadMeshFromResource( path_resource_ ).isNull() ) {
    ROS_ERROR( "TechPosition: failed to load model resource '%s'.", path_resource_.c_str() );
    return;
  }
}

void TechPosition::activate()
{
  if( moving_position_node_ )
  {
    moving_position_node_->setVisible( true );

    current_position_property_ = new rviz::VectorProperty( "Position " + QString::number( position_nodes_.size() ));
    current_position_property_->setReadOnly( true );
    getPropertyContainer()->addChild( current_position_property_ );
  }

  for (auto node : path_nodes_) {
    node->setVisible( true );
  }

  for (auto node : position_nodes_) {
    node->setVisible( true );
  }

  for (auto node : graph_nodes_) {
    node->setVisible( true );
  }
}

void TechPosition::deactivate()
{
  if( moving_position_node_ )
  {
    moving_position_node_->setVisible( false );
    delete current_position_property_;
    current_position_property_ = nullptr;
  }
}

int TechPosition::processMouseEvent( rviz::ViewportMouseEvent& event ) {
  if( !moving_position_node_ ) {
    return Render;
  }

  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );

  if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection )) {
    moving_position_node_->setVisible( true );
    moving_position_node_->setPosition( intersection );
    current_position_property_->setVector( intersection );

    if( event.leftDown() ) {
      makePosition( intersection );
      current_position_property_ = nullptr; // Drop the reference so that deactivate() won't remove it.
      return Render | Finished;
    }
  } else {
    moving_position_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the flag.
  }

  return Render;
}

void TechPosition::makePosition( const Ogre::Vector3& position )
{
  makeNode(position, position_resource_, &position_nodes_);
  publishPositions();
}

void TechPosition::makeNodePosition( const Ogre::Vector3& position ) {
  makeNode(position, node_resource_, &graph_nodes_);
}

void TechPosition::makePathPosition( const Ogre::Vector3& position ) {
  makeNode(position, path_resource_, &path_nodes_);
}

void TechPosition::publishPositions() {
  if( ros::ok() && tech_position_publisher_ ) {
    std_msgs::String msg;
  
    QJsonObject obj;
    QJsonArray arr;

    int num = 0;

    for (auto pos : position_nodes_) {
      QJsonObject _obj;

      _obj.insert("pos_name", QString("TP #%1").arg(num++));
      
      auto vec3 = pos->getPosition();

      _obj.insert("x", vec3.x);
      _obj.insert("y", vec3.y);
      _obj.insert("z", vec3.z);

      arr.append(_obj);
    }

    obj.insert("positions", arr);
    QJsonDocument doc(obj);
    msg.data = doc.toJson().toStdString();

    tech_position_publisher_.publish( msg );
  }
}

void TechPosition::makeNode(const Ogre::Vector3 &position, std::string resource, std::vector<Ogre::SceneNode *> *storage)
{
    Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity( resource );
    node->attachObject( entity );
    node->setVisible( true );
    node->setPosition( position );
    storage->push_back( node );
}

void TechPosition::save( rviz::Config config ) const
{
}

void TechPosition::load( const rviz::Config& config )
{
}

}; // namespace pf_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pf_plugin::TechPosition,rviz::Tool )
