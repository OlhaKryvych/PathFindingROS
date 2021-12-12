#ifndef TECH_POSITION_HPP
#define TECH_POSITION_HPP

#include <rviz/tool.h>

namespace Ogre {
  class SceneNode;
  class Vector3;
}

namespace rviz {
  class VectorProperty;
  class VisualizationManager;
  class ViewportMouseEvent;
}

namespace pf_plugin {

/**
 * @brief The TechPosition class
 * @details This class provide a tool to creating and
 * displaying tech poses, graph nodes and current path
 * of mobile robot
 */
class TechPosition: public rviz::Tool {
    Q_OBJECT
public:
  TechPosition();
  ~TechPosition();

  /**
   * @brief onInitialize
   * @details Called during library initialization
   */
  virtual void onInitialize();

  /**
   * @brief activate
   * @details Called when plugin activates
   */
  virtual void activate();

  /**
   * @brief deactivate
   * @details Called on deactivation of plugin
   */
  virtual void deactivate();

  /**
   * @brief load
   * @param config
   * @details Load saved config
   */
  virtual void load( const rviz::Config& config );

  /**
   * @brief save
   * @param config
   * @details Saves current config
   */
  virtual void save( rviz::Config config ) const;
  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

protected slots:
  /**
  * @brief loadGraphNodesPositions
  * @param msg
  * @details A slot for reciving graph nodes
  * position and displaing them in 3D space
  */
  void loadGraphNodesPositions(const std_msgs::String::ConstPtr& msg);

  /**
   * @brief loadPathNodesPositions
   * @param msg
   * @details A slot for reciving a current path
   * and displaying it in 3D space
   */
  void loadPathNodesPositions(const std_msgs::String::ConstPtr& msg);

private:

  /**
   * @brief makePosition
   * @param position
   * @details This function creates a 3D object
   * for tech position
   */
  void makePosition( const Ogre::Vector3& position );

  /**
  * @brief makeNodePosition
  * @param position
  * @details This function creates a 3D object
  * for graph node position
  */
  void makeNodePosition( const Ogre::Vector3& position );

  /**
   * @brief makePathPosition
   * @param position
   * @details This function creates a 3D object
   * for part of current path
   */
  void makePathPosition( const Ogre::Vector3& position );

  /**
   * @brief publishPositions
   * @details This function execute posting
   * current technological positions to ROS
   */
  void publishPositions();

  void makeNode(const Ogre::Vector3& position, std::string resource, std::vector<Ogre::SceneNode*>* storage);

  std::vector<Ogre::SceneNode*> position_nodes_;
  std::vector<Ogre::SceneNode*> graph_nodes_;
  std::vector<Ogre::SceneNode*> path_nodes_;

  rviz::VectorProperty* current_position_property_;

  Ogre::SceneNode* moving_position_node_;

  // Resource string (for feture resource loading and manipulation)
  std::string position_resource_;
  std::string node_resource_;
  std::string path_resource_;

  // A ROS Publisher for posting a new tech poses
  ros::Publisher tech_position_publisher_;

  // A ROS Subscibers for reciving graph 
  // nodes (and displaying them) and current path 
  ros::Subscriber graph_nodes_subscriber_;
  ros::Subscriber path_nodes_subscriber_;

  // ROS Node Handler for creating and managing nodes and objects in RVIZ
  ros::NodeHandle nh_;
};


} // TECH_POSITION_HPP


#endif
