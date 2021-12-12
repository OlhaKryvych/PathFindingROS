#ifndef PATH_GRAPH_HPP
#define PATH_GRAPH_HPP

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <QSet>
#include <QColor>
#include <QHash>
#include <QTimer>
#include <QList>
#include <QThread>
#include "astar.hpp"

#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "utils.hpp"

#include <OgreVector3.h>

#include "posmover.hpp"


class QLineEdit;

class QPushButton;

class QComboBox;

class QLabel;

class QTableWidget;

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

namespace pf_plugin {


class PathGraph: public rviz::Panel {
    Q_OBJECT
public:
    PathGraph( QWidget* parent = nullptr );

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

signals:
    void proccessToPosition(QVector<double> rPos, QVector<double> pos, QVector<double> nextPos);

public slots:
    void setTopic( const QString& topic );

    /**
     * @brief targetReached
     * @details This function is executed when
     * target robot position reached
     */
    void targetReached();

    /**
     * @brief stopMoving
     *
     * @details This function stops current moving process
     */
    void stopMoving();

protected slots:
    /**
     * @brief updateTopic
     *
     * @details Updates current map`s topic to a new one
     */
    void updateTopic();

    /**
     * @brief openMapFile
     *
     * @details open and read new map file
     */
    void openMapFile();

    /**
     * @brief updateSystem
     *
     * @details Totol system update (graph update positions update)
     */
    void updateSystem();

    /**
     * @brief nextPath
     *
     * @details This function start
     * finding shortest path to a
     * next position from current
     * robot position
     */
    void nextPath();

    /**
     * @brief placeRobot
     *
     * @details Places robot to a default (0,0,0) position
     */
    void placeRobot();

    /**
     * @brief addTechPosToList
     *
     * @details Addes current selected techposition to a visit list
     */
    void addTechPosToList();

    /**
     * @brief moveToNextPosition
     *
     * @details Start moving to next position in visit list
     */
    void moveToNextPosition();
private:
    QLabel* map_topic_editor_;
    QPushButton* open_map_file_btn;
    QPushButton* add_target_to_visit_;
    QPushButton* update_graph_btn_;
    QPushButton* start_moving_btn_;
    QPushButton* stop_moving_btn_;
    QPushButton* place_robot_btn_;
    QComboBox* positions_combo_box_;
    QLabel* current_robot_pos_label_;
    QLabel* current_goal_pos_label_;
    QTableWidget* techpos_visit_list_;
    QString map_path_;

    QThread* mover_thread;

    PosMover* mover;

    /**
     * @brief loadDataFromMapFile
     * @param path a full path to a map file
     *
     * @details Loads data (map image and map parameters) from map file
     */
    void loadDataFromMapFile(const QString& path);

    /**
     * @brief createGraph
     *
     * @details Generate a road graph
     */
    void createGraph();

    /**
     * @brief resetGraph
     *
     * @details Deletes all nodes from current road graph (clearing it)
     */
    void resetGraph();

    /**
     * @brief clearNodesAndGraph
     *
     * @details Deletes all nodes and road graph
     */
    void clearNodesAndGraph();

    /**
     * @brief generateNodes
     *
     * @details This function create a set of nodes for current map
     */
    void generateNodes();

    /**
     * @brief generateGraph
     *
     * @details This function connects all Nodes in a single road graph
     */
    void generateGraph();

    /**
     * @brief resetRobotPosition
     * @param x
     * @param y
     *
     * @details Set robot position to (x, y, 0)
     */
    void resetRobotPosition(float x = 0, float y = 0);

    /**
     * @brief clearListToVisit
     *
     * @details This function clear all visit list by removing all techposes from it
     */
    void clearListToVisit();

    /**
     * @brief removePointFromList
     * @param row
     * @param coll
     *
     * @details Removes selected techpose from visit list
     */
    void removePointFromList(int row, int coll);

    /**
     * @brief getClosestNode
     * @param pos
     * @return A closes node to a given position
     */
    CNode* getClosestNode(Ogre::Vector3 pos);

    // map parameters

    float robot_size = 1.0; // 1 metter
    float cell_size = 0.0;

    float robotPosX = 0;
    float robotPosY = 0;

    geometry_msgs::PoseWithCovarianceStamped robotPosAMCL;

    QImage rawImage;
    QImage nodesImage;
    QImage graphImage;
    QImage currentPathImage;

    QString lastMap;

    int gridSize = 0;


    // for graph

    const QColor wallColor = "#000000";
    const QColor wayColor = "#ffffff";
    const QColor outColor = "#cdcdcd";
    const QColor nodeColor = "#0000FF";
    const QColor robotColor = "#FF0000";
    const QColor graphNode = "#00FF00";
    const QColor graphLine = "#00FF00";

    float distance(QPointF a, QPointF b);

    QSet<QPointF> nodes;

    QSet<CNode*> graph;
    AStar astar;

    // Communication

    ros::Subscriber tech_position_subsciber_;
    ros::Subscriber robot_position_subsciber_;
    ros::Subscriber amcl_position_subsciber_;
    ros::Subscriber map_subsciber_;

    // Control

    ros::Publisher graph_nodes_publisher_;

    ros::Publisher path_nodes_publisher_;

    ros::Publisher robot_position_publisher_;

    ros::NodeHandle nh_;

    void techPosesCallback(const std_msgs::String::ConstPtr& msg);

    void robotPosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void mapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void publishNodesPositions();

    void publishPathNodes(QList<Ogre::Vector3> path);

    QList<Ogre::Vector3> techPoses;

    void moveToPoint(Ogre::Vector3 start, Ogre::Vector3 end);

    // Path to move on and functions

    void moveTo(Ogre::Vector3 pos, Ogre::Vector3 next_pos = {0.0, 0.0, 0.0});

    QList<Ogre::Vector3> targetPath;

    QTimer stepTimer;
};

} // namespace pf_rviz_plugin

inline uint qHash (const QPointF & key)
{
    return qHash (QPair<double,double>(key.x(), key.y()) );
}

#endif // PATH_GRAPH_HPP
