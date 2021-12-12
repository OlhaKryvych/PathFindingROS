#include "pathgraph.hpp"

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QComboBox>
#include <QMessageBox>
#include <QPushButton>
#include <QFileInfo>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QVariant>
#include <QStandardPaths>
#include <QFileDialog>
#include <QTableWidget>
#include <QtConcurrent/QtConcurrent>

#include <ros/console.h>
#include <ros/ros.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/vector_property.h>

#include "yaml-cpp/yaml.h"

#include "config.hpp"

#include "utils.hpp"

Q_DECLARE_METATYPE(geometry_msgs::PoseWithCovarianceStamped);
Q_DECLARE_METATYPE(Ogre::Vector3);

namespace pf_plugin {

PathGraph::PathGraph(QWidget* parent) : rviz::Panel( parent ) {

  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Map topic:" ));
  map_topic_editor_ = new QLabel;
  topic_layout->addWidget( map_topic_editor_ );


  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );

  open_map_file_btn = new QPushButton;
  open_map_file_btn->setText("Open Map File");
  layout->addWidget(open_map_file_btn);

  update_graph_btn_ = new QPushButton;
  update_graph_btn_->setText("Update");
  layout->addWidget(update_graph_btn_);

  layout->addWidget(new QLabel("Tech poses"));

  positions_combo_box_ = new QComboBox;
  layout->addWidget(positions_combo_box_);

  add_target_to_visit_ = new QPushButton;
  add_target_to_visit_->setText("Add point visit list");
  layout->addWidget(add_target_to_visit_);

  start_moving_btn_ = new QPushButton;
  start_moving_btn_->setText("Start moving");
  layout->addWidget(start_moving_btn_);

  stop_moving_btn_ = new QPushButton;
  stop_moving_btn_->setText("Stop moving");
  layout->addWidget(stop_moving_btn_);
  stop_moving_btn_->setEnabled(false);

  layout->addWidget(new QLabel("Robot pos:"));

  current_robot_pos_label_ = new QLabel;
  layout->addWidget(current_robot_pos_label_);
  current_robot_pos_label_->setVisible(false);

  place_robot_btn_ = new QPushButton;
  place_robot_btn_->setText("Place hear");
  layout->addWidget(place_robot_btn_);
  place_robot_btn_->setVisible(false);
  
  techpos_visit_list_ = new QTableWidget(0, 2); // Name, pos (in list)
  techpos_visit_list_->setHorizontalHeaderItem(0, new QTableWidgetItem("Name"));
  techpos_visit_list_->setHorizontalHeaderItem(1, new QTableWidgetItem("In List pos")); // for debug only
  layout->addWidget(techpos_visit_list_);

  setLayout( layout );

  mover_thread = new QThread(this);

  mover = new PosMover;

  connect(this, &PathGraph::proccessToPosition, mover, &PosMover::moveToPosition);

  connect(mover, &PosMover::positionReached, this, &PathGraph::targetReached);

  connect(mover, &PosMover::positionDoesNotReached, this, &PathGraph::stopMoving);

  mover->moveToThread(mover_thread);

  mover_thread->start(QThread::HighPriority);

//   connect( map_topic_editor_, &QLineEdit::editingFinished, this, &PathGraph::updateTopic);

  connect( start_moving_btn_, &QPushButton::clicked, this, &PathGraph::nextPath);

  connect( add_target_to_visit_, &QPushButton::clicked, this, &PathGraph::addTechPosToList);

  connect( stop_moving_btn_, &QPushButton::clicked, this, &PathGraph::stopMoving);

  connect( place_robot_btn_, &QPushButton::clicked, this, &PathGraph::placeRobot);

  connect( update_graph_btn_, &QPushButton::clicked, this, &PathGraph::updateSystem);

  connect( open_map_file_btn, &QPushButton::clicked, this, &PathGraph::openMapFile);

  connect( techpos_visit_list_, &QTableWidget::cellDoubleClicked, this, &PathGraph::removePointFromList);

  // Communication

  tech_position_subsciber_ = nh_.subscribe(CONFIG::TECH_POSES_TOPIC, 1000, &PathGraph::techPosesCallback, this);

  robot_position_subsciber_ = nh_.subscribe(CONFIG::ROBOT_POSITION_TOPIC, 1000, &PathGraph::robotPosCallback, this);

  amcl_position_subsciber_ = nh_.subscribe(CONFIG::AMCL_POSITION_TOPIC, 1000, &PathGraph::robotPosCallback, this);

//   map_subsciber_ = nh_.subscribe(CONFIG::MAP_TOPIC, 1000, &PathGraph::mapDataCallback, this);

  robot_position_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(CONFIG::ROBOT_POSITION_TOPIC, 1000);

  graph_nodes_publisher_ = nh_.advertise<std_msgs::String>(CONFIG::NODES_POSES_TOPIC, 1000);

  path_nodes_publisher_ = nh_.advertise<std_msgs::String>(CONFIG::CURRENT_PATH_NODES_TOPIC, 1000);
}

void PathGraph::removePointFromList(int row, int coll) {
    techpos_visit_list_->removeRow(row);
}

void PathGraph::addTechPosToList() {
    if (!techPoses.isEmpty()) {
        int posToSetTo = positions_combo_box_->currentIndex();

        if(posToSetTo >= techPoses.size()) {
            posToSetTo = 0;
            positions_combo_box_->setCurrentIndex(posToSetTo);
        }

        auto posToPlace = techPoses[posToSetTo];

        techpos_visit_list_->insertRow(techpos_visit_list_->rowCount());

        auto name = new QLabel(positions_combo_box_->currentText());
        techpos_visit_list_->setCellWidget ( techpos_visit_list_->rowCount()-1, 0, name);

        auto inListPosition = new QTableWidgetItem(QString::number(posToSetTo));
        techpos_visit_list_->setItem ( techpos_visit_list_->rowCount()-1, 1, inListPosition);
    }
}

void PathGraph::clearListToVisit() {
    techpos_visit_list_->setRowCount(0);
}

void PathGraph::stopMoving() {
    targetPath.clear();
    start_moving_btn_->setEnabled(true);
    stop_moving_btn_->setEnabled(false);
    publishPathNodes(targetPath);
}

void PathGraph::openMapFile() { 
    QString fileName = QFileDialog::getOpenFileName(this, ("Open Map File"), QStandardPaths::displayName(QStandardPaths::HomeLocation), tr("Map (*.yaml)"));

    if( !fileName.isNull() ) {
        map_topic_editor_->setText(fileName);
        updateTopic();
    }
}

void PathGraph::mapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    // QMessageBox mb;

    auto data = msg.get();

    QString placeHolder("Map [%1 x %2], resolution %3, resault image: [%4 x %5]");

    placeHolder = placeHolder.arg(data->info.width).arg(data->info.height).arg(data->info.resolution);

    float minPass = 0.5; // TEST Only

    cell_size = data->info.resolution;
    gridSize = std::ceil(robot_size / cell_size);

    QImage temp(data->info.width, data->info.height, QImage::Format_ARGB32);
    
    QPainter painter(&temp);

    int pixel = -1;

    for (int i = 0; i < data->info.width; ++i) {
        for (int j = 0; j < data->info.height; ++j) {
            pixel++;
            auto pixelIntensity = data->data[pixel];

            if (pixelIntensity > 65) {
                painter.setPen(Qt::white);
            } else {
                painter.setPen(Qt::black);
            }

            painter.drawPoint(i, j);
        }
    }

    rawImage = temp;

    placeHolder = placeHolder.arg(rawImage.width()).arg(rawImage.height());
}

void PathGraph::techPosesCallback(const std_msgs::String::ConstPtr& msg) {

    clearListToVisit();

    QJsonDocument document = QJsonDocument::fromJson(msg->data.c_str());

    QJsonObject obj = document.object();

    QJsonArray arr = obj["positions"].toArray();

    techPoses.clear();

    positions_combo_box_->clear();

    int pos = 1;

    for (auto item : arr) {
        auto _obj = item.toObject();

        Ogre::Vector3 vec3;

        vec3.x = _obj["x"].toDouble();
        vec3.y = _obj["y"].toDouble();
        vec3.z = _obj["z"].toDouble();

        techPoses.append(vec3);

        QString placeHolder("Position #%1");

        positions_combo_box_->insertItem(pos - 1, placeHolder.arg(pos++));
    }
}

void PathGraph::robotPosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    auto pos = msg.get()->pose.pose.position;

    robotPosX = pos.x;
    robotPosY = pos.y;

    robotPosAMCL = *msg.get();

    QString placeHolder("x: %1, y: %2, z: %3");

    auto rPos = robotPosAMCL.pose.pose.position;

    current_robot_pos_label_->setText(placeHolder.arg(rPos.x).arg(rPos.y).arg(rPos.z));
}

void PathGraph::updateTopic() {
    setTopic( map_topic_editor_->text() );
}

void PathGraph::updateSystem() {
    resetGraph();
    loadDataFromMapFile(lastMap);
    createGraph();
}

CNode* PathGraph::getClosestNode(Ogre::Vector3 pos) {
    CNode* closestNode = graph.toList()[rand() % graph.size()];

    QPointF posPoint = {pos.x, pos.y};
    
    Ogre::Vector3 closestV3;

    closestV3.x = closestNode->getPos().x;
    closestV3.y = closestNode->getPos().y;
    closestV3.z = 0.0;

    closestV3 = convertFromImageToRealPosition(closestV3);

    QPointF closesPoint = {closestV3.x, closestV3.y};

    float d = distance(posPoint, closesPoint);

    for (auto cNode : graph) {

        closestV3.x = cNode->getPos().x;
        closestV3.y = cNode->getPos().y;
        closestV3.z = 0.0;

        closestV3 = convertFromImageToRealPosition(closestV3);

        closesPoint = {closestV3.x, closestV3.y};

        float cd = distance(posPoint, closesPoint);

        if(cd < d) {
            d = cd;
            closestNode = cNode;
        }
    }

    return closestNode;
}

void PathGraph::setTopic( const QString& new_topic )
{
  if( new_topic != map_path_ ) {
    map_path_ = new_topic;

    if( map_path_ == "" ) {
        resetGraph();
    } else {
        loadDataFromMapFile(map_path_);
        createGraph();
    }

    emit configChanged();
  }
}

void PathGraph::nextPath() {
    if(!techPoses.isEmpty()) {

        stop_moving_btn_->setEnabled(true); // just check if we can stop in any time

        if (techpos_visit_list_->rowCount() != 0) {

            // get first position in list;

            auto cell = techpos_visit_list_->item(0, 1); // first row,  second coll.

            if(cell) { 
                int posToSetTo = cell->text().toInt(); // lets check//

                if(posToSetTo < techPoses.size()) {
                    auto posToPlace = techPoses[posToSetTo];

                    Ogre::Vector3 robotPos;
                    robotPos.x = robotPosX;
                    robotPos.y = robotPosY;
                    robotPos.z = 0;

                    moveToPoint(robotPos, posToPlace);

                    moveToNextPosition();
                }
            } else {
                QMessageBox mb;

                mb.setText("cell in nullptr");
                mb.exec();
            }
        }
    }
}

void PathGraph::placeRobot() {
    if (!techPoses.isEmpty()) {
        int posToSetTo = positions_combo_box_->currentIndex();

        auto posToPlace = techPoses[posToSetTo];

        resetRobotPosition(posToPlace.x, posToPlace.y);
    }
}

void PathGraph::moveToPoint(Ogre::Vector3 start, Ogre::Vector3 end) {

    CNode *startNode = getClosestNode(start);
    CNode *endNode = getClosestNode(end);

    if(startNode && endNode) {
        auto rawPath = astar.findPath(startNode, endNode);

        targetPath.clear();

        for (auto step : rawPath) {
            Ogre::Vector3 pos;

            pos.z = 0.0; // Default
            pos.x = step->getPos().x;
            pos.y = step->getPos().y;

            pos = convertFromImageToRealPosition(pos);

            targetPath.append(pos);
        }

        publishPathNodes(targetPath);
    }
}

void PathGraph::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "MapFile", map_path_ );
}

void PathGraph::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "MapFile", &topic ))
  {
    map_topic_editor_->setText( topic );
    updateTopic();
  }
}

void PathGraph::loadDataFromMapFile(const QString& path) {
  try {
    lastMap = path;

    YAML::Node mapConfig = YAML::LoadFile(path.toStdString());

    cell_size = mapConfig["resolution"].as<float>();
    gridSize = std::ceil(robot_size / cell_size);

    const std::string mapFileName = mapConfig["image"].as<std::string>();

    // loading image 

    QFileInfo fileInfo(path);

    QString originalImagePath = fileInfo.absolutePath() + QDir::separator() + QString::fromStdString(mapFileName);

    QImage temp;

    if(temp.load(originalImagePath)) {
      rawImage = temp.convertToFormat(QImage::Format::Format_ARGB32);
    }
  } catch (...) {
    QMessageBox msgBox;
    msgBox.setText("An ERROR occured while opening file...");
    msgBox.exec();
  }
}

void PathGraph::resetGraph() {
    resetRobotPosition();
}

void PathGraph::moveToNextPosition() {

    static bool firstMove = true;

    if(firstMove) {
        start_moving_btn_->setEnabled(false);
        stop_moving_btn_->setEnabled(true);
        firstMove = false;

        // remove first row 
        if(techpos_visit_list_->rowCount() > 0)
            techpos_visit_list_->removeRow(0);
    }

    if(!targetPath.isEmpty()) {
        auto pos = targetPath.last();

        Ogre::Vector3 nPos = {0.0, 0.0, 0.0};

        if(targetPath.size() >= 2) {
            nPos = targetPath[1]; // second
        }

        moveTo(pos, nPos); 
    } else {
        stopMoving();
        firstMove = true;

        if(techpos_visit_list_->rowCount() > 0)
            nextPath();
    }
}

void PathGraph::moveTo(Ogre::Vector3 pos, Ogre::Vector3 next_pos) {
    QVector<double> rPos_ = {
        robotPosAMCL.pose.pose.position.x, 
        robotPosAMCL.pose.pose.position.y, 
        robotPosAMCL.pose.pose.position.z,
        // 
        robotPosAMCL.pose.pose.orientation.x,
        robotPosAMCL.pose.pose.orientation.y,
        robotPosAMCL.pose.pose.orientation.z,
        robotPosAMCL.pose.pose.orientation.w,
        };
    QVector<double> tPos_ = {pos.x, pos.y, pos.z};
    QVector<double> nPos_ = {next_pos.x, next_pos.y, next_pos.z};
    // QMessageBox mb;
    // mb.setText(QString("-> %1, %2, %3").arg(tPos_[0]).arg(tPos_[1]).arg(tPos_[2]));
    // mb.exec();

    emit proccessToPosition(rPos_, tPos_, nPos_);
}

void PathGraph::targetReached() {
    targetPath.removeLast();

    publishPathNodes(targetPath); // just to see wtf is going on

    moveToNextPosition();     
}

void PathGraph::resetRobotPosition(float x, float y) {
    if(ros::ok() && robot_position_publisher_ ) {
        geometry_msgs::PoseWithCovarianceStamped msg;

        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();

        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = 0.0;

        msg.pose.pose.orientation.w = 1.0;

        robot_position_publisher_.publish( msg );
    }
}

void PathGraph::createGraph() {
   try {
    generateNodes();
    generateGraph();
   } catch (...) {
    QMessageBox msgBox;
    msgBox.setText("An ERROR occured while generation graph ");
    msgBox.exec();
   }
}

void PathGraph::generateNodes() {

    nodes.clear();

    nodesImage = rawImage;
    nodes.reserve(1000);

    for (int i = 0; i < nodesImage.width(); i += gridSize) {
        for (int j = 0; j < nodesImage.height(); j += gridSize) {
            // cell center position

            int x = i + gridSize / 2;
            int y = j + gridSize / 2;

            auto cPixel = nodesImage.pixelColor(x, y).rgb();

            if(cPixel != wallColor.rgb() && cPixel != outColor.rgb()) {
                // nodesImage.setPixelColor({x, y}, nodeColor); // remove later
                nodes.insert(QPointF(x, y));
            }
        }
    }

    QSet<QPointF> nodesToRemove;

    for (auto &node : (nodes)) {

        // for each node check if it can reach any other if no, remove

        bool canReach = true;

        int cellX = node.x() - gridSize / 2;
        int cellY = node.y() - gridSize / 2;

        for (int i = cellX; i < cellX + gridSize + 1; ++i) {
            for (int j = cellY; j < cellY + gridSize + 1; ++j) {
                if(rawImage.pixelColor({i,j}).rgb() == wallColor.rgb()){
                    canReach = false;
                    break;
                }
            }
        }

        if(!canReach) nodesToRemove.insert(node);
    }

    nodesImage = rawImage;

    nodes = nodes.subtract(nodesToRemove);

    for (auto &node : (nodes)) {
        nodesImage.setPixelColor({int(node.x()), int(node.y())}, nodeColor);
    }

    publishNodesPositions();
}

void PathGraph::publishNodesPositions() {
    if( ros::ok() && graph_nodes_publisher_ ) {
        std_msgs::String msg;
    
        QJsonObject obj;
        QJsonArray arr;

        for (auto pos : nodes) {
            QJsonObject _obj;
            
            Ogre::Vector3 vec3;

            vec3.z = 0.0; // default
            vec3.x = pos.x(); // need transformation
            vec3.y = pos.y(); // need transformation

            vec3 = convertFromImageToRealPosition(vec3);

            _obj.insert("x", vec3.x);
            _obj.insert("y", vec3.y);
            _obj.insert("z", vec3.z);

            arr.append(_obj);
        }

        obj.insert("nodes", arr);
        QJsonDocument doc(obj);
        msg.data = doc.toJson().toStdString();

        graph_nodes_publisher_.publish( msg );
    }
}

void PathGraph::publishPathNodes(QList<Ogre::Vector3> path) {
    if( ros::ok() && path_nodes_publisher_ ) {
        std_msgs::String msg;
    
        QJsonObject obj;
        QJsonArray arr;

        for (auto pos : path) {
            QJsonObject _obj;
        
            _obj.insert("x", pos.x);
            _obj.insert("y", pos.y);
            _obj.insert("z", pos.z);

            arr.append(_obj);
        }

        obj.insert("path", arr);
        QJsonDocument doc(obj);
        msg.data = doc.toJson().toStdString();

        path_nodes_publisher_.publish( msg );
    }
}

void PathGraph::generateGraph() {
    QHash<QPointF, CNode*> graphNodes; //

    for (auto &node : nodes) {
        auto n = new CNode({float(node.x()), float(node.y())});
        graphNodes.insert({float(node.x()), float(node.y())}, n);
    }

    for (auto &node : nodes) {
        // find and connect every neighbor

        auto nd = graphNodes[{node.x(), node.y()}];

        if(nd) {

            int x = nd->getPos().x;
            int y = nd->getPos().y;

            int off = gridSize;

            QPoint leftTopCoord = {x - off, y - off};
            QPoint leftCoord = {x - off, y};
            QPoint leftBottomCoord = {x - off, y + off};
            QPoint topCoord = {x, y - off};
            QPoint bottomCoord = {x, y + off};
            QPoint rightTopCoord = {x + off, y - off};
            QPoint rightCoord = {x + off, y};
            QPoint rightBottomCoord =  {x + off, y + off};

            if(nodes.contains(leftTopCoord)) {
                if(graphNodes[leftTopCoord])
                    nd->addNeighbor(graphNodes[leftTopCoord]);
            }

            if(nodes.contains(leftCoord)) {
                if(graphNodes[leftCoord])
                    nd->addNeighbor(graphNodes[leftCoord]);
            }

            if(nodes.contains(leftBottomCoord)) {
                if(graphNodes[leftBottomCoord])
                    nd->addNeighbor(graphNodes[leftBottomCoord]);
            }

            if(nodes.contains(topCoord)) {
                if(graphNodes[topCoord])
                    nd->addNeighbor(graphNodes[topCoord]);
            }

            if(nodes.contains(bottomCoord)) {
                if(graphNodes[bottomCoord])
                    nd->addNeighbor(graphNodes[bottomCoord]);
            }

            if(nodes.contains(rightTopCoord)) {
                if(graphNodes[rightTopCoord])
                    nd->addNeighbor(graphNodes[rightTopCoord]);
            }

            if(nodes.contains(rightCoord)) {
                if(graphNodes[rightCoord])
                    nd->addNeighbor(graphNodes[rightCoord]);
            }

            if(nodes.contains(rightBottomCoord)) {
                if(graphNodes[rightBottomCoord])
                    nd->addNeighbor(graphNodes[rightBottomCoord]);
            }
        }
    }

    graphImage = rawImage;

    graph = graphNodes.values().toSet();

    QPainter painter(&graphImage);

    for (auto node : graphNodes) {
        if(node) {
            painter.setPen(graphLine);

            for (auto neighbor : node->getNeighbors()) {
                painter.setPen(graphLine);
                painter.drawLine(node->getPos().x, node->getPos().y, neighbor->getPos().x, neighbor->getPos().y);
            }

            painter.setPen(graphNode);
            painter.drawPoint(QPointF(node->getPos().x, node->getPos().y));
        }
    }

    painter.setPen(robotColor);
    painter.drawPoint(robotPosX, robotPosY);

    astar.graph = graph;
}

float PathGraph::distance(QPointF a, QPointF b)
{
    return std::sqrt(std::abs(
        (b.x() - a.x()) * (b.x() - a.x()) +
        (b.y() - a.y()) * (b.y() - a.y())
    ));
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pf_plugin::PathGraph,rviz::Panel )