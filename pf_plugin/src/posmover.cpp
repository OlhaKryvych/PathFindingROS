#include "posmover.hpp"

#include "config.hpp"

#include <QTimer>
#include <QMessageBox>

void PosMover::moveToPosition(QVector<double> raw_robotPosAMCL, QVector<double> raw_pos, QVector<double> raw_nPos) {
    geometry_msgs::PoseWithCovarianceStamped robotPosAMCL;
    robotPosAMCL.pose.pose.position.x = raw_robotPosAMCL[0];
    robotPosAMCL.pose.pose.position.y = raw_robotPosAMCL[1];
    robotPosAMCL.pose.pose.position.z = raw_robotPosAMCL[2];

    Ogre::Vector3 pos;

    pos.x = raw_pos[0];
    pos.y = raw_pos[1];
    pos.z = raw_pos[2];

    Ogre::Vector3 nPos;

    nPos.x = raw_nPos[0];
    nPos.y = raw_nPos[1];
    nPos.z = raw_nPos[2];

    MoveBaseClient _mbc("move_base", true);

    while(!_mbc.waitForServer(ros::Duration(1.0))) {
        ROS_INFO("Waiting for move_base server");
    }

    move_base_msgs::MoveBaseGoal goal;

    Ogre::Vector3 rPos;

    rPos.x = robotPosAMCL.pose.pose.position.x;
    rPos.y = robotPosAMCL.pose.pose.position.y;
    rPos.z = robotPosAMCL.pose.pose.position.z;

    // Calculation a differance vector between target goal and current robot position
    auto _diff = pos - rPos;

    // QMessageBox mb;
    // mb.setText(QString("Pos to move: %1, %2, %3").arg(pos.x).arg(pos.y).arg(pos.z));
    // mb.exec();

    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = _diff.x;
    goal.target_pose.pose.position.y = _diff.y;
    goal.target_pose.pose.position.z = 0.0;

    // need to calculate 

    auto _q = getQuaternionOfVectors(pos, nPos); // WARNING

    // goal.target_pose.pose.orientation = _q; // ?

    // goal.target_pose.pose.orientation.x = _q.x; 
    // goal.target_pose.pose.orientation.y = _q.y; 
    // goal.target_pose.pose.orientation.z = _q.z; 

    goal.target_pose.pose.orientation.w = 1.0; // was 1.0 -> 0.1

    _mbc.sendGoal(goal);
    _mbc.waitForResult(ros::Duration(CONFIG::MOVE_TIMER_INTERVAL_MS / 1000));

    if(_mbc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        emit positionReached();
    else
        emit positionDoesNotReached();
}
