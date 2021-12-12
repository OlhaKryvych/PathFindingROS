#ifndef POS_MOVER_HPP
#define POS_MOVER_HPP

#include <QObject>
#include <OgreVector3.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "utils.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PosMover : public QObject {
    Q_OBJECT

public:
    PosMover(QObject *parent = nullptr) : QObject(parent) {}
    ~PosMover() {}

Q_SIGNALS:
    void positionReached();
    void positionDoesNotReached();

public Q_SLOTS:
    void moveToPosition(QVector<double> raw_rPos, QVector<double> raw_pos, QVector<double> raw_nPos);
};

Q_DECLARE_METATYPE(QVector<double>);

#endif