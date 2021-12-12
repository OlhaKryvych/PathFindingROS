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
    /**
     * @brief positionReached
     * @details Emits when target postion was reached by robot
     */
    void positionReached();

    /**
     * @brief positionDoesNotReached
     * @details Emits when target position can not be reached by robot
     */
    void positionDoesNotReached();

public Q_SLOTS:
    /**
     * @brief moveToPosition
     * @param raw_rPos current robot position
     * @param raw_pos target position
     * @param raw_nPos next target position (needed for corect robot aligment)
     *
     * @details Start moving to target position
     */
    void moveToPosition(QVector<double> raw_rPos, QVector<double> raw_pos, QVector<double> raw_nPos);
};

Q_DECLARE_METATYPE(QVector<double>);

#endif
