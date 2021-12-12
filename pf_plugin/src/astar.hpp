#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "anode.hpp"

#include <QSet>
#include <QList>
#include <QHash>

class AStar
{
public:
    AStar();

    QList<CNode*> findPath(CNode* start = nullptr, CNode* end = nullptr);

    QSet<CNode*> graph;

private:

    QList<CNode *> reconstructPath(QHash<CNode*, CNode*> cameFrom, CNode* begin, CNode* current);

    const int MAX_OP = 10000; // Just a maximum number of iterations (could be set to INF)

    CNode* lowestInSet(QHash<CNode*, float> &map, QSet<CNode*> &set);
};

#endif // ASTAR_HPP
