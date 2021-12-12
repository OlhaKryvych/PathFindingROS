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

    /**
     * @brief findPath
     * @param start node
     * @param end node
     * @return  a list of nodes for shortes path
     */
    QList<CNode*> findPath(CNode* start = nullptr, CNode* end = nullptr);

    /**
     * @brief graph
     *
     * @details current road graph
     */
    QSet<CNode*> graph;

private:

    /**
     * @brief reconstructPath
     * @param cameFrom
     * @param begin
     * @param current
     * @return a list of nodes for founded path
     *
     * @details This function reconstructs founded path between
     * @begin and @current nodes using a set of 'come from' nodes
     */
    QList<CNode *> reconstructPath(QHash<CNode*, CNode*> cameFrom, CNode* begin, CNode* current);

    const int MAX_OP = 10000; // Just a maximum number of iterations (could be set to INF)

    /**
     * @brief lowestInSet
     * @param map
     * @param set
     * @return return a node with lowes set value
     */
    CNode* lowestInSet(QHash<CNode*, float> &map, QSet<CNode*> &set);
};

#endif // ASTAR_HPP
