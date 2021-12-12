#ifndef A_NODE_HPP_
#define A_NODE_HPP_

#include <QSet>
#include <cmath>

// some very specific functions 

/**
 * @brief The SPos struct
 * @details General stracture
 * for storing posints coordinates
 *
 * Also contains heuristic function (static)
 */
struct SPos {
	float x;
	float y;
	
    /**
     * @brief heuristic
     * @param a
     * @param b
     * @return distance betwean two points
     */
    static float heuristic(SPos a, SPos b);
};

/**
 * @brief The CNode class
 * @details Class for stroing graph node
 */
class CNode {

public:
    CNode(SPos pos);
	~CNode();
	
    /**
     * @brief addNeighbor
     * @param neighbor
     *
     * @details This function addes a neighbor
     * for current node
     */
	void addNeighbor(CNode* neighbor = nullptr);

    /**
     * @brief getNeighbors
     * @return a set of neighbors for current node (empty set if no neighbors)
     */
    const QSet<CNode*> &getNeighbors();

    /**
     * @brief getPos
     * @return 2D coordinate of current node
     */
    const SPos getPos();

private:
    /**
     * @brief _neighbors
     * @details a SET of current node`s neighbors
     */
    QSet<CNode*> _neighbors;

    /**
     * @brief _pos
     * @details 2D node position
     */
    SPos _pos;
};


#endif // A_NODE_HPP_

