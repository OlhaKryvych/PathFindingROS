#ifndef A_NODE_HPP_
#define A_NODE_HPP_

#include <QSet>
#include <cmath>

// some very specific functions 

struct SPos {
	float x;
	float y;
	
    static float heuristic(SPos a, SPos b);
};

class CNode {

public:
    CNode(SPos pos);
	~CNode();
	
	void addNeighbor(CNode* neighbor = nullptr);
    const QSet<CNode*> &getNeighbors();
    const SPos getPos();

private:
    QSet<CNode*> _neighbors;
    SPos _pos;
};


#endif // A_NODE_HPP_

