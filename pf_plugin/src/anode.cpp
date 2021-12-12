#include "anode.hpp"

float SPos::heuristic(SPos a, SPos b) {
	return std::sqrt(std::abs(
		(b.x - a.x) * (b.x - a.x) +
		(b.y - a.y) * (b.y - a.y)
	));
}

CNode::CNode(SPos pos) {
	_pos = pos;
}

CNode::~CNode() {
}

void CNode::addNeighbor(CNode* neighbor) {
	if(neighbor) {
		_neighbors.insert(neighbor);
	}
}

const SPos CNode::getPos() {
    return _pos;
}

const QSet<CNode*> &CNode::getNeighbors() {
	return _neighbors;
}
