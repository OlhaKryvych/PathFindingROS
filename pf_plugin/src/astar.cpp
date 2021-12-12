#include "astar.hpp"

AStar::AStar()
{

}

QList<CNode*> AStar::findPath(CNode *start, CNode *end)
{
    QHash<CNode*, float> gScore;
    QHash<CNode*, float> fScore;
    QHash<CNode*, float> hScore;

    QSet<CNode*> openSet;

    openSet.insert(start);

    QHash<CNode*, CNode*> cameFrom;

    for (auto n : graph) {
        gScore.insert(n, std::numeric_limits<float>::max());
        fScore.insert(n, std::numeric_limits<float>::max());
    }

    gScore[start] = 0;
    fScore[start] = SPos::heuristic(start->getPos(), end->getPos());

    while (!openSet.isEmpty()) {
        CNode* current = lowestInSet(fScore, openSet);

        if(current == end) {
            return reconstructPath(cameFrom, start, current);
        }

        openSet.remove(current);

        for (auto nei : current->getNeighbors()) {
            float tg = gScore[current] + SPos::heuristic(current->getPos(), nei->getPos());

            if(tg < gScore[nei]) {
                cameFrom.insert(nei, current);
                gScore[nei] = tg;
                fScore[nei] = gScore[nei] + SPos::heuristic(nei->getPos(), end->getPos());

                if(!openSet.contains(nei)) {
                    openSet.insert(nei);
                }
            }
        }
    }

    return {};
}


QList<CNode *> AStar::reconstructPath(QHash<CNode*, CNode*> cameFrom, CNode* begin, CNode *current)
{
    QList<CNode *> path;
    path.push_back(current);

    while (cameFrom.keys().contains(current)) {
        current = cameFrom[current];
        path.push_back(current);
    }

    return path;
}

CNode *AStar::lowestInSet(QHash<CNode *, float> &map, QSet<CNode*> &set) {
    CNode* res = nullptr;

    float cv = std::numeric_limits<float>::max();

    for (auto n : set) {
        if(map.contains(n)) {
            if(map[n] < cv) {
                cv = map[n];
                res = n;
            }
        }
    }

    return res;
}
