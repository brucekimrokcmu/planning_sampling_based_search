#pragma once

#include <chrono>
#include <iostream>
#include <iterator>
#include <math.h>
#include <memory>
#include <unordered_map>
#include <queue>
#include <vector>

#include "FValueCompare.hpp"
#include "Node.hpp"


#define NUMOFDIRS 8

class AStarPlanner {

    public:
        AStarPlanner(); // private ? or public? 
        std::unordered_map<int, double> ComputeBackwardDijkstraHeuristics(double* map, int maxX, int maxY, Node* pgoal);
        std::vector<std::pair<int, int>> AStar(double* map, int maxX, int maxY, Node* pstart, Node* pgoal, std::unordered_map<int, double>* heuristics);

    private:        
        int mdX[NUMOFDIRS] = {-1, -1, -1,  0, 0,  1, 1, 1};
        int mdY[NUMOFDIRS] = {-1,  0,  1, -1, 1, -1, 0, 1}; 
        

        std::vector<Node*> GetOptimalPath(Node* pgoalNode);
        int GetLocation(Node* pnode, int maxX, int maxY);
        int GetLocationFromPose(int x, int y, int maxX, int maxY);
        bool InBoundary(Node* pnode, int maxX, int maxY);
        bool InBoundary(int x, int y, int maxX, int maxY);
        bool IsCollisionFree(Node* pnode, int threshold, double* map, int maxX, int maxY);
        bool IsCollisionFree(int location, int threshold, double* map);

        int CompareNodes(Node* node1, Node* node2);
        double ComputeFValue(double gValue, double hValue);

};