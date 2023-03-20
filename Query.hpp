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
#include "Graph.hpp"
#include "Node.hpp"

class Query {

    public:
        Query();
        std::vector<std::vector<double>> Dijkstra(std::unique_ptr<Graph>& pgraph, std::shared_ptr<Node> startSmartPtr, std::shared_ptr<Node> goalSmartPtr);
        std::unordered_map<int, double> ComputeBackwardDijkstraHeuristics(double* map, int maxX, int maxY, Node* pgoal);
        std::vector<std::pair<int, int>> AStar(double* map, int maxX, int maxY, Node* pstart, Node* pgoal, std::unordered_map<int, double>* heuristics);

    private:        
        double ComputeFValue(double gValue, double hValue, double weight);
        std::vector<std::vector<double>> GetOptimalPath(std::shared_ptr<Node> pgoal);
};