#pragma once

#include <algorithm>
#include <memory>
#include <ostream>
#include <queue>
#include <unordered_map>
#include <vector>

#include "NeighborCompare.hpp"
#include "Graph.hpp"
#include "Query.hpp"

#define PI 3.141592654

class PRMSolver
{

    public:
        PRMSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, int numOfSamples, const double maxDist);
        ~PRMSolver();  

        std::unique_ptr<Graph> InitializeGraph();
        void BuildRoadMap(std::unique_ptr<Graph>& pgraph);
        // std::vector<std::vector<double>> QueryRoadMap(std::unique_ptr<Graph>& pgraph);
        std::vector<std::vector<double>> QueryRoadMap(std::unique_ptr<Graph>& pgraph);
        std::unordered_map<int, std::vector<std::shared_ptr<Node>>> mcomponentMap;

    private:
        double* mmap;
        int mmaxX;
        int mmaxY;
        double* mstartPose;
        double* mgoalPose;  
        int mnumOfDOFs;
        int mnumOfSamples;
        // std::unordered_map<int, double> mheuristics;
        double mmaxDist;

        
        std::vector<double> SampleRandomVertex();
        double ComputeDistance(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr);
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NeighborCompare> GetNearestNeighbor(std::shared_ptr<Node> node1SmartPtr, Graph* pgraph);
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NeighborCompare> GetKNumberOfNeighbor(std::shared_ptr<Node> node1SmartPtr, Graph* pgraph);
        bool IsConnect(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr);
        
        std::shared_ptr<Node> GetClosestNode(std::unique_ptr<Graph>& pgraph, double* pose);
        //  SearchRoadMap();
        

};