#pragma once
// #include "AStarPlanner.hpp"
#include <algorithm>
#include <memory>
#include <ostream>
#include <queue>

#include "NeighborCompare.hpp"
#include "Graph.hpp"

#define PI 3.141592654

class PRMSolver{

    public:
        PRMSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, int numOfSamples, const double maxDist);
        ~PRMSolver();  

        std::unique_ptr<Graph> BuildRoadMap();
        double* QueryRoadMap(std::unique_ptr<Graph>& pgraph);

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
        bool IsConnect(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr);
        
        //  SearchRoadMap();
        

};