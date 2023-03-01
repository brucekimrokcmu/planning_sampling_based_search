#pragma once
// #include "AStarPlanner.hpp"
#include <algorithm>
#include <memory>
#include "NeighborCompare.hpp"
#include "Node.hpp"
#include "Graph.hpp"
#include <queue>

#define PI 3.141592654

class PRMSolver{

    public:
        PRMSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, int numOfSamples, double maxDist);
        ~PRMSolver();  

        void BuildRoadMap(); 

        // double* QueryRoadMap();

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
        double ComputeDistance(std::vector<double> vertex, Node node2);
        std::priority_queue<Node, std::vector<Node>, NeighborCompare> GetNearestNeighbor(std::vector<double> vertex, Graph* pgraph);
        bool IsConnect(std::vector<double> vertex, Node node);
        
        //  SearchRoadMap();
        

};