#pragma once
#include "stdlib.h"
#include "Node.hpp"
#include "Utils.hpp"

#include <ostream>
#include <random>
#include <set>
#include <unordered_map>


class Graph //: public DisjointSet, public Edge, public Kruskal
{
    public:
        Graph(double* map, int maxX, int maxY, int numOfDOFs, int numOfSamples);
        ~Graph();        
        std::vector<std::shared_ptr<Node>>& GetGraph() {return mgraph;};
        void AddVertex(std::shared_ptr<Node> nodeSmartPtr);
        bool IsSameComponent(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr);
        void AddEdge(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr);
        void UpdateComponentID(int id, std::shared_ptr<Node> pprevNode, std::shared_ptr<Node> pcurrNode);


    private:
        double* mmap;
        int mmaxX;
        int mmaxY;
        int mnumOfDOFs;
        int mnumOfSamples;

        std::vector<std::shared_ptr<Node>> mgraph;

    // look up KD tree
};



// #include "DisjointSet.hpp"
// #include "Edge.hpp"
// #include "Kruskal.hpp"