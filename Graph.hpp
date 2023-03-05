#pragma once
#include "stdlib.h"
// #include "DisjointSet.hpp"
// #include "Edge.hpp"
// #include "Kruskal.hpp"
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
        // std::unordered_map<int, std::shared_ptr<Node>>& GetGraph() {return mgraph;};
        std::vector<std::shared_ptr<Node>>& GetGraph() {return mgraph;};
        void AddVertex(int idx, std::shared_ptr<Node> nodeSmartPtr);
        void AddVertex(std::shared_ptr<Node> nodeSmartPtr);
        bool IsSameComponent(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr);
        void AddEdge(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr);



    private:
        double* mmap;
        int mmaxX;
        int mmaxY;
        int mnumOfDOFs;
        int mnumOfSamples;
        // std::unordered_map<int, std::shared_ptr<Node>> mgraph;
        std::vector<std::shared_ptr<Node>> mgraph;


    // look up KD tree
};

