#pragma once
#include "Node.hpp"
#include <random>
#include <set>
#include "stdlib.h"
#include <unordered_map>
#include "Utils.hpp"


class Graph
{
    public:
        Graph(double* map, int maxX, int maxY, int numOfDOFs, int numOfSamples);
        ~Graph();
        // inline std::unordered_map<std::vector<double>, std::shared_ptr<Node>>& GetGraph(){return mgraph;};
        std::unordered_map<int, Node>& GetGraph();
        std::vector<std::vector<double>>& GetVertices();
        void AddVertex(int idx, std::vector<double> vertex);
        bool IsSameComponent(int idx, Node node);
        void AddEdge(int idx, Node node);

    private:
        double* mmap;
        int mmaxX;
        int mmaxY;
        int mnumOfDOFs;
        int mnumOfSamples;
        std::vector<std::vector<double>> mvertices;
        std::unordered_map<int, Node> mgraph;

    // look up KD tree
};

