#include "Graph.hpp"

Graph::Graph(double* map, int maxX, int maxY, int numOfDOFs, int numOfSamples) 
    : mmap(map), mmaxX(maxX), mmaxY(maxY), mnumOfDOFs(numOfDOFs), mnumOfSamples(numOfSamples)
    {
    }

Graph::~Graph()
    {
    }

std::unordered_map<int, Node>& Graph::GetGraph()
{
    return mgraph;
}

std::vector<std::vector<double>>& Graph::GetVertices()
{
    return mvertices;
}

void Graph::AddVertex(int idx, std::vector<double> vertex)
{
    // std::shared_ptr<Node> nodeSmartPtr = std::make_shared<Node>(vertex); 
    // mgraph[vertex] = nodeSmartPtr;
    Node newNode(vertex);
    mgraph[idx] = newNode;
}

bool Graph::IsSameComponent(int idx, Node node)
{
    // Implementation ideas?





    return false;
}

void Graph::AddEdge(int idx, Node node)
{
    mgraph[idx].SetEdge(node);
}
