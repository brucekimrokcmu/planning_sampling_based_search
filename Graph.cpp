#include "Graph.hpp"

Graph::Graph(double* map, int maxX, int maxY, int numOfDOFs, int numOfSamples) 
    : mmap(map), mmaxX(maxX), mmaxY(maxY), mnumOfDOFs(numOfDOFs), mnumOfSamples(numOfSamples)
    {
    }

Graph::~Graph()
    {
    }

void Graph::AddVertex(std::shared_ptr<Node> nodeSmartPtr)
{
    mgraph.push_back(nodeSmartPtr);    
}

bool Graph::IsSameComponent(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr)
{
    if(node1SmartPtr->GetComponentID() == node2SmartPtr->GetComponentID()){
        // std::cout<<"ID Same" <<std::endl;
        return true;
    }
    // std::cout<<"ID DIFFERENT" <<std::endl;
    return false;
}


void Graph::UpdateComponentID(int id, std::shared_ptr<Node> pprevNode, std::shared_ptr<Node> pcurrNode)
{
    // Mark the current node as visited and update ID
    pcurrNode->SetComponentID(id);
    // Recur for all the vertices adjacent to this vertex
    for (auto edge : pcurrNode->GetEdges()){
        if (edge == pprevNode) {
            continue;
        } 
        // std::cout<< "prev, curr, edge: " << pprevNode->GetIndex() << " " << pcurrNode->GetIndex() << " " << edge->GetIndex() <<std::endl;
        UpdateComponentID(id, pcurrNode, edge);
    }
        
}

void Graph::AddEdge(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr)
{
    node1SmartPtr->AddEdge(node2SmartPtr);

}