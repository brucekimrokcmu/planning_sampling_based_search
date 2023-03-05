#include "Graph.hpp"

Graph::Graph(double* map, int maxX, int maxY, int numOfDOFs, int numOfSamples) 
    : mmap(map), mmaxX(maxX), mmaxY(maxY), mnumOfDOFs(numOfDOFs), mnumOfSamples(numOfSamples)
    {
    }

Graph::~Graph()
    {
    }

// std::unordered_map<int, Node>& Graph::GetGraph()

void Graph::AddVertex(int idx, std::shared_ptr<Node> nodeSmartPtr)
{
    mgraph[idx] = nodeSmartPtr;

    // std::cout<<"vertex added!"<<std::endl; 
}

void Graph::AddVertex(std::shared_ptr<Node> nodeSmartPtr)
{
    mgraph.push_back(nodeSmartPtr);    
}

/*
bool Graph::IsSameComponent(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr)
{

    // Run Kruskal algorithm to check if the nodes are connected
    Kruskal kruskal(mgraph);    // Kruskal kruskal(nodes, edges);
    std::vector<Edge> mst = kruskal.GetMST();    

    return kruskal.AreSameConnected(mst, node1SmartPtr, node2SmartPtr);
}
*/

bool Graph::IsSameComponent(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr)
{
    // std::cout<<"node1: " << node1SmartPtr->GetComponentID() << " node2: " << node2SmartPtr->GetComponentID() <<std::endl;
    
    if(node1SmartPtr->GetComponentID() == node2SmartPtr->GetComponentID()){
        // std::cout<<"ID Same" <<std::endl;
        return true;
    }
    // std::cout<<"ID DIFFERENT" <<std::endl;
    return false;
}

void UpdateComponentIDs(std::shared_ptr<Node>& node1SmartPtr, std::shared_ptr<Node> node2SmartPtr) {

    if (node2SmartPtr == node2SmartPtr->GetEdge()){
        std::cout << "ERROR: Node points to itself. Good luck!" << std::endl;
    }
    
    if (node2SmartPtr->GetEdge() == nullptr) {
        return;
    }
    // std::cout<< "update componentID" << std:: endl;
    // std::cout<< node2SmartPtr<< " " << node2SmartPtr->GetEdge() << " : " <<node2SmartPtr.use_count() << std::endl;
    node2SmartPtr->SetComponentID(node1SmartPtr->GetComponentID());
    UpdateComponentIDs(node1SmartPtr, node2SmartPtr->GetEdge());
}



void Graph::AddEdge(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr)
{
    node1SmartPtr->SetEdge(node2SmartPtr);
    UpdateComponentIDs(node1SmartPtr, node2SmartPtr);
}
