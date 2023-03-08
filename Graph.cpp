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
    int id = node1SmartPtr->GetComponentID(); 
    if (node1SmartPtr != node2SmartPtr){
        node1SmartPtr->AddEdge(node2SmartPtr);
        UpdateComponentID(id, node1SmartPtr, node2SmartPtr);

        
    }

}


    // if (node1SmartPtr != node2SmartPtr){
    //     node1SmartPtr->AddEdge(node2SmartPtr);
        
    //     if (node2SmartPtr->GetEdges().size()!=0) {
    //         UpdateComponentIDs(node1SmartPtr, node2SmartPtr);
    //     } else {
    //         node2SmartPtr->SetComponentID(node1SmartPtr->GetComponentID());
    //     }

    //     // std::cout<< "node1, idx: " << node1SmartPtr <<", "<<node1SmartPtr->GetIndex() << " node2: " << node2SmartPtr<<std::endl;
    // }
        // if (node2SmartPtr->GetEdges().size()==0) {
        //     node2SmartPtr->SetComponentID(node1SmartPtr->GetComponentID());   
        //     node1SmartPtr->AddEdge(node2SmartPtr);
        // } else{
        //     UpdateComponentIDs(node1SmartPtr, node2SmartPtr);
        //     node1SmartPtr->AddEdge(node2SmartPtr);
        // }



/*
bool Graph::IsSameComponent(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr)
{

    // Run Kruskal algorithm to check if the nodes are connected
    Kruskal kruskal(mgraph);    // Kruskal kruskal(nodes, edges);
    std::vector<Edge> mst = kruskal.GetMST();    

    return kruskal.AreSameConnected(mst, node1SmartPtr, node2SmartPtr);
}
*/