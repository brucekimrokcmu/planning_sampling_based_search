#include "PRMSolver.hpp"

PRMSolver::PRMSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, int numOfSamples, const double maxDist) 
        : mmap(map), mmaxX(maxX), mmaxY(maxY), mstartPose(startPos), mgoalPose(goalPos), mnumOfDOFs(numOfDOFs), mnumOfSamples(numOfSamples), mmaxDist(maxDist)        
        {
        }

PRMSolver::~PRMSolver()
{
}

vector<double> PRMSolver::SampleRandomVertex()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 2*PI);
    std::vector<double> vertex(mnumOfDOFs);
    
    for (int i=0; i<mnumOfDOFs; i++) {   
        vertex[i] = dis(gen);
    }
    
    return vertex;
}

double PRMSolver::ComputeDistance(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr)
{
    double dist=0;
    for (int i=0; i<mnumOfDOFs; i++){

        dist += (node1SmartPtr->GetJointPose()[i] - node2SmartPtr->GetJointPose()[i]) * (node1SmartPtr->GetJointPose()[i] - node2SmartPtr->GetJointPose()[i]);
    }
    dist = std::sqrt(dist);
    return dist;
}

std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NeighborCompare> PRMSolver::GetNearestNeighbor(std::shared_ptr<Node> node1SmartPtr, Graph* pgraph)
{
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NeighborCompare> neighbors;
    double dist;

    for (int i=0; i<pgraph->GetGraph().size(); i++) {
        dist = ComputeDistance(node1SmartPtr, pgraph->GetGraph()[i]);
        if (dist <= mmaxDist && dist > 0) {
            pgraph->GetGraph()[i]->SetTempDist(dist);
            neighbors.push(pgraph->GetGraph()[i]);
            // std::cout<<"Added! dist: " << dist << " maxDist: "<< mmaxDist <<std::endl;
        }
    }

    return neighbors;    
}

bool PRMSolver::IsConnect(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr)
{
    int t=0;
    const int LOCAL_STEPS = 100;

    while (t < LOCAL_STEPS)
    {
        std::vector<double> localPath(mnumOfDOFs);
        for(int i = 0; i < mnumOfDOFs; i++){
            localPath[i] = node1SmartPtr->GetJointPose()[i] + ((double)(t)/(LOCAL_STEPS-1))*std::abs(node2SmartPtr->GetJointPose()[i] - node1SmartPtr->GetJointPose()[i]);
        }    
            
        if(!IsValidArmConfiguration(localPath.data(), mnumOfDOFs, mmap, mmaxX, mmaxY)) {
            // std::cout <<"t: " << t << " collision! DO NOT ADD EDGE" << std::endl;
            return false;
        }
    
        t++;
    }
    // std::cout << "t: "<< t << " you are safe. no collision." << std::endl;
    return true;
}

std::unique_ptr<Graph> PRMSolver::BuildRoadMap()
{
    std::unique_ptr<Graph> graphSmartPtr(new Graph(mmap, mmaxX, mmaxY, mnumOfDOFs, mnumOfSamples));
    // std::cout<<"Created graphSmartPtr"<<std::endl;
    int i = 0;
    while (i < mnumOfSamples)
    {
        int idx = i;
        std::vector<double> vertex = SampleRandomVertex();
        // std::cout<<"Sampled vertex"<<std::endl;
        double* angles = vertex.data();
        if(!IsValidArmConfiguration(angles, mnumOfDOFs, mmap, mmaxX, mmaxY)) 
            // std::cout<<"new vertex conflict!"<<std::endl;
            continue;               

        std::shared_ptr<Node> node1SmartPtr = std::make_shared<Node>(idx, vertex);
        // graphSmartPtr->AddVertex(idx, node1SmartPtr);
        graphSmartPtr->AddVertex(node1SmartPtr);
        i++;

        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NeighborCompare> neighbors = GetNearestNeighbor(node1SmartPtr, graphSmartPtr.get());
        if (neighbors.size() == 0){
            continue;            
        }

        while (!neighbors.empty()){
            const std::shared_ptr<Node>& neighborRef = neighbors.top();
            // std::cout<< node1SmartPtr << " " << neighborRef << std::endl;
            neighbors.pop();

            // std::cout << node1SmartPtr->GetComponentID()<< " " << neighborRef->GetComponentID()<<std::endl;;
            
            if((!graphSmartPtr->IsSameComponent(node1SmartPtr, neighborRef)) || IsConnect(node1SmartPtr, neighborRef))
            // if(!graphSmartPtr->IsSameComponent(node1SmartPtr, neighborRef))
            // if(IsConnect(node1SmartPtr, neighborRef))
                graphSmartPtr->AddEdge(node1SmartPtr, neighborRef);
        }      
        // std::cout<<"neightbor empty!"<<std::endl; 
    }
    std::cout<<"sampling complete"<<std::endl; 
    
    return graphSmartPtr;
}

double* PRMSolver::QueryRoadMap(std::unique_ptr<Graph>& pgraph)
{

}