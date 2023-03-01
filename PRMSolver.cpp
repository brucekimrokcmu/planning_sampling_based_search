#include "PRMSolver.hpp"

PRMSolver::PRMSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, int numOfSamples, double maxDist) 
        : mmap(map), mmaxX(maxX), mmaxY(maxY), mstartPose(startPos), mgoalPose(goalPos), mnumOfDOFs(numOfDOFs), mnumOfSamples(numOfSamples), mmaxDist(maxDist)        
        {
            // double** mrandomRoadMap = InitRandomVertices();
            // double* mgaussianRoadMap = BuildGaussianRoadMap();
        }

PRMSolver::~PRMSolver()
{
}

vector<double> PRMSolver::SampleRandomVertex()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 2*PI);
    std::vector<double> vertex;


    for (int i=0; i<mnumOfDOFs; i++) {
        vertex[i] = dis(gen);
    }
        
    return vertex;
}

double PRMSolver::ComputeDistance(std::vector<double> vertex, Node node2)
{
    double dist=0;
    for (int i=0; i<mnumOfDOFs; i++){
        dist += std::sqrt(vertex[i]*vertex[i] + node2.GetJointPose()[i]*node2.GetJointPose()[i]);
    }

    return dist;
}

std::priority_queue< Node, std::vector<Node>, NeighborCompare> PRMSolver::GetNearestNeighbor(std::vector<double> vertex, Graph* pgraph)
{
    std::priority_queue< Node, std::vector<Node>, NeighborCompare> neighbors;
    double dist;
    //Bruteforce graph to compute distance 
    for (auto& g : pgraph->GetGraph()) {
        dist = ComputeDistance(vertex, g.second);
        if (dist <= mmaxDist){
            g.second.SetTempDist(dist);
            neighbors.push(g.second);
        } 
    }

    return neighbors;    
}

bool PRMSolver::IsConnect(std::vector<double> vertex, Node node)
{
    int localSteps = 100;
    std::vector<double> localPath(mnumOfDOFs);
    for(int i = 0; i < mnumOfDOFs; i++){
        localPath[i] = localPath[i] + ((double)(i)/(localSteps-1))*(node.GetJointPose()[i] - vertex[i]);
    }    
    if(!IsValidArmConfiguration(localPath.data(), mnumOfDOFs, mmap, mmaxX, mmaxY)) {
		return false;
    }

    return true;
}

void PRMSolver::BuildRoadMap()
{
    std::unique_ptr<Graph> graphSmartPtr(new Graph(mmap, mmaxX, mmaxY, mnumOfDOFs, mnumOfSamples));
    
    int i = 0;
    while (i < mnumOfSamples)
    {
        int idx = i;
        std::vector<double> vertex = SampleRandomVertex();
        double* angles = vertex.data();
        if(!IsValidArmConfiguration(angles, mnumOfDOFs, mmap, mmaxX, mmaxY))
            continue;               
        
        graphSmartPtr->GetVertices().emplace_back(vertex); 
        graphSmartPtr->AddVertex(idx, vertex);
        i++;
 
        std::priority_queue<Node, std::vector<Node>, NeighborCompare> neighbors = GetNearestNeighbor(vertex, graphSmartPtr.get());
 
        while (!neighbors.empty()){
            const Node& neighborRef = neighbors.top();
            neighbors.pop();
            if((!graphSmartPtr->IsSameComponent(idx, neighborRef)) || !IsConnect(vertex, neighborRef))
                continue;
            graphSmartPtr->AddEdge(idx, neighborRef);
        }
        
    }
    

}