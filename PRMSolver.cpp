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
        // std::cout<<vertex[i] << " ";
    }
    // std::cout<<std::endl;
    return vertex;
}

double PRMSolver::ComputeDistance(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr)
{
    double dist=0;
    for (int i=0; i<mnumOfDOFs; i++){

        dist += (node1SmartPtr->GetJointPose()[i] - node2SmartPtr->GetJointPose()[i]) * (node1SmartPtr->GetJointPose()[i] - node2SmartPtr->GetJointPose()[i]);
        // std::cout<<"each dist: " << dist << std::endl;
    }
    dist = std::sqrt(dist);
    // std::cout<<"final dist: " << dist << std::endl;
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


std::unique_ptr<Graph> PRMSolver::InitializeGraph()
{
    std::unique_ptr<Graph> graphSmartPtr(new Graph(mmap, mmaxX, mmaxY, mnumOfDOFs, mnumOfSamples));

    int i = 0;
    while(i<mnumOfSamples){
        int idx = i;
        std::vector<double> vertex = SampleRandomVertex();
        double* angles = vertex.data();
        if(!IsValidArmConfiguration(angles, mnumOfDOFs, mmap, mmaxX, mmaxY)){
            continue;
        }
        std::shared_ptr<Node> node1SmartPtr = std::make_shared<Node>(idx, vertex);
        graphSmartPtr->AddVertex(node1SmartPtr);
        i++;
    }

    return graphSmartPtr;

}


void PRMSolver::BuildRoadMap(std::unique_ptr<Graph>& pgraph)
{

    // std::cout<<"Created graphSmartPtr"<<std::endl;
    int i = 0;
    float progress = 0.0;
    while (i < mnumOfSamples)
    {
        // int idx = i;
        // std::vector<double> vertex = SampleRandomVertex();
        // // std::cout<<"Sampled vertex"<<std::endl;
        // double* angles = vertex.data();
        // if(!IsValidArmConfiguration(angles, mnumOfDOFs, mmap, mmaxX, mmaxY)) {
        //     // std::cout<<"new vertex conflict!"<<std::endl;
        //     continue;               
        // }
            
        // std::shared_ptr<Node> node1SmartPtr = std::make_shared<Node>(idx, vertex);
        // pgraph->AddVertex(node1SmartPtr);

        // std::cout<<"i is: "<< i <<"\n" ;
        std::shared_ptr<Node> node1SmartPtr = pgraph->GetGraph()[i];
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NeighborCompare> neighbors = GetNearestNeighbor(node1SmartPtr, pgraph.get());
        // std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NeighborCompare> neighbors = GetKNumberOfNeighbor(node1SmartPtr, graphSmartPtr.get());
        
        while (!neighbors.empty()){
            const std::shared_ptr<Node>& neighborRef = neighbors.top();
            // std::cout<< node1SmartPtr << " " << neighborRef << std::endl;
            neighbors.pop();
            int node1ID = node1SmartPtr->GetComponentID();
            int neighborID = neighborRef->GetComponentID();

            mcomponentMap[node1ID].push_back(node1SmartPtr);
            // std::cout << node1SmartPtr->GetComponentID()<< " " << neighborRef->GetComponentID()<<std::endl;;
            if((!pgraph->IsSameComponent(node1SmartPtr, neighborRef)) || IsConnect(node1SmartPtr, neighborRef)) {
                if (node1SmartPtr == neighborRef) {
                    continue;
                }
                // graphSmartPtr->AddEdge(node1SmartPtr, neighborRef);                    
                if (mcomponentMap[node1ID].size() >= mcomponentMap[neighborID].size()){
                    node1SmartPtr->AddEdge(neighborRef);
                    for (auto edge : neighborRef->GetEdges()){
                        edge->SetComponentID(node1ID);
                    }
                    mcomponentMap.erase(neighborID);
                } else {
                    neighborRef->AddEdge(node1SmartPtr);
                    for (auto edge : node1SmartPtr->GetEdges()){
                        edge->SetComponentID(neighborID);
                    } 
                    mcomponentMap.erase(node1ID);
                }
                
            }
        } 
        // std::cout<< "node1's edgesize: " << node1SmartPtr->GetEdges().size() <<std::endl;
        if (float(i)/float(mnumOfSamples) > progress)
        {
            // Progress achieved
            std::cout << progress*100 << "%... ";
            progress += 0.1;
        }
        // std::cout<<"neightbor empty!"<<std::endl; 
        i++;
    }

    std::cout<<"graph size is: "<<pgraph->GetGraph().size()<<std::endl;
    // for (int i=0; i<graphSmartPtr->GetGraph().size(); i++) {
    //     std::cout<<"Vertex, idx : "<<graphSmartPtr->GetGraph()[i]<< ", "<< graphSmartPtr->GetGraph()[i]->GetIndex() <<" edge: ";
    //     for (auto& edge : graphSmartPtr->GetGraph()[i]->GetEdges()){
    //         std::cout << edge<<", ";
    //     }
    //     std::cout<<std::endl;
    // }

}

inline std::vector<double> ArrayToVector(double* arr, size_t arr_len) {
    return std::vector<double>(arr, arr + arr_len);
}

std::shared_ptr<Node> PRMSolver::GetClosestNode(std::unique_ptr<Graph>& pgraph, double* pose)
{
    std::vector<double> pos = ArrayToVector(pose, mnumOfDOFs);   
    std::shared_ptr<Node> ppose = std::make_shared<Node>(-1, pos);
    std::vector<std::shared_ptr<Node>> neighbors;

    double dist;
    const double DIST_THRESHOLD = 4*PI/3;
    // std::cout<<ppose<<std::endl;
    //This takes a lot of time!!
    for (int i=0; i<pgraph->GetGraph().size(); i++) {
        dist = ComputeDistance(ppose, pgraph->GetGraph()[i]);
        // std::cout<<pgraph->GetGraph()[i]<<std::endl;
        // std::cout<<"dist btw start and node is: "<<dist<<std::endl;
        if (dist <= DIST_THRESHOLD && dist > 0) {
            if (!IsConnect(ppose, pgraph->GetGraph()[i])) {
                continue;   
            }

            if (pgraph->GetGraph()[i]->GetEdges().size() == 0) {
                continue;
            }

            neighbors.push_back(pgraph->GetGraph()[i]);                

        }
    }

    return neighbors[0];
}


std::vector<std::vector<double>> PRMSolver::QueryRoadMap(std::unique_ptr<Graph>& pgraph)
{
    Query query;
    std::vector<std::vector<double>> path;
    std::shared_ptr<Node> pstart = GetClosestNode(pgraph, mstartPose);   
    std::shared_ptr<Node> pgoal = GetClosestNode(pgraph, mgoalPose);   

    pstart->SetGValue(0.0);   
    pstart->SetFValue(pstart->GetGValue());

    std::cout << "start id: " << pstart->GetIndex() << " goal id: " << pgoal->GetIndex() << std::endl;

    path = query.Dijkstra(pgraph, pstart, pgoal);


    std::vector<double> mstartPoseVector = convertToVector(mstartPose, mnumOfDOFs);
    std::vector<double> mgoalPoseVector = convertToVector(mgoalPose, mnumOfDOFs);
    
    path.insert(path.begin(), mstartPoseVector);
    path.insert(path.end(), mgoalPoseVector);

    std::cout<< "path size: "<<path.size()<<std::endl;

    for (int i=0; i<path.size(); i++){
        for (int j=0; j<path[i].size();j++){
            std:cout<<path[i][j]<<" ";
        }
        std::cout<<std::endl;
    }

    return path;
    
    
    
}