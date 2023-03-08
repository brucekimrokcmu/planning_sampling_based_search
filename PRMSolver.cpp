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

std::unique_ptr<Graph> PRMSolver::BuildRoadMap()
{
    std::unique_ptr<Graph> graphSmartPtr(new Graph(mmap, mmaxX, mmaxY, mnumOfDOFs, mnumOfSamples));
    // std::cout<<"Created graphSmartPtr"<<std::endl;
    int i = 0;
    float progress = 0.0;
    while (i < mnumOfSamples)
    {
        int idx = i;
        std::vector<double> vertex = SampleRandomVertex();
        // std::cout<<"Sampled vertex"<<std::endl;
        double* angles = vertex.data();
        if(!IsValidArmConfiguration(angles, mnumOfDOFs, mmap, mmaxX, mmaxY)) {
            // std::cout<<"new vertex conflict!"<<std::endl;
            continue;               
        }
            
        std::shared_ptr<Node> node1SmartPtr = std::make_shared<Node>(idx, vertex);
        graphSmartPtr->AddVertex(node1SmartPtr);
        i++;
        // std::cout<<"i is: "<< i <<"\n" ;
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NeighborCompare> neighbors = GetNearestNeighbor(node1SmartPtr, graphSmartPtr.get());
        if (neighbors.size() == 0){
            continue;            
        }

        while (!neighbors.empty()){
            const std::shared_ptr<Node>& neighborRef = neighbors.top();
            // std::cout<< node1SmartPtr << " " << neighborRef << std::endl;
            neighbors.pop();
            // std::cout << node1SmartPtr->GetComponentID()<< " " << neighborRef->GetComponentID()<<std::endl;;
            if((!graphSmartPtr->IsSameComponent(node1SmartPtr, neighborRef)) || IsConnect(node1SmartPtr, neighborRef)) {
                graphSmartPtr->AddEdge(node1SmartPtr, neighborRef);
            }
        } 
        std::cout<< "node1's edgesize: " << node1SmartPtr->GetEdges().size() <<std::endl;

        if (float(i)/float(mnumOfSamples) > progress)
        {
            // Progress achieved
            std::cout << progress*100 << "%... ";
            progress += 0.1;
        }
        // std::cout<<"neightbor empty!"<<std::endl; 
    }

    std::cout<<"graph size is: "<<graphSmartPtr->GetGraph().size()<<std::endl;
    // for (int i=0; i<graphSmartPtr->GetGraph().size(); i++) {
    //     std::cout<<"Vertex, idx : "<<graphSmartPtr->GetGraph()[i]<< ", "<< graphSmartPtr->GetGraph()[i]->GetIndex() <<" edge: ";
    //     for (auto& edge : graphSmartPtr->GetGraph()[i]->GetEdges()){
    //         std::cout << edge<<", ";
    //     }
    //     std::cout<<std::endl;
    // }

    return graphSmartPtr; // C++ omits the copy operation here.
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
            // std::cout<<ppose.use_count()<<", " << pgraph->GetGraph()[i].use_count() << std::endl;;
            printf("should connect?\n");
            // std::cout<<pgraph->GetGraph()[i]<<std::endl;
            neighbors.push_back(pgraph->GetGraph()[i]);                
            // std::cout<<"pushed to neighbor: "<<dist<<std::endl;
        }
    }
    // std::sort(neighbors.begin(), neighbors.end());
    // for (int i=0; i<neighbors.size();i++){
    //     std::cout<< neighbors[i] << " ";
    // }

    return neighbors[0];
}


void PRMSolver::QueryRoadMap(std::unique_ptr<Graph>& pgraph)
{
    Query query;
    std::vector<std::vector<double>> path;
    std::cout<<"let's get closest startandgoal\n";
    std::shared_ptr<Node> pstart = GetClosestNode(pgraph, mstartPose);   
    std::shared_ptr<Node> pgoal = GetClosestNode(pgraph, mgoalPose);   

    pstart->SetGValue(0.0);   
    pstart->SetFValue(pstart->GetGValue());
    // std::cout<<"start: "<<pstart<< " edge size " << pstart->GetEdges().size()<<std::endl;
    std::cout<<"input goal:  "<<mgoalPose<<std::endl;
    for (int i=0; i<mnumOfDOFs; i++){
        std::cout<<mgoalPose[i] <<" ";
    }
    std::cout<<std::endl;
 
    std::cout<<"goal Node found:  "<<pgoal<< " idx: "<<pgoal->GetIndex() << " edgesize: "<<pgoal->GetEdges().size() <<std::endl;
    for (int i=0; i<pgoal->GetJointPose().size(); i++){
        std::cout<<pgoal->GetJointPose()[i] <<" ";
    }
    std::cout<<std::endl;

    for (int i=0; i<pgraph->GetGraph().size();i++){
        std::cout<< "edge size for node: "<< i << " : "<<pgraph->GetGraph()[i]->GetEdges().size()<<std::endl;
    }



    path = query.Dijkstra(pgraph, pstart, pgoal);

    //Step1. connect double* mstartPose, double* mgoalPose into feasible vertices in graph

    //Step2. Run Dijkstra -> return path 
    
    
    
    
}