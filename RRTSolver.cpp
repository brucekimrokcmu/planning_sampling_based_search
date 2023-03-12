#include "RRTSolver.hpp"

RRTSolver::RRTSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, double eps, double stepIters, double goalTol, int maxIters)
    : mmap(map), mmaxX(maxX), mmaxY(maxY), mstartPose(startPos), mgoalPose(goalPos), mnumOfDOFs(numOfDOFs), meps(eps), mstepIters(stepIters), mgoalTol(goalTol), mmaxIters(maxIters)
    {   
        mtree.SetRoot(mstartPose, mnumOfDOFs);
    }

RRTSolver::~RRTSolver()
    {
        
    }


std::vector<double> RRTSolver::SampleRandomVertex(std::random_device& rd)
{
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 2*PI);
    std::vector<double> vertex(mnumOfDOFs);
    
    for (int i=0; i<mnumOfDOFs; i++) {   
        vertex[i] = dis(gen);
        
    }

    return vertex;
}


Tree RRTSolver::BuildRRT()
{
    std::cout<<"Instantiating RRTSolver class"<<std::endl;
    std::random_device rd;
    for (int k=0; k<mmaxIters; k++) {
        std::vector<double> qRand = SampleRandomVertex(rd);
        // std::cout<< "qRand: ";
        // printVector(qRand, mnumOfDOFs);
        State state = ExtendTree(qRand, mgoalTol, mmaxIters);
        if (state == State::REACHED) {
            if (mtree.GetTree().back()->GetJointPose().data() == mgoalPose){
                return mtree;
            }
        }
        
    }

    return mtree;
}

std::pair<State, std::vector<double>> RRTSolver::CheckNewConfig(const std::vector<double>& qRand, const std::shared_ptr<Node> qNearNode)
{
    const double step = meps/mstepIters;    
    std::vector<double> qNewPrev(mnumOfDOFs);
    std::vector<double> qNew = qNearNode->GetJointPose();
    

    double dist = 0.0;
    for (int i=0; i<mnumOfDOFs; i++){
        double diff = qRand[i] - qNearNode->GetJointPose()[i];
        dist += diff*diff;
    }
    dist = std::sqrt(dist);
    
    // if (dist < meps) {}?

    int t=1;
    while(t<mstepIters) 
    {
        qNewPrev = qNew;
        for(int i = 0; i < mnumOfDOFs; i++){
            qNew[i] = qNearNode->GetJointPose()[i] + (qRand[i] - qNearNode->GetJointPose()[i]) * t * step / dist;            
        }
        
        if(!IsValidArmConfiguration(qNew.data(), mnumOfDOFs, mmap, mmaxX, mmaxY)) {
            if (t==1){
                std::cout<< "Colliding instantly->Trapped?: " << t << std::endl;
                printVector(qNew, mnumOfDOFs);
                return std::make_pair(State::TRAPPED, qNewPrev);
            } else {
                // std::cout<< "progressed t steps: " << t << std::endl;
                return std::make_pair(State::ADVANCED, qNewPrev);      
            }        
        } 
        t++;
    }
    // std::cout<< "progressed t steps: " << t << std::endl;
    return std::make_pair(State::NOT_TRAPPED, qNew);
}



State RRTSolver::ExtendTree(const std::vector<double>& qRand, double goalTol, int maxIters) 
{
    std::shared_ptr<Node> qNearNode = mtree.GetNearestNode(qRand);    
    // std::cout<< "qnear   : ";
    // printVector(qNearNode->GetJointPose(), mnumOfDOFs);
    std::pair<State, std::vector<double>> qNewConfig = CheckNewConfig(qRand, qNearNode);

    if (qNewConfig.first == State::TRAPPED){
        // printf("State: TRAPPED\n");
        // std::cout<< "qrand   : ";
        // printVector(qRand, mnumOfDOFs); 
        // std::cout<< "qNear   : ";
        // printVector(qNearNode->GetJointPose(), mnumOfDOFs);
       
        
        return State::TRAPPED;

    }else if (qNewConfig.first == State::ADVANCED){
        // add config before being trapped to the mtree
        std::shared_ptr<Node> qNewPrevNode = std::make_shared<Node>(mtree.GetTree().size(), qNewConfig.second);        
        mtree.AddNode(qNewPrevNode);
        qNewPrevNode->SetParent(qNearNode);
        // mtree->AddEdge(qNearNode, qNewPrevNode);        
        // std::cout<< "qNear   : ";
        // printVector(qNearNode->GetJointPose(), mnumOfDOFs);
        // std::cout<< "qNewPrev: ";
        // printVector(qNewPrevNode->GetJointPose(), mnumOfDOFs);
        printf("State: ADVANCED\n\n");
        return State::ADVANCED;
    }
    std::vector<double> qNew = qNewConfig.second;
    std::shared_ptr<Node> qNewNode = std::make_shared<Node>(mtree.GetTree().size(), qNew);    
    mtree.AddNode(qNewNode);
    qNewNode->SetParent(qNearNode);
    // mtree->AddEdge(qNearNode, qNewNode);


    if (mtree.ComputeDistance(qNew, qRand) < goalTol) { //goalTol used as regular tolerance at here
        
        // std::cout<< "qNear   : ";
        // printVector(qNearNode->GetJointPose(), mnumOfDOFs);
        // std::cout<< "qNew   : ";
        // printVector(qNewNode->GetJointPose(), mnumOfDOFs);
        printf("State: computing distance. REACHED\n\n");
        return State::REACHED;
    } else {
        printf("State: ADVANCED?\n\n");
        return State::ADVANCED;
    }

}

