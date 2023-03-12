#include "RRTSolver.hpp"

RRTSolver::RRTSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, double eps, double stepIters, double goalTol, int maxIters)
    : mmap(map), mmaxX(maxX), mmaxY(maxY), mstartPose(startPos), mgoalPose(goalPos), mnumOfDOFs(numOfDOFs), meps(eps), mstepIters(stepIters), mgoalTol(goalTol), mmaxIters(maxIters)
    {   
        mmyTree.SetRoot(mstartPose, mnumOfDOFs);
        mmyTree2.SetRoot(mgoalPose, mnumOfDOFs);
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

    Tree& tree1 = mmyTree;

    for (int k=0; k<mmaxIters; k++) {
        std::vector<double> qRand = SampleRandomVertex(rd);
        State state = ExtendTree(tree1, qRand, mgoalTol, mmaxIters);
        if (state == State::REACHED) {            
            std::cout<< "Return tree"  << std::endl;
            return mmyTree;
        }
    }

    return mmyTree;
}

// Tree RRTSolver::BuildRRTConnect()
// {
//     std::cout<<"Instantiating RRTSolver class"<<std::endl;
//     std::random_device rd;

//     for (int k=0; k<mmaxIters; k++) {
//         std::vector<double> qRand = SampleRandomVertex(rd);
//         State state = ExtendTree(qRand, mgoalTol, mmaxIters);
        
        
//         if (state == State::REACHED) {            
//             std::cout<< "Return tree"  << std::endl;
//             return mmyTree;
//         }
//     }


//     return mmyTree;
// }

/*
State RRTSolver::ExtendTree(const std::vector<double>& qRand, double goalTol, int maxIters) 
{
    std::shared_ptr<Node> qNearNode = mmyTree.GetNearestNode(qRand);    
    std::pair<State, std::vector<double>> qNewConfig = CheckNewConfig(qRand, qNearNode);

    if (qNewConfig.first == State::TRAPPED){
        return State::TRAPPED;
    } 
    
    std::shared_ptr<Node> qNewNode = std::make_shared<Node>(mmyTree.GetTree().size(), qNewConfig.second);        
    mmyTree.AddNode(qNewNode);
    qNewNode->SetParent(qNearNode);

    if (qNewConfig.first == State::REACHED) {
        std::shared_ptr<Node> goalNode = std::make_shared<Node>(mmyTree.GetTree().size(), convertToVector(mgoalPose, qRand.size()));
        mmyTree.AddNode(goalNode);
        goalNode->SetParent(qNewNode);
        // ///// ERASE BELOW PRINT////////
        // std::cout<< "REACHED.\n GoalNode is: "  << goalNode << " pose: ";
        // printVector(goalNode->GetJointPose(), qRand.size());
        // printf(" qnew: ");
        // printVector(qNewNode->GetJointPose(), qRand.size());
        // ////////////////    
        return State::REACHED;
    }
    if (qNewConfig.first == State::ADVANCED){
        // std::cout<< "ADVANCED"  << std::endl;
        return State::ADVANCED;
    }
    // return State::ADVANCED; // not sure about this
}
*/

State RRTSolver::ExtendTree(Tree& tree, const std::vector<double>& qRand, double goalTol, int maxIters) 
{
    std::shared_ptr<Node> qNearNode = tree.GetNearestNode(qRand);    
    std::pair<State, std::vector<double>> qNewConfig = CheckNewConfig(qRand, qNearNode);

    if (qNewConfig.first == State::TRAPPED){
        return State::TRAPPED;
    } 
    
    std::shared_ptr<Node> qNewNode = std::make_shared<Node>(tree.GetTree().size(), qNewConfig.second);        
    tree.AddNode(qNewNode);
    qNewNode->SetParent(qNearNode);

    if (qNewConfig.first == State::REACHED) {
        std::shared_ptr<Node> goalNode = std::make_shared<Node>(tree.GetTree().size(), convertToVector(mgoalPose, qRand.size()));
        tree.AddNode(goalNode);
        goalNode->SetParent(qNewNode);
        // ///// ERASE BELOW PRINT////////
        // std::cout<< "REACHED.\n GoalNode is: "  << goalNode << " pose: ";
        // printVector(goalNode->GetJointPose(), qRand.size());
        // printf(" qnew: ");
        // printVector(qNewNode->GetJointPose(), qRand.size());
        // ////////////////    
        return State::REACHED;
    }
    if (qNewConfig.first == State::ADVANCED){
        // std::cout<< "ADVANCED"  << std::endl;
        return State::ADVANCED;
    }

    // return State::ADVANCED; // not sure about this
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
        if(IsValidArmConfiguration(qNew.data(), mnumOfDOFs, mmap, mmaxX, mmaxY)){
            if(checkGoal(qNew)){
                return std::make_pair(State::REACHED, qNew);
            }

        } else {
            if (t==1){
                // std::cout<< "Colliding instantly->Trapped?: " << t << std::endl;
                // printVector(qNew, mnumOfDOFs);
                
                return std::make_pair(State::TRAPPED, qNewPrev);
            } else {
                // std::cout<< "progressed t steps: " << t << std::endl;
                return std::make_pair(State::ADVANCED, qNewPrev);      
            }        
        } 
        t++;
    }
    // std::cout<< "progressed t steps: " << t << std::endl;
    return std::make_pair(State::ADVANCED, qNew);
}

bool RRTSolver::checkGoal(std::vector<double> qNew){

    if (mmyTree.ComputeDistance(qNew, convertToVector(mgoalPose, mnumOfDOFs)) < mgoalTol){
        return true;
    }
	return false;
}


