#include "RRTSolver.hpp"

RRTSolver::RRTSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, double eps, double goalTol, int maxIters)
    : mmap(map), mmaxX(maxX), mmaxY(maxY), mstartPose(startPos), mgoalPose(goalPos), mnumOfDOFs(numOfDOFs), meps(eps), mgoalTol(goalTol), mmaxIters(maxIters)
    {   
    }
RRTSolver::~RRTSolver()
    {
        
    }


std::vector<double> RRTSolver::SampleRandomVertex()
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




std::unique_ptr<Tree> RRTSolver::BuildRRT()
{
    
    std::unique_ptr<Tree> ptree(new Tree(std::make_shared<Node>(-1, mstartPose)));
    
    int k = 0;
    const int K_THRESHOLD = 1000;
    while (k<K_THRESHOLD) {
        std::vector<double> qRand = SampleRandomVertex();
        State state = ExtendTree(ptree, qRand, meps, mgoalTol, mmaxIters);
        if (state == State::REACHED) {
            if (ptree->GetTree().back()->GetJointPose().data() == mgoalPose){
                return ptree;
            }
        }
    }

    return ptree;
}

std::pair<State, std::vector<double>> RRTSolver::CheckNewConfig(const std::vector<double>& qRand, const std::shared_ptr<Node> qNearNode, double eps)
{
    int t=0;
    std::vector<double> qNewPrev(mnumOfDOFs);
    std::vector<double> qNew(mnumOfDOFs);
    
    while (t < eps)    
    {
        qNewPrev = qNew;
        for(int i = 0; i < mnumOfDOFs; i++){
            qNew[i] = qNearNode->GetJointPose()[i] + ((double)(t)/(eps-1))*std::abs(qRand[i] - qNearNode->GetJointPose()[i]);
        }    
        if(t = 0 && !IsValidArmConfiguration(qNew.data(), mnumOfDOFs, mmap, mmaxX, mmaxY)) {
            return std::make_pair(State::TRAPPED, qNewPrev);
        } else if (t > 0 && !IsValidArmConfiguration(qNew.data(), mnumOfDOFs, mmap, mmaxX, mmaxY)) {
            return std::make_pair(State::ADVANCED, qNewPrev);
        }

        t++;
    }
    // std::cout << "t: "<< t << " you are safe. no collision." << std::endl;

    return std::make_pair(State::NOT_TRAPPED, qNew);
}

State RRTSolver::ExtendTree(std::unique_ptr<Tree>& ptree, const std::vector<double>& qRand, double eps, double goalTol, int maxIters) 
{
    std::unique_ptr<Tree> tree = std::move(ptree);
    std::shared_ptr<Node> qNearNode = tree->GetNearestNode(qRand);    
    std::pair<State, std::vector<double>> qNewConfig = CheckNewConfig(qRand, qNearNode, eps);
    if (qNewConfig.first == State::TRAPPED){
        return State::TRAPPED;

    }else if (qNewConfig.first == State::ADVANCED){
        // add config before being trapped to the tree
        std::shared_ptr<Node> qNewPrevNode = std::make_shared<Node>(tree->GetTree().size(), qNewConfig.second);        
        tree->AddNode(qNewPrevNode);
        qNewPrevNode->SetParent(qNearNode);
        // tree->AddEdge(qNearNode, qNewPrevNode);        
        return State::ADVANCED;
    }
    std::vector<double> qNew = qNewConfig.second;
    std::shared_ptr<Node> qNewNode = std::make_shared<Node>(tree->GetTree().size(), qNew);    
    tree->AddNode(qNewNode);
    qNewNode->SetParent(qNearNode);
    // tree->AddEdge(qNearNode, qNewNode);

    if (tree->ComputeDistance(qNew, qRand) < goalTol) { //goalTol used as regular tolerance at here
        return State::REACHED;
    } else {
        return State::ADVANCED;
    }

}






    // int iters = 1;
    // while (iters < maxIters) {

    //     tree->AddNode(qNewNode);
    //     tree->AddEdge(qNear_node, qNewNode);
    //     if (tree->ComputeDistance(qNewNode, std::make_shared<Node>(tree->GetRoot()->GetJointPose())) < goalTol) {
    //         throw std::runtime_error("Goal reached");
    //     }
    //     qNear_node = qNewNode;

    //     iters++;
    // }