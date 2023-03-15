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
    std::cout<<"Start RRT"<<std::endl;
    std::random_device rd;

    Tree& tree1 = mmyTree;

    for (int k=0; k<mmaxIters; k++) {
        std::vector<double> qRand = SampleRandomVertex(rd);
        State state = ExtendTree(tree1, qRand);
        if (state == State::REACHED) {            
            std::cout<< "Return tree"  << std::endl;
            return mmyTree;
        }
    }

    return mmyTree;
}

std::vector<std::vector<double>> RRTSolver::BuildRRTConnect()
{
    std::vector<std::vector<double>> path;
    std::cout<<"Start RRT Connect"<<std::endl;
    std::random_device rd;

    Tree& tree1 = mmyTree;
    Tree& tree2 = mmyTree2;

    for (int k=0; k<mmaxIters; k++) {
        std::vector<double> qRand = SampleRandomVertex(rd);
        State extendState = ExtendTree(tree1, qRand);

        if (extendState != State::TRAPPED) {            
            std::vector<double> qNew = tree1.GetTree().back()->GetJointPose();
            State connectState = ConnectTree(tree2, qNew);

            //no need to have a goal region... how do I modify extend tree function?

            if (connectState == State::REACHED) {
                std::cout<< "Return tree"  << std::endl;
                path = GetRRTConnectPath(tree1, tree2);
                return path;
            }

        }
        SwapTree(tree1, tree2);
    
    }

    return path;
}

Tree RRTSolver::BuildRRTStar()
{
    std::cout<<"Start RRT Star"<<std::endl;
    std::random_device rd;

    Tree& tree1 = mmyTree;
    
    std::vector<double> qInit = convertToVector(mstartPose, mnumOfDOFs);

    std::shared_ptr<Node> qInitNode = std::make_shared<Node>(-1, qInit);        
    tree1.AddNode(qInitNode);
    qInitNode->SetGValue(0.0);
    qInitNode->SetFValue(qInitNode->GetGValue());

    for (int k=0; k<mmaxIters; k++) {
        std::vector<double> qRand = SampleRandomVertex(rd);
        State state = ExtendAndRewire(tree1, qRand);
        if (state == State::REACHED) {            
            std::cout<< "Return tree"  << std::endl;
            return tree1;
        }
    }

    return tree1;
}

bool RRTSolver::IsObstacleFree(const std::shared_ptr<Node> qStartNode, const std::shared_ptr<Node> qEndNode)
{
    std::vector<double> q; 
    int t=0;
    while(t<mstepIters) {
        for (int i = 0; i < mnumOfDOFs; i++){
            q[i] = qStartNode->GetJointPose()[i] + ((t)/(mstepIters-1))*(qEndNode->GetJointPose()[i] - qStartNode->GetJointPose()[i]);
        }
        if(!IsValidArmConfiguration(q.data(), mnumOfDOFs, mmap, mmaxX, mmaxY)) {
                return false;
        }
        t++;
    }

    return true;
}

std::vector<std::shared_ptr<Node>> RRTSolver::GetNear(Tree& tree, const std::shared_ptr<Node> qNewNode)
{
    // radius = std::min(  ( (gamma/)*(std::log(n)/n)  )  , )
    // TOO MUCH TO TUNE ABOVE

    // SIMPLIFIED APPROACH
    const double R_THRESHOLD = 0.5*PI;
    std::vector<std::shared_ptr<Node>> nearNodes;
    double dist;

    for (const auto& node : tree.GetTree()) {
        dist = tree.ComputeDistance(qNewNode, node);
        if (dist < R_THRESHOLD) {
            nearNodes.push_back(node);
        }
    }
    return nearNodes;
}

double RRTSolver::GetLineCost(const std::shared_ptr<Node> qNode1, const std::shared_ptr<Node> qNode2)
{
    // cost is simply a euclidean distance
    double dist = 0;
    for (int i=0; i<qNode1->GetJointPose().size(); i++){
        dist += (qNode1->GetJointPose()[i] - qNode2->GetJointPose()[i]) * (qNode1->GetJointPose()[i] - qNode2->GetJointPose()[i]);
    }
    return std::sqrt(dist);

}

State RRTSolver::ExtendAndRewire(Tree& tree, const std::vector<double>& qRand) 
{
    std::shared_ptr<Node> qNearestNode = tree.GetNearestNode(qRand);    
    std::pair<State, std::vector<double>> qNewConfig = CheckNewConfig(qRand, qNearestNode);

    if (qNewConfig.first == State::TRAPPED){
        return State::TRAPPED;
    } 
    std::vector<double> qNew = qNewConfig.second;

    //IsObstacleFree <-- already checked Positive in CheckNewConfig()
    std::shared_ptr<Node> qNewNode = std::make_shared<Node>(tree.GetTree().size(), qNewConfig.second);        
    
    tree.AddNode(qNewNode);
    std::vector<std::shared_ptr<Node>> qNearNodes = GetNear(tree, qNewNode);    
    
    std::vector<double> qMin = qNearestNode->GetJointPose();
    int qMinIdx;
    for (auto& qNearNode: qNearNodes) {
        if(IsObstacleFree(qNearNode, qNewNode)){            
            double cost = qNearNode->GetFValue() + GetLineCost(qNearNode, qNewNode); 
            qNewNode->SetGValue(cost);
            qNewNode->SetFValue(qNewNode->GetGValue());
            if (cost < qNewNode->GetFValue()){
                qMin = qNearNode->GetJointPose();
                qMinIdx = qNearNode->GetIndex();
            }
        }
    }
    //Line#13
    qNewNode->SetParent(tree.GetTree()[qMinIdx]); // This is how I understand adding edge. but let's keep see how the rest of the algorithm is like

    for(auto& qNearNode: qNearNodes){
        if (qNearNode->GetIndex() == qMinIdx) {
            continue;
        }
        if(IsObstacleFree(tree.GetTree()[qMinIdx], qNearNode) && qNearNode->GetFValue() > qNewNode->GetFValue() + GetLineCost(qNewNode, qNearNode)){
            qNearNode->SetParent(qNewNode);
        }


    }

    if (qNewConfig.first == State::REACHED) {
        std::shared_ptr<Node> goalNode = std::make_shared<Node>(tree.GetTree().size(), convertToVector(mgoalPose, qRand.size()));
        tree.AddNode(goalNode);
        goalNode->SetParent(qNewNode);
      
        return State::REACHED;
    }
    if (qNewConfig.first == State::ADVANCED){
        // std::cout<< "ADVANCED"  << std::endl;
        return State::ADVANCED;
    }

    // return State::ADVANCED; // not sure about this
}



State RRTSolver::ExtendTree(Tree& tree, const std::vector<double>& qRand) 
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
  
        return State::REACHED;
    }
    if (qNewConfig.first == State::ADVANCED){
        // std::cout<< "ADVANCED"  << std::endl;
        return State::ADVANCED;
    }

    // return State::ADVANCED; // not sure about this
}

State RRTSolver::ConnectTree(Tree& tree, const std::vector<double>& qNew)
{
    State connectState;
    while(true){
        connectState = ExtendTree(tree, qNew);
    
        if (connectState != State::ADVANCED){
            return connectState;
        }
    }
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
            if(CheckGoal(qNew)){
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

bool RRTSolver::CheckGoal(std::vector<double> qNew){

    if (mmyTree.ComputeDistance(qNew, convertToVector(mgoalPose, mnumOfDOFs)) < mgoalTol){
        return true;
    }
	return false;
}

void RRTSolver::SwapTree(Tree& tree1, Tree& tree2)
{
    Tree* temp = &tree1;
    tree1 = tree2;
    tree2 = *temp;
}

std::vector<std::vector<double>> RRTSolver::GetRRTConnectPath(Tree& tree1, Tree& tree2)
{
    
    std::vector<std::vector<double>> path1; 
    std::vector<std::vector<double>> path2; 

    path1 = tree1.GetPath(tree1.GetTree().back());
    path2 = tree2.GetPath(tree2.GetTree().back());
    std::reverse(path2.begin(), path2.end());

    path1.insert(path1.end(), path2.begin(), path2.end());

    return path1;
}
