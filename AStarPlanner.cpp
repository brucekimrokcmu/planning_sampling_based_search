#include "AStarPlanner.hpp"

AStarPlanner::AStarPlanner(){};

std::unordered_map<int, double> AStarPlanner::ComputeBackwardDijkstraHeuristics(double* map, int maxX, int maxY, Node* pgoal) 
{
    std::unordered_map<int, double> heuristicsTable;
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList;     
    std::unordered_map<int, Node*> closedList;
    std::unordered_map<int, Node*> visitedList;
    
    pgoal->SetCurrentTime(0);
    pgoal->SetGValue(0.0);
    pgoal->SetFValue(pgoal->GetGValue());
    openList.push(pgoal);
        
    while((!openList.empty())) {
        // printf("open list size: %d\n", openList.size());
        // printf("closed list size: %d\n", closedList.size());

        Node* pparentNode = openList.top();
        openList.pop();

        visitedList[GetLocation(pparentNode, maxX, maxY)] = pparentNode;
        closedList[GetLocation(pparentNode, maxX, maxY)] = pparentNode;

        for (int dir=0; dir<NUMOFDIRS; dir++){ 
            // Calculate new position of node
            int newX = pparentNode->GetPoseX() + mdX[dir];
            int newY = pparentNode->GetPoseY() + mdY[dir];
            int newLocation = GetLocationFromPose(newX, newY, maxX, maxY);
            // Check if the new poses are within map
            if (!InBoundary(newX, newY, maxX, maxY))
                continue;
            // Check if the new poses are collision free?
            // if (!IsCollisionFree(newLocation, threshold, map))
            //     continue;
            
            Node* psuccNode = new Node(newX, newY);
                                
            psuccNode->SetHeuristics(0.0);                
            if (visitedList.find(newIndex) != visitedList.end()){ // If visited                     
                // printf("Visited.\n");

                if (closedList.find(newIndex) == closedList.end()){  //If not inside the closde list
                    Node* pexistingNode = visitedList.at(newIndex);
                    
                    if(pexistingNode->GetFValue() > pparentNode->GetGValue()+mmap[newIndex]){
                        psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                        psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), 0, 0));
                        psuccNode->SetParent(pparentNode);
                        openList.push(psuccNode);
                        visitedList[newIndex] = psuccNode;
                        // printf("updated closed list and pushed to openlist.\n\n");                   
                    } 
                }                    
            } else { // If NOT visited 
            
                visitedList[newIndex]=psuccNode;
                // printf("Not visited\n");
                // Always add the node to the open list since this is the first time seeing it          
                psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), 0, 0));                
                psuccNode->SetParent(pparentNode);
                openList.push(psuccNode);   
            }
        }            
    }

    // printf("D* exits while loop\n");
    // loop through closedlist, openlist and delete all elements    
    for (auto i=closedList.begin(); i != closedList.end();i++) {
        heuristicsTable[GetNodeIndex(i->second)] = (i->second)->GetGValue();
        // printf("stores heuristics table from closed list\n");
        delete i->second;
    }
    
    while (!openList.empty()){
        heuristicsTable[GetNodeIndex(openList.top())] = openList.top()->GetGValue();
        // printf("stores heuristics table from open list\n");
        delete openList.top();
        openList.pop(); // deallocates memory
    }
    // printf("D* ran;\n");
    return heuristicsTable;
}


std::vector<std::pair<int, int>> AStarPlanner::AStar(double* map, Node* start, Node* goal, std::unordered_map<int, double>* heuristics)
{
    std::vector<std::pair<int, int>> path;
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 
    std::unordered_map<int, Node*> closedList;
    std::unordered_map<int, Node*> visitedList;
    
    
    Node* root = start;
    root->SetGValue(0.0);
    root->SetHeuristics(GetLocation(start));    
    root->SetFValue(ComputeFValue(root->GetGValue(), root->GetHeuristics()));
    openList.push(root);     
    
    while((!openList.empty())) {
        Node* pparentNode = openList.top();  
        openList.pop();    
        
        visitedList[GetNodeIndex(pparentNode)] = pparentNode;
        closedList[GetNodeIndex(pparentNode)] = pparentNode;
       
        if (GetNodeIndex(pparentNode) == GetIndexFromPose(targetGoalPose.first, targetGoalPose.second)) {

            Node* pgoalNode = new Node(targetGoalPose.first, targetGoalPose.second, pparentNode->GetCurrentTime()+1);
            Node* p = pparentNode;
            while (p != nullptr) {
                path.push_back(std::make_pair(p->GetPoseX(), p->GetPoseY()));
                p = p->GetParent();    
            }
            std::reverse(path.begin(), path.end());

            while(!openList.empty()){
                delete openList.top();
                openList.pop();
            }

            delete pgoalNode;
            break;// printf("Not visited\n");
        }    

        for (int dir=0; dir<NUMOFDIRS; dir++){ 
            int newX = pparentNode->GetPoseX() + mdX[dir];
            int newY = pparentNode->GetPoseY() + mdY[dir];
            int newIndex = GetIndexFromPose(newX, newY);

            if (!inBounds(newX, newY))
                continue;

            if (!isCollisionFree(newIndex))
                continue;
            
            Node* psuccNode = new Node(newX, newY, currTime+1);   
                           
            psuccNode->SetHeuristics((*pg_DHueristics)[GetNodeIndex(psuccNode)]);                
            if (visitedList.find(newIndex) != visitedList.end()){ // If visited                     
                // printf("Visited.\n");
                if (closedList.find(newIndex) == closedList.end()){  //If inside the closde list
                    Node* pexistingNode = visitedList.at(newIndex);
                    
                    if(pexistingNode->GetFValue() > pparentNode->GetGValue()+mmap[newIndex]+weight*(psuccNode->GetHeuristics())){
                        psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                        psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));
                        psuccNode->SetParent(pparentNode);
                        openList.push(psuccNode);  
                        visitedList[newIndex] = psuccNode; 
                        // printf("updated visited list and pushed to openlist.\n\n");                  
                    }                         
                } 
            } else { // If NOT visited 
                // printf("Not visited\n");
                visitedList[newIndex]=psuccNode;                            
                psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));                
                psuccNode->SetParent(pparentNode);
                openList.push(psuccNode);                                         
                // printf("pushed to open list.\n\n");                                                     
            }                        
        }                     
    }

   for (auto i=closedList.begin(); i != closedList.end();i++){
        delete i->second;
    }
    
    return path;

}

std::vector<std::pair<int, int>> FindPath::MultigoalAStar(Node startNode, int currTime, int targetTime)
{
    std::vector<std::pair<int, int>> path;
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 
    std::unordered_map<int, Node*> closedList;
    std::unordered_map<int, Node*> goalList; // a separate goallist     
    std::unordered_map<int, Node*> visitedList;
    //create a goallist
    for (int t=0; t<mtargetSteps;t++){
        Node* pgoalNode = new Node((int)mtargetTrajectory[t], (int)mtargetTrajectory[t+mtargetSteps], t);
        goalList[GetNodeIndex(pgoalNode)] = pgoalNode;    
    }
    
    double weight = 200.0;    
    Node* pstartNode = new Node(startNode);
    pstartNode->SetGValue(0.0);
    pstartNode->SetHeuristics(ComputeEuclideanHeuristics(pstartNode, goalList[GetIndexFromPose((int)mtargetTrajectory[0], (int)mtargetTrajectory[mtargetSteps])]));
    pstartNode->SetFValue(ComputeFValue(pstartNode->GetGValue(), pstartNode->GetHeuristics(), weight));
    openList.push(pstartNode); 
    // printf("initialized pstartNode and pushed into openlist.\n");
    // check time -> extract goal from the trajectory
    // I can also check the elapsed time running AStar and add that to goal time.
    
    while((!openList.empty())) {
        if(IsCellValid(openList.top())){
            Node* pparentNode = openList.top();  
            openList.pop();
            
            // printf("parent node x y t: %d %d %d\n", pparentNode->GetPoseX(), pparentNode->GetPoseY(), pparentNode->GetCurrentTime());
            visitedList[GetNodeIndex(pparentNode)] = pparentNode;
            closedList[GetNodeIndex(pparentNode)] = pparentNode;
            // check if parentnode == every single goal nodes along with time 
            // pparentnode.time <= goal time 
            currTime = pparentNode->GetCurrentTime();
            // targetTime >= currTime
            Node* ptargetGoalNode = goalList[GetIndexFromPose((int)mtargetTrajectory[targetTime], (int)mtargetTrajectory[targetTime+mtargetSteps])];
            // printf("goal   node x y t: %d %d %d\n", ptargetGoalNode->GetPoseX(), ptargetGoalNode->GetPoseY(), ptargetGoalNode->GetCurrentTime());
            // printf("popped openlist\n");
            // printf("ptargetgoal created.\n");
            
            if (GetNodeIndex(pparentNode) == GetNodeIndex(ptargetGoalNode)) {
                printf("parent meets goal\n");
                ptargetGoalNode->SetParent(pparentNode);
                
                Node* p = ptargetGoalNode->GetParent();
                while (p != nullptr) {
                    path.push_back(std::make_pair(p->GetPoseX(), p->GetPoseY()));
                    p = p->GetParent();    
                }
                std::reverse(path.begin(), path.end());
                
                // for (int i=0; i<3; i++){
                //     printf("next pose x y: %d %d \n", path[i].first, path[i].second);
                // }

                while(!openList.empty()){
                    delete openList.top();
                    // printf("deallocating memory from openlist\n");
                    openList.pop();
                }
                break;
            }    

            for (int dir=0; dir<NUMOFDIRS; dir++){ 
                int newX = pparentNode->GetPoseX() + mdX[dir];
                int newY = pparentNode->GetPoseY() + mdY[dir];
                int newIndex = GetIndexFromPose(newX, newY);
                // printf("num of succ: %d\n", dir);
                if (newX >= 1 && newX <= mxSize && newY >= 1 && newY <= mySize) {    
                    if (((int)mmap[newIndex] >= 0) && ((int)mmap[newIndex] < mcollisionThresh)) {
                        Node* psuccNode = new Node(newX, newY, currTime+1);   
                        // printf("psucc node x y t: %d %d %d;\n", psuccNode->GetPoseX(), psuccNode->GetPoseY(), psuccNode->GetCurrentTime());
                        if (IsCellValid(psuccNode)) {                            
                            psuccNode->SetHeuristics(ComputeEuclideanHeuristics(psuccNode, ptargetGoalNode));                
                            if (visitedList.find(newIndex) != visitedList.end()){ // If visited                     
                                // printf("Visited.\n");
                                if (closedList.find(newIndex) != closedList.end()){  //If inside the closde list
                                Node* pexistingNode = closedList.at(newIndex);
                                    if(pexistingNode->GetFValue() > pparentNode->GetGValue()+mmap[newIndex]+weight*(psuccNode->GetHeuristics())){
                                        psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                                        psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));
                                        psuccNode->SetParent(pparentNode);
                                        openList.push(psuccNode);   
                                        // printf("updated closed list and pushed to openlist.\n\n");                   
                                        
                                        
                                    } 
                                     
                                } else {
                                    if (psuccNode->GetGValue()> pparentNode->GetGValue()+mmap[newIndex]){                
                                        psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                                        psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));                
                                        psuccNode->SetParent(pparentNode);
                                        openList.push(psuccNode);     
                                    
                                    
                                    // printf("pushed to open list.\n\n");                
                                    }
                                }                    
                            } else { // If NOT visited 
                                visitedList[newIndex]=psuccNode;
                                // printf("Not visited\n");
                                if (psuccNode->GetGValue()> pparentNode->GetGValue()+mmap[newIndex]){                
                                    psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                                    psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));                
                                    psuccNode->SetParent(pparentNode);
                                    openList.push(psuccNode);     
                                    
                                    
                                    // printf("pushed to open list.\n\n");                
                                }                     
                            }            
                        } 
                        else {
                            printf("Invalid succNode.\n");
                            continue;
                        }
                    } else {
                        continue;
                    }
                } else {
                    continue;
                }
            }
            
        }
    }

    // printf("exists while loop\n");
    // loop through closedlist, openlist, goallist and delete all elements    

    for (auto i=goalList.begin(); i != goalList.end();i++){
        delete i->second;
    }
    // printf("goal list is deleted\n");

   for (auto i=closedList.begin(); i != closedList.end();i++){
        delete i->second;
    }
    // printf("closed list is deleted\n");
    // for (auto i=visitedList.begin(); i != visitedList.end();i++) {
    //     delete i->second;
    // }
    // printf("visited list is deleted\n");


    return path;
}

std::vector<std::pair<int, int>> FindPath::AStarwithMultiBackwardDijkstra(Node startNode, int currTime, std::unordered_map<int, double>* pg_DHueristics, int targetTime)
{
    std::vector<std::pair<int, int>> path;
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 
    std::unordered_map<int, Node*> closedList;
    std::unordered_map<int, std::pair<int, int>> goalList;     
    std::unordered_map<int, Node*> visitedList;
    //create a goallist
    for (int t=0; t<mtargetSteps;t++){
        goalList[t] = std::make_pair((int)mtargetTrajectory[t], (int)mtargetTrajectory[t+mtargetSteps]);    
    }
    
    
    double weight = mcollisionThresh/2;    
    Node* pstartNode = new Node(startNode);
    pstartNode->SetGValue(0.0);
    pstartNode->SetHeuristics((*pg_DHueristics)[GetNodeIndex(pstartNode)]);    
    pstartNode->SetFValue(ComputeFValue(pstartNode->GetGValue(), pstartNode->GetHeuristics(), weight));
    openList.push(pstartNode);     
    std::pair<int, int> targetGoalPose = goalList[targetTime];
    
    while((!openList.empty())) {
    
        Node* pparentNode = openList.top();  
        if(!IsCellValid(openList.top()))
            continue;
    
        openList.pop();    
        visitedList[GetNodeIndex(pparentNode)] = pparentNode;
        closedList[GetNodeIndex(pparentNode)] = pparentNode;
       
        if (GetNodeIndex(pparentNode) == GetIndexFromPose(targetGoalPose.first, targetGoalPose.second)) {

            Node* pgoalNode = new Node(targetGoalPose.first, targetGoalPose.second, pparentNode->GetCurrentTime()+1);
            Node* p = pparentNode;
            while (p != nullptr) {
                path.push_back(std::make_pair(p->GetPoseX(), p->GetPoseY()));
                p = p->GetParent();    
            }
            std::reverse(path.begin(), path.end());

            while(!openList.empty()){
                delete openList.top();
                openList.pop();
            }

            delete pgoalNode;
            break;// printf("Not visited\n");
        }    

        for (int dir=0; dir<NUMOFDIRS; dir++){ 
            int newX = pparentNode->GetPoseX() + mdX[dir];
            int newY = pparentNode->GetPoseY() + mdY[dir];
            int newIndex = GetIndexFromPose(newX, newY);
            
            if (!inBounds(newX, newY))
                continue;

            if (!isCollisionFree(newIndex))
                continue;
            
            Node* psuccNode = new Node(newX, newY, currTime+1);   
                           
            psuccNode->SetHeuristics((*pg_DHueristics)[GetNodeIndex(psuccNode)]);                
            if (visitedList.find(newIndex) != visitedList.end()){ // If visited                     
                // printf("Visited.\n");
                if (closedList.find(newIndex) == closedList.end()){  //If inside the closde list
                    Node* pexistingNode = visitedList.at(newIndex);
                    
                    if(pexistingNode->GetFValue() > pparentNode->GetGValue()+mmap[newIndex]+weight*(psuccNode->GetHeuristics())){
                        psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                        psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));
                        psuccNode->SetParent(pparentNode);
                        openList.push(psuccNode);  
                        visitedList[newIndex] = psuccNode; 
                        // printf("updated visited list and pushed to openlist.\n\n");                  
                    }                         
                } 
            } else { // If NOT visited 
                // printf("Not visited\n");
                visitedList[newIndex]=psuccNode;                            
                psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));                
                psuccNode->SetParent(pparentNode);
                openList.push(psuccNode);                                         
                // printf("pushed to open list.\n\n");                                                     
            }                        
        }                     
    }

   for (auto i=closedList.begin(); i != closedList.end();i++){
        delete i->second;
    }
    
    return path;
}

/*######################################################################################################################################*/
/*######################################################################################################################################*/
/*######################################################################################################################################*/
/*######################################################################################################################################*/
/*######################################################################################################################################*/

int AStarPlanner::GetLocation(Node* pnode, int maxX, int maxY) 
{
    int x = pnode->GetPoseX();
    int y = pnode->GetPoseY();
    return ((y-1)*maxX + (x-1));

}

int AStarPlanner::GetLocationFromPose(int x, int y, int maxX, int maxY)
{
    return x >= 1 && x <= maxX && y >= 1 && y <= maxY;
}

bool AStarPlanner::InBoundary(Node* pnode, int maxX, int maxY)
{
    int x = pnode->GetPoseX();
    int y = pnode->GetPoseY();

    return x >= 1 && x <= maxX && y >= 1 && y <= maxY;
}

bool AStarPlanner::InBoundary(int x, int y, int maxX, int maxY)
{
    return x >= 1 && x <= maxX && y >= 1 && y <= maxY;
}

bool AStarPlanner::IsCollisionFree(Node* pnode, int threshold, double* map, int maxX, int maxY)
{
    int idx = GetLocation(pnode, maxX, maxY);
    return map[idx] >= 0 && map[idx] < threshold;
}

bool AStarPlanner::IsCollisionFree(int location, int threshold, double* map)
{
    return map[location] >= 0 && map[location] < threshold;
}

std::vector<Node*> FindPath::GetOptimalPath(Node* pgoalNode)
{
    std::vector<Node*> path;

    path.push_back(pgoalNode);

    Node* pparent = pgoalNode->GetParent();
    while(pparent != nullptr){

        pparent = pparent->GetParent();
        path.push_back(pparent);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

int FindPath::GetNodeIndex(Node node)
{
    // if(!IsCellValid(node)){
    //     return
    // }
    int x = node.GetPoseX();
    int y = node.GetPoseY();
    return ((y-1)*mxSize + (x-1));
}

int FindPath::GetNodeIndex(Node* pnode)
{
    int x = pnode->GetPoseX();
    int y = pnode->GetPoseY();
    return ((y-1)*mxSize + (x-1));
}




bool FindPath::IsCellValid(Node node)
{
    int x = node.GetPoseX();
    int y = node.GetPoseY();
    int index = GetIndexFromPose(x,y);
    if (x >= 1 && x <= mxSize && y >= 1 && y <= mySize) {    
        if (((int)mmap[index] >= 0) && ((int)mmap[index] < mcollisionThresh)) {
            return true;
        }
    }
    return false;
}

bool FindPath::IsCellValid(Node* pnode)
{
    int x = pnode->GetPoseX();
    int y = pnode->GetPoseY();
    int index = GetIndexFromPose(x,y);
    if (x >= 1 && x <= mxSize && y >= 1 && y <= mySize) {    
        if (((int)mmap[index] >= 0) && ((int)mmap[index] < mcollisionThresh)) {
            return true;
        }
    }
    return false;
}

bool inline FindPath::InBounds(int x, int y)
{
    return x >= 1 && x <= mxSize && y >= 1 && y <= mySize;
}

bool inline FindPath::IsCollisionFree(int idx)
{
    return mmap[idx] >= 0 && mmap[idx] < mcollisionThresh;
}

double FindPath::ComputeEuclideanHeuristics(Node node, Node goalNode)
{
    int goalX = goalNode.GetPoseX();
    int goalY = goalNode.GetPoseY();
    int x = node.GetPoseX();
    int y = node.GetPoseY();

    double euclideanHeuristics = (double)std::sqrt(((goalX - x)*(goalX-x) + (goalY - y)*(goalY-y)));

    return euclideanHeuristics;
}

double FindPath::ComputeEuclideanHeuristics(Node* pnode, Node* pgoalNode)
{
    int goalX = pgoalNode->GetPoseX();
    int goalY = pgoalNode->GetPoseY();
    int x = pnode->GetPoseX();
    int y = pnode->GetPoseY();

    double euclideanHeuristics = (double)std::sqrt(((goalX - x)*(goalX-x) + (goalY - y)*(goalY-y)));

    return euclideanHeuristics;
}

double FindPath::ComputeFValue(double gValue, double heuristics, double weight)
{
    double fValue = gValue + heuristics*weight;
    return fValue;
}
