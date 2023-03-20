#include "Query.hpp"

Query::Query(){};

inline double ComputeDistance(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr)
{
    double dist=0;
    int numOFDOFs = node1SmartPtr->GetJointPose().size();
    for (int i=0; i<numOFDOFs; i++){
        dist += (node1SmartPtr->GetJointPose()[i] - node2SmartPtr->GetJointPose()[i]) * (node1SmartPtr->GetJointPose()[i] - node2SmartPtr->GetJointPose()[i]);
    }
    dist = std::sqrt(dist);
    return dist;
}

std::vector<std::vector<double>> Query::GetOptimalPath(std::shared_ptr<Node> pgoal)
{
    std::vector<std::vector<double>> path;

    path.push_back(pgoal->GetJointPose());
    std::shared_ptr<Node> pparent = pgoal->GetParent();

    while(pparent != nullptr){
        path.push_back(pparent->GetJointPose());
        pparent = pparent->GetParent();
    }
    std::reverse(path.begin(), path.end());

    return path;
}

double Query::ComputeFValue(double gValue, double heuristics, double weight)
{
    double fValue = gValue + heuristics*weight;
    return fValue;
}

std::vector<std::vector<double>> Query::Dijkstra(std::unique_ptr<Graph>& pgraph, std::shared_ptr<Node> startSmartPtr, std::shared_ptr<Node> goalSmartPtr)
{
    std::cout<<"Dijkstra starts"<<std::endl;

    std::vector<std::vector<double>> path;

    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, FValueCompare> openList;     
    std::unordered_map<int, std::shared_ptr<Node>> closedList;
    std::unordered_map<int, std::shared_ptr<Node>> visitedList;
    
    double weight = 1.0;    
    openList.push(startSmartPtr);

    if (startSmartPtr == goalSmartPtr) {
        std::cout<<"ERROR: Sampled start and goal nodes are same!!!"<<std::endl; // A way to improve: throw runtime error
    }
    while(!openList.empty()) {
        const std::shared_ptr<Node> pparent = openList.top(); 
        openList.pop();
        int parentIdx = pparent->GetIndex();
        visitedList[parentIdx] = pparent;
        closedList[parentIdx] = pparent;
        
        if(pparent->GetEdges().size()==0){
            // std::cout<<"skipping this parent as edge is nullptr\n";
            continue;
        } 
        
        for (auto pchild : pparent->GetEdges()){                                                 
            int childIdx = pchild->GetIndex();
            if (visitedList.find(childIdx) != visitedList.end()) {
                if (closedList.find(childIdx) == closedList.end()) { 
                    std::shared_ptr<Node>& pvisited = visitedList.at(childIdx);

                    if(pvisited->GetFValue() > pparent->GetGValue()+ComputeDistance(pparent, pchild)){
                        pchild->SetGValue(pparent->GetGValue()+ComputeDistance(pparent, pchild));
                        pchild->SetFValue(ComputeFValue(pchild->GetGValue(), 0, weight));
                        pchild->SetParent(pparent);
                        openList.push(pchild);
                        visitedList[childIdx] = pchild;
                    } 
                }                    
            } else { 
                visitedList[childIdx] = pchild;
                pchild->SetGValue(pparent->GetGValue()+ComputeDistance(pparent, pchild));
                pchild->SetFValue(ComputeFValue(pchild->GetGValue(), 0, weight));                
                pchild->SetParent(pparent);
                openList.push(pchild);   

            }
            
        }            

    }

    path = Query::GetOptimalPath(goalSmartPtr);
    
    return path;
}

