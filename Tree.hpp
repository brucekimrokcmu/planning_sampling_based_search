#pragma once
#include "Node.hpp"
// #include "nanoflann/include/nanoflann.hpp"
#include <memory>
#include <random>
#include <vector>
#include "Utils.hpp"

class Tree 
{
    public:
        Tree() {};

        void SetRoot(double* startPose, int numOfDOF) 
        {
            std::shared_ptr<Node> root = std::make_shared<Node> (-1, convertToVector(startPose, numOfDOF));
            mtree.push_back(root);
        }

        void AddNode(std::shared_ptr<Node> node) { mtree.push_back(node); }
        void AddEdge(std::shared_ptr<Node> parent, std::shared_ptr<Node> child) {parent->AddEdge(child);}
        std::shared_ptr<Node> GetRoot() const { return mroot;}
        std::vector<std::shared_ptr<Node>> GetTree() const { return mtree; }

        double ComputeDistance(const std::shared_ptr<Node> qRand, const std::shared_ptr<Node> pnode) const 
        {
            double dist = 0;
            for (int i=0; i<qRand->GetJointPose().size(); i++){
                dist += (qRand->GetJointPose()[i] - pnode->GetJointPose()[i]) * (qRand->GetJointPose()[i] - pnode->GetJointPose()[i]);
            }
            return std::sqrt(dist);
        }
        
        double ComputeDistance(const std::vector<double> qRand, const std::shared_ptr<Node> pnode) const 
        {
            double dist = 0;
            for (int i=0; i<qRand.size(); i++){
                dist += (qRand[i] - pnode->GetJointPose()[i]) * (qRand[i] - pnode->GetJointPose()[i]);
            }
            return std::sqrt(dist);
        }

        double ComputeDistance(const std::vector<double> pnode1, const std::vector<double> pnode2) const 
        {
            double dist = 0;
            for (int i=0; i<pnode1.size(); i++){
                dist += (pnode1[i] - pnode2[i]) * (pnode1[i] - pnode2[i]);
            }
            return std::sqrt(dist);
        }

        std::shared_ptr<Node> GetNearestNode(const std::vector<double> qRand) const  // can optimize using KD Tree
        {
            std::shared_ptr<Node> nearestNode;
            double minDist = std::numeric_limits<double>::infinity();
            
            for (const auto& node : mtree) {
                double dist = ComputeDistance(qRand, node);
                if (dist < minDist) {
                    nearestNode = node;
                    minDist = dist;
                }
            }
            return nearestNode;
        }

        std::vector<std::vector<double>> GetPath(std::shared_ptr<Node> pgoal) const 
        {
            std::vector<std::vector<double>> path;
            path.push_back(pgoal->GetJointPose());
            //////////            
            printf("input goal node: ");
            printVector(pgoal->GetJointPose(), pgoal->GetJointPose().size());
            ///////////
            std::shared_ptr<Node> pparent = pgoal->GetParent();
            while (pparent != nullptr) {                
                path.push_back(pparent->GetJointPose());
                pparent = pparent->GetParent();
            }
            std::reverse(path.begin(), path.end());
            
            return path;
        }

    private:        
        std::shared_ptr<Node> mroot;
        std::vector<std::shared_ptr<Node>> mtree;
    
};
