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

        // use KD Tree for optimization
        std::shared_ptr<Node> GetNearestNode(const std::vector<double> qRand) const 
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



// Using KD Tree
// class Tree 
// {
//     public:
//         typedef std::vector<double> Point;
//         typedef nanoflann::KDTreeSingleIndexAdaptor<
//             nanoflann::L2_Simple_Adaptor<double, Tree>,
//             Tree,
//             mnumOfDOF /* dimension of the space */,
//             int /* index type */
//         > KDTree;

//         Tree(int numOfDOF) 
//         : mnumOfDOF(numOfDOF), mKDTree(numOfDOF, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10)) 
//         {

//         }

//         std::shared_ptr<Node> GetRoot() const { return mRoot; }
//         void SetRoot(std::shared_ptr<Node> root) { mRoot = root; }

//         void AddNode(std::shared_ptr<Node> node) {
//             mNodes.push_back(node);
//             mPoints.push_back(node->GetJointPose());
//             mKDTree.addPoints(mNodes.size()-1, mPoints.back().data());
//         }

//         std::shared_ptr<Node> GetNearestNeighbor(const std::vector<double>& query) const {
//             std::vector<size_t> indices(1);
//             std::vector<double> distances(1);
//             mKDTree.knnSearch(query.data(), 1, indices.data(), distances.data());
//             return mNodes[indices[0]];
//         }

//     private:
        
//         int mnumOfDOF;
//         std::shared_ptr<Node> mRoot;
//         std::vector<std::shared_ptr<Node>> mNodes;
//         std::vector<Point> mPoints;
//         KDTree mKDTree;
// };
