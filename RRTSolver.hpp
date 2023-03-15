#pragma once
#include <algorithm>
#include <memory>
#include <ostream>
#include <queue>
#include <unordered_map>
#include <vector>
#include "Tree.hpp"
#include "NeighborCompare.hpp"
#include "Utils.hpp"

enum class State {TRAPPED, REACHED, ADVANCED};

class RRTSolver
{

    public:
        RRTSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, double eps, double stepSize, double goalTol, int maxIters);
        ~RRTSolver();  


        std::vector<double> SampleRandomVertex(std::random_device& rd);
        Tree BuildRRT();
        std::vector<std::vector<double>> BuildRRTConnect();
        Tree BuildRRTStar();
        State ExtendAndRewire(Tree& tree, const std::vector<double>& qRand);

        // std::vector<double> GetNearest(Tree& tree, const std::vector<double>& qRand);
        bool IsObstacleFree(const std::shared_ptr<Node> qStartNode, const std::shared_ptr<Node> qEndNode);
        std::vector<std::shared_ptr<Node>> GetNear(Tree& tree, const std::shared_ptr<Node> qNewNode);
        double GetLineCost(const std::shared_ptr<Node> qNode1, const std::shared_ptr<Node> qNode2);


        std::pair<State, std::vector<double>> CheckNewConfig(const std::vector<double>& qRand, const std::shared_ptr<Node> qNearNode);
        State ExtendTree(Tree& tree, const std::vector<double>& qRand);
        State ConnectTree(Tree& tree, const std::vector<double>& qNew);
        // State ExtendTree(const std::vector<double>& qRand, double goalTol, int maxIters);
        void SwapTree(Tree& tree1, Tree& tree2);
        bool CheckGoal(std::vector<double> qNew);
        std::vector<std::vector<double>> GetRRTConnectPath(Tree& tree1, Tree& tree2);

        

    private:
        Tree mmyTree;
        Tree mmyTree2;

        double* mmap;
        int mmaxX;
        int mmaxY;
        double* mstartPose;
        double* mgoalPose;  
        int mnumOfDOFs;

        const double meps;
        const double mstepIters;
        const double mgoalTol;
        const int mmaxIters;


};
