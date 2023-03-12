#pragma once;
#include <algorithm>
#include <memory>
#include <ostream>
#include <queue>
#include <unordered_map>
#include <vector>
#include "Tree.hpp"
#include "NeighborCompare.hpp"
#include "RRTState.hpp"
#include "Utils.hpp"

enum class State {TRAPPED, REACHED, ADVANCED, NOT_TRAPPED};

class RRTSolver
{

    public:
        RRTSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, double eps, double goalTol, int maxIters);
        ~RRTSolver();  


        std::unique_ptr<Tree> InitializeTree();
        std::vector<double> SampleRandomVertex();
        std::unique_ptr<Tree> BuildRRT();
        std::pair<State, std::vector<double>> CheckNewConfig(const std::vector<double>& qRand, const std::shared_ptr<Node> qNearNode, double eps);
        State ExtendTree(std::unique_ptr<Tree>& ptree, const std::vector<double>& qRand, double eps, double goalTol, int maxIters);


    private:
        double* mmap;
        int mmaxX;
        int mmaxY;
        double* mstartPose;
        double* mgoalPose;  
        int mnumOfDOFs;

        double meps;
        double mgoalTol;
        int mmaxIters


};
