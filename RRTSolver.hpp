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
        // Tree BuildRRTConnect();
        std::pair<State, std::vector<double>> CheckNewConfig(const std::vector<double>& qRand, const std::shared_ptr<Node> qNearNode);
        // State ExtendTree(const std::vector<double>& qRand, double goalTol, int maxIters);
        State ExtendTree(Tree& tree, const std::vector<double>& qRand, double goalTol, int maxIters);
        bool checkGoal(std::vector<double> qNew);

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
