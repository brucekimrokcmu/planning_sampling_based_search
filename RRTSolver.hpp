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

enum class State {TRAPPED, REACHED, ADVANCED, NOT_TRAPPED};

class RRTSolver
{

    public:
        RRTSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, double eps, double stepSize, double goalTol, int maxIters);
        ~RRTSolver();  


        std::vector<double> SampleRandomVertex(std::random_device& rd);
        Tree BuildRRT();
        std::pair<State, std::vector<double>> CheckNewConfig(const std::vector<double>& qRand, const std::shared_ptr<Node> qNearNode);
        State ExtendTree(const std::vector<double>& qRand, double goalTol, int maxIters);


    private:
        Tree mtree;
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
