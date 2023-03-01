#pragma once
#include <limits>
#include <memory>
#include <vector>


class Node{
    
    public:
        // For hw2, the following constructor is used
        Node();
        Node(std::vector<double> vertex);  
        
        inline std::vector<double> GetJointPose() const {return mvertex;};
        inline std::vector<Node*> GetEdge() const {return mpedges;};
        void SetEdge(Node node);

        inline double GetGValue() const {return mG;};
        inline void SetGValue(const double val) {mG = val;};
        inline double GetHValue() const {return mH;};
        inline void SetHvalue(const double val){mH = val;};
        inline double GetFValue() const {return mF;};
        inline void SetFValue(const double val) {mF = val;};
        inline double GetTempDist() const {return mtempDist;};
        inline void SetTempDist(const double val) {mtempDist = val;};

    private:
        std::vector<double> mvertex;
        std::vector<Node*> mpedges;
        // Cell cost g-value and heuristics
        double mG;
        double mH;
        double mF;
        //
        double mtempDist;

        // Backtracking 
        // Node* mpParent;
        // keep a pointer to your parent
 
};