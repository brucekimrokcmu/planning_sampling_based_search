#pragma once
// #include "Edge.hpp"
#include <limits>
#include <memory>
#include <vector>

class Node{
    
    public:
        Node(){};
        Node(int idx, std::vector<double> vertex)
        : midx(idx), mvertex(vertex), mG(std::numeric_limits<double>::infinity()), mH(0), mF(0), mpedge(nullptr), mcomponentID(idx)
        {
        }
        inline std::vector<double> GetJointPose() const {return mvertex;};
        inline int GetIndex() const {return midx;}

        // using simple linked list 
        inline std::shared_ptr<Node> GetEdge() const {return mpedge;};
        inline void SetEdge(std::shared_ptr<Node> edge) {mpedge = edge;};
        inline int GetComponentID() const {return mcomponentID;};
        inline void SetComponentID(int val) {mcomponentID = val;};

        // // Kruskal's algorithm
        // inline std::vector<Edge> GetEdges() const {return medges;};
        // inline void SetEdges(std::vector<Edge> edge) {};
        

        inline double GetGValue() const {return mG;};
        inline void SetGValue(const double val) {mG = val;};
        inline double GetHValue() const {return mH;};
        inline void SetHvalue(const double val){mH = val;};
        inline double GetFValue() const {return mF;};
        inline void SetFValue(const double val) {mF = val;};
        inline double GetTempDist() const {return mtempDist;};
        inline void SetTempDist(const double val) {mtempDist = val;};

    private:
        int midx; // Not needed after discarding kruskal's algorithm
        std::vector<double> mvertex;
        std::shared_ptr<Node> mpedge;
        int mcomponentID;
        // std::vector<Edge> medges;
        double mG;
        double mH;
        double mF;
        double mtempDist;



};