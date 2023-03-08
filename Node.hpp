#pragma once
// #include "Edge.hpp"
#include <limits>
#include <memory>
#include <vector>

class Node{
    
    public:
        Node(){};
        Node(int idx, std::vector<double> vertex)
        : midx(idx), mvertex(vertex), mG(std::numeric_limits<double>::infinity()), mH(0), mF(0), mcomponentID(idx), mpParent(nullptr) //, mvisited(false)
        {
        }
        std::vector<double> GetJointPose() const {return mvertex;};
        int GetIndex() const {return midx;}

        std::vector<std::shared_ptr<Node>> GetEdges() const {return mpedges;};

        void  AddEdge(std::shared_ptr<Node> edge) {mpedges.push_back(edge);};

        // bool IsVisited() {return mvisited;};
        // void SetVisited(bool val) { mvisited=val; };

        int GetComponentID() const {return mcomponentID;};
        void SetComponentID(int val) {mcomponentID = val;};
        void SetParent(std::shared_ptr<Node> pnode) { mpParent = pnode;};
        std::shared_ptr<Node> GetParent() const { return mpParent;};
        
        // // Kruskal's algorithm
        // inline std::vector<Edge> GetEdges() const {return medges;};
        // inline void SetEdges(std::vector<Edge> edge) {};
        

        double GetGValue() const {return mG;};
        void SetGValue(const double val) {mG = val;};
        double GetHValue() const {return mH;};
        void SetHvalue(const double val){mH = val;};
        double GetFValue() const {return mF;};
        void SetFValue(const double val) {mF = val;};
        double GetTempDist() const {return mtempDist;};
        void SetTempDist(const double val) {mtempDist = val;};

    private:
        int midx; 
        std::vector<double> mvertex;
        std::vector<std::shared_ptr<Node>> mpedges;

        // bool mvisited;
        int mcomponentID;
        std::shared_ptr<Node> mpParent;
        // std::vector<Edge> medges; // Not needed after discarding kruskal's algorithm
        double mG;
        double mH;
        double mF;
        double mtempDist;



};