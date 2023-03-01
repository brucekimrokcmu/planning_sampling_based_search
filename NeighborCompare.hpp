#pragma once
#include "Node.hpp"

class NeighborCompare{
    public:
        bool operator()(Node node1, Node node2)
        {
            if ((node1.GetTempDist()) >= (node2.GetTempDist())) {
                return true;
            } else {
                return false;
            }
        }
};