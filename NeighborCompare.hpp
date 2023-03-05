#pragma once
#include <iostream>
#include "Node.hpp"

class NeighborCompare{
    public:
        bool operator()(std::shared_ptr<Node> pnode1, std::shared_ptr<Node> pnode2)
        {
            // std::cout<<"start compare" << std::endl;
            if ((pnode1->GetTempDist()) >= (pnode2->GetTempDist())) {
                // std::cout<<"return true" << std::endl;
                return true;
            } else {
                // std::cout<<"return false" << std::endl;
                return false;
            }
        }
};