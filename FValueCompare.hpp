#pragma once
#include "Node.hpp"

class FValueCompare{
    public:
        bool operator()(std::shared_ptr<Node> node1SmartPtr, std::shared_ptr<Node> node2SmartPtr)
        {

            double fValueNode1 = node1SmartPtr->GetFValue();
            // double fValueNode2 = node2.GetFValue();
            double fValueNode2 = node2SmartPtr->GetFValue();

            if ((fValueNode1) >= (fValueNode2)) {
                return true;
            } else {
                return false;
            }
        }
};
