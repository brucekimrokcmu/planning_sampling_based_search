#pragma once
#include "Node.hpp"

class FValueCompare{
    public:
        bool operator()(Node* node1, Node* node2)
        {
            // double fValueNode1 = node1.GetFValue();
            double fValueNode1 = node1->GetFValue();
            // double fValueNode2 = node2.GetFValue();
            double fValueNode2 = node2->GetFValue();

            if ((fValueNode1) >= (fValueNode2)) {
                return true;
            } else {
                return false;
            }
        }
};
