#include "Node.hpp"



Node::Node(){}
Node::Node(std::vector<double> vertex)
    : mvertex(vertex), mG(std::numeric_limits<double>::infinity()), mH(0), mF(0)
    {
    }

void Node::SetEdge(Node node)
{
    Node* pnode = &node; // shared ptr?
    mpedges.emplace_back(pnode);

}
