#pragma once

#include <vector>

namespace vermils
{
namespace directy
{
    class RootNode;
    using NodeContainer = std::vector<RootNode>;
    class RootNode
    {
    protected:
        NodeContainer _children;
        unsigned _refcnt=0;
    public:
    };


}
}
