#ifndef QUERYEDGE_HPP
#define QUERYEDGE_HPP

#include "util/typedefs.hpp"

#include <tuple>

namespace osrm
{
namespace contractor
{

struct QueryEdge
{
    NodeID source;
    NodeID target;
    struct EdgeData
    {
        EdgeData() : id(0), shortcut(false), weight(0), forward(false), backward(false), distance_data(DistanceData()) {}

        template <class OtherT> EdgeData(const OtherT &other)
        {
            weight = other.weight;
            shortcut = other.shortcut;
            id = other.id;
            forward = other.forward;
            backward = other.backward;
            distance_data = other.distance_data;
        }
        // this ID is either the middle node of the shortcut, or the ID of the edge based node (node
        // based edge) storing the appropriate data. If `shortcut` is set to true, we get the middle
        // node. Otherwise we see the edge based node to access node data.
        NodeID id : 31;
        bool shortcut : 1;
        int weight : 30;
        bool forward : 1;
        bool backward : 1;
        DistanceData distance_data;
    } data;

    QueryEdge() : source(SPECIAL_NODEID), target(SPECIAL_NODEID) {}

    QueryEdge(NodeID source, NodeID target, EdgeData data)
        : source(source), target(target), data(std::move(data))
    {
    }

    bool operator<(const QueryEdge &rhs) const
    {
        return std::tie(source, target) < std::tie(rhs.source, rhs.target);
    }

    bool operator==(const QueryEdge &right) const
    {
        return (source == right.source && target == right.target &&
                data.weight == right.data.weight && data.shortcut == right.data.shortcut &&
                data.forward == right.data.forward && data.backward == right.data.backward &&
                data.id == right.data.id &&
                data.distance_data == right.data.distance_data);
    }
};
}
}

#endif // QUERYEDGE_HPP
