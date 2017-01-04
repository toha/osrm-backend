#ifndef NODE_BASED_EDGE_HPP
#define NODE_BASED_EDGE_HPP

#include "extractor/travel_mode.hpp"
#include "util/typedefs.hpp"

#include "extractor/guidance/road_classification.hpp"

namespace osrm
{
namespace extractor
{

struct NodeBasedEdge
{
    NodeBasedEdge();

    NodeBasedEdge(NodeID source,
                  NodeID target,
                  NodeID name_id,
                  EdgeWeight weight,
                  bool forward,
                  bool backward,
                  bool roundabout,
                  bool circular,
                  bool startpoint,
                  TravelMode travel_mode,
                  bool is_split,
                  const LaneDescriptionID lane_description_id,
                  guidance::RoadClassification road_classification,
                  const DistanceData & distance_data);

    bool operator<(const NodeBasedEdge &other) const;

    NodeID source;
    NodeID target;
    NodeID name_id;
    EdgeWeight weight;
    bool forward : 1;
    bool backward : 1;
    bool roundabout : 1;
    bool circular : 1;
    bool startpoint : 1;
    bool is_split : 1;
    TravelMode travel_mode : 4;
    LaneDescriptionID lane_description_id;
    guidance::RoadClassification road_classification;
    DistanceData distance_data;
};

struct NodeBasedEdgeWithOSM : NodeBasedEdge
{
    NodeBasedEdgeWithOSM(OSMNodeID source,
                         OSMNodeID target,
                         NodeID name_id,
                         EdgeWeight weight,
                         bool forward,
                         bool backward,
                         bool roundabout,
                         bool circular,
                         bool startpoint,
                         TravelMode travel_mode,
                         bool is_split,
                         const LaneDescriptionID lane_description_id,
                         guidance::RoadClassification road_classification,
                         const DistanceData & distance_data);

    OSMNodeID osm_source_id;
    OSMNodeID osm_target_id;
};

// Impl.

inline NodeBasedEdge::NodeBasedEdge()
    : source(SPECIAL_NODEID), target(SPECIAL_NODEID), name_id(0), weight(0), forward(false),
      backward(false), roundabout(false), circular(false), startpoint(true), is_split(false),
      travel_mode(false), lane_description_id(INVALID_LANE_DESCRIPTIONID), distance_data(DistanceData())
{
}

inline NodeBasedEdge::NodeBasedEdge(NodeID source,
                                    NodeID target,
                                    NodeID name_id,
                                    EdgeWeight weight,
                                    bool forward,
                                    bool backward,
                                    bool roundabout,
                                    bool circular,
                                    bool startpoint,
                                    TravelMode travel_mode,
                                    bool is_split,
                                    const LaneDescriptionID lane_description_id,
                                    guidance::RoadClassification road_classification,
                                    const DistanceData & distance_data)
    : source(source), target(target), name_id(name_id), weight(weight), forward(forward),
      backward(backward), roundabout(roundabout), circular(circular), startpoint(startpoint),
      is_split(is_split), travel_mode(travel_mode), lane_description_id(lane_description_id),
      road_classification(std::move(road_classification)), distance_data(distance_data)
{
}

inline bool NodeBasedEdge::operator<(const NodeBasedEdge &other) const
{
    if (source == other.source)
    {
        if (target == other.target)
        {
            if (weight == other.weight)
            {
                return forward && backward && ((!other.forward) || (!other.backward));
            }
            return weight < other.weight;
        }
        return target < other.target;
    }
    return source < other.source;
}

inline NodeBasedEdgeWithOSM::NodeBasedEdgeWithOSM(OSMNodeID source,
                                                  OSMNodeID target,
                                                  NodeID name_id,
                                                  EdgeWeight weight,
                                                  bool forward,
                                                  bool backward,
                                                  bool roundabout,
                                                  bool circular,
                                                  bool startpoint,
                                                  TravelMode travel_mode,
                                                  bool is_split,
                                                  const LaneDescriptionID lane_description_id,
                                                  guidance::RoadClassification road_classification,
                                                  const DistanceData & distance_data)
    : NodeBasedEdge(SPECIAL_NODEID,
                    SPECIAL_NODEID,
                    name_id,
                    weight,
                    forward,
                    backward,
                    roundabout,
                    circular,
                    startpoint,
                    travel_mode,
                    is_split,
                    lane_description_id,
                    std::move(road_classification),
                    distance_data),
      osm_source_id(std::move(source)), osm_target_id(std::move(target))
{
}

} // ns extractor
} // ns osrm

#endif /* NODE_BASED_EDGE_HPP */
