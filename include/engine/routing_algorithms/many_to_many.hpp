#ifndef MANY_TO_MANY_ROUTING_HPP
#define MANY_TO_MANY_ROUTING_HPP

#include "engine/routing_algorithms/routing_base.hpp"
#include "engine/search_engine_data.hpp"
#include "util/typedefs.hpp"

#include <boost/assert.hpp>

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

template <class DataFacadeT>
class ManyToManyRouting final
    : public BasicRoutingInterface<DataFacadeT, ManyToManyRouting<DataFacadeT>>
{
    using super = BasicRoutingInterface<DataFacadeT, ManyToManyRouting<DataFacadeT>>;
    using QueryHeap = SearchEngineData::QueryHeap;
    using NodeDistances = std::unordered_map<NodeID, DistanceData>;
    SearchEngineData &engine_working_data;

    struct NodeBucket
    {
        unsigned target_id; // essentially a row in the weight matrix
        EdgeWeight weight;
        DistanceData distance_data;
        NodeBucket(const unsigned target_id, const EdgeWeight weight, const DistanceData & distance_data)
            : target_id(target_id), weight(weight), distance_data(distance_data)
        {
        }
    };

    // FIXME This should be replaced by an std::unordered_multimap, though this needs benchmarking
    using SearchSpaceWithBuckets = std::unordered_map<NodeID, std::vector<NodeBucket>>;
    using ResultEntry = std::pair<EdgeWeight,DistanceData>;
  public:
    ManyToManyRouting(SearchEngineData &engine_working_data)
        : engine_working_data(engine_working_data)
    {
    }


    std::vector<EdgeWeight> durations(const DataFacadeT &facade,
                                      const std::vector<PhantomNode> &phantom_nodes,
                                      const std::vector<std::size_t> &source_indices,
                                      const std::vector<std::size_t> &target_indices) const
    {
        std::vector<ResultEntry> table = operator()(facade, phantom_nodes, source_indices, target_indices);
        std::vector<EdgeWeight> durations;
        std::transform(table.begin(),
               table.end(),
               std::back_inserter(durations),
               [](const ResultEntry& e) { return e.first; });
        return durations;
    }

    std::vector<ResultEntry> operator()(const DataFacadeT &facade,
                                       const std::vector<PhantomNode> &phantom_nodes,
                                       const std::vector<std::size_t> &source_indices,
                                       const std::vector<std::size_t> &target_indices) const
    {
        const auto number_of_sources =
            source_indices.empty() ? phantom_nodes.size() : source_indices.size();
        const auto number_of_targets =
            target_indices.empty() ? phantom_nodes.size() : target_indices.size();
        const auto number_of_entries = number_of_sources * number_of_targets;
        std::vector<ResultEntry> result_table(number_of_entries,
                                              std::make_pair(std::numeric_limits<EdgeWeight>::max(), INVALID_DISTANCE_DATA));

        engine_working_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes());

        QueryHeap &query_heap = *(engine_working_data.forward_heap_1);
        NodeDistances heap_node_distances;

        SearchSpaceWithBuckets search_space_with_buckets;

        unsigned column_idx = 0;
        const auto search_target_phantom = [&](const PhantomNode &phantom) {
            query_heap.Clear();
            heap_node_distances.clear();
            // insert target(s) at weight 0

            if (phantom.forward_segment_id.enabled)
            {
                query_heap.Insert(phantom.forward_segment_id.id,
                                  phantom.GetForwardWeightPlusOffset(),
                                  phantom.forward_segment_id.id);
                heap_node_distances[phantom.forward_segment_id.id] = phantom.forward_distance_data;
            }
            if (phantom.reverse_segment_id.enabled)
            {
                query_heap.Insert(phantom.reverse_segment_id.id,
                                  phantom.GetReverseWeightPlusOffset(),
                                  phantom.reverse_segment_id.id);
                heap_node_distances[phantom.reverse_segment_id.id] = phantom.reverse_distance_data;
            }

            // explore search space
            while (!query_heap.Empty())
            {
                BackwardRoutingStep(facade, column_idx, query_heap, heap_node_distances, search_space_with_buckets);
            }
            ++column_idx;
        };

        // for each source do forward search
        unsigned row_idx = 0;
        const auto search_source_phantom = [&](const PhantomNode &phantom) {
            query_heap.Clear();
            heap_node_distances.clear();
            // insert target(s) at weight 0

            if (phantom.forward_segment_id.enabled)
            {
                query_heap.Insert(phantom.forward_segment_id.id,
                                  -phantom.GetForwardWeightPlusOffset(),
                                  phantom.forward_segment_id.id);
                heap_node_distances[phantom.forward_segment_id.id] = -phantom.forward_distance_data;
            }
            if (phantom.reverse_segment_id.enabled)
            {
                query_heap.Insert(phantom.reverse_segment_id.id,
                                  -phantom.GetReverseWeightPlusOffset(),
                                  phantom.reverse_segment_id.id);
                heap_node_distances[phantom.reverse_segment_id.id] = -phantom.reverse_distance_data;
            }

            // explore search space
            while (!query_heap.Empty())
            {
                ForwardRoutingStep(facade,
                                   row_idx,
                                   number_of_targets,
                                   query_heap,
                                   heap_node_distances,
                                   search_space_with_buckets,
                                   result_table);
            }
            ++row_idx;
        };

        if (target_indices.empty())
        {
            for (const auto &phantom : phantom_nodes)
            {
                search_target_phantom(phantom);
            }
        }
        else
        {
            for (const auto index : target_indices)
            {
                const auto &phantom = phantom_nodes[index];
                search_target_phantom(phantom);
            }
        }

        if (source_indices.empty())
        {
            for (const auto &phantom : phantom_nodes)
            {
                search_source_phantom(phantom);
            }
        }
        else
        {
            for (const auto index : source_indices)
            {
                const auto &phantom = phantom_nodes[index];
                search_source_phantom(phantom);
            }
        }

        return result_table;
    }

    void ForwardRoutingStep(const DataFacadeT &facade,
                            const unsigned row_idx,
                            const unsigned number_of_targets,
                            QueryHeap &query_heap,
                            NodeDistances & heap_node_distances,
                            const SearchSpaceWithBuckets &search_space_with_buckets,
                            std::vector<ResultEntry> &result_table) const
    {
        const NodeID node = query_heap.DeleteMin();
        const int source_weight = query_heap.GetKey(node);
        const DistanceData source_distance_data = heap_node_distances[node];
        heap_node_distances.erase(node);

        // check if each encountered node has an entry
        const auto bucket_iterator = search_space_with_buckets.find(node);
        // iterate bucket if there exists one
        if (bucket_iterator != search_space_with_buckets.end())
        {
            const std::vector<NodeBucket> &bucket_list = bucket_iterator->second;
            for (const NodeBucket &current_bucket : bucket_list)
            {
                // get target id from bucket entry
                const unsigned column_idx = current_bucket.target_id;
                const int target_weight = current_bucket.weight;
                auto &current_weight = result_table[row_idx * number_of_targets + column_idx].first;
                // check if new weight is better
                const EdgeWeight new_weight = source_weight + target_weight;
                if (new_weight < 0)
                {
                    const std::pair<EdgeWeight,DistanceData> loop_weight_and_distance = super::GetLoopWeightAndDistance(facade, node);
                    const int & loop_weight = loop_weight_and_distance.first;


                    const int new_weight_with_loop = new_weight + loop_weight;
                    if (loop_weight != INVALID_EDGE_WEIGHT && new_weight_with_loop >= 0 && new_weight_with_loop < current_weight)
                    {
                        const DistanceData new_path_distance_data = source_distance_data + current_bucket.distance_data + loop_weight_and_distance.second;
                        result_table[row_idx * number_of_targets + column_idx] = std::make_pair(new_weight_with_loop, new_path_distance_data);
                    }
                }
                else if (new_weight < current_weight)
                {
                    const DistanceData new_path_distance_data = source_distance_data + current_bucket.distance_data;
                    result_table[row_idx * number_of_targets + column_idx] = std::make_pair(new_weight, new_path_distance_data);
                }
            }
        }
        if (StallAtNode<true>(facade, node, source_weight, query_heap, heap_node_distances))
        {
            return;
        }
        RelaxOutgoingEdges<true>(facade, node, source_weight, source_distance_data, query_heap, heap_node_distances);
    }

    void BackwardRoutingStep(const DataFacadeT &facade,
                             const unsigned column_idx,
                             QueryHeap &query_heap,
                             NodeDistances & heap_node_distances,
                             SearchSpaceWithBuckets &search_space_with_buckets) const
    {
        const NodeID node = query_heap.DeleteMin();
        const int target_weight = query_heap.GetKey(node);
        const DistanceData target_distance_data = heap_node_distances[node];
        heap_node_distances.erase(node);

        // store settled nodes in search space bucket
        search_space_with_buckets[node].emplace_back(column_idx, target_weight, target_distance_data);

        if (StallAtNode<false>(facade, node, target_weight, query_heap, heap_node_distances))
        {
            return;
        }

        RelaxOutgoingEdges<false>(facade, node, target_weight, target_distance_data, query_heap, heap_node_distances);
    }

    template <bool forward_direction>
    inline void RelaxOutgoingEdges(const DataFacadeT &facade,
                                   const NodeID node,
                                   const EdgeWeight weight,
                                   const DistanceData & distance_data,
                                   QueryHeap &query_heap,
                                   NodeDistances & heap_node_distances) const
    {
        for (auto edge : facade.GetAdjacentEdgeRange(node))
        {
            const auto &data = facade.GetEdgeData(edge);
            const bool direction_flag = (forward_direction ? data.forward : data.backward);
            if (direction_flag)
            {
                const NodeID to = facade.GetTarget(edge);
                const int edge_weight = data.weight;
                const DistanceData & edge_distance_data = data.distance_data;

                BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");
                const int to_weight = weight + edge_weight;
                const DistanceData to_distance_data = distance_data + edge_distance_data;

                // New Node discovered -> Add to Heap + Node Info Storage
                if (!query_heap.WasInserted(to))
                {
                    query_heap.Insert(to, to_weight, node);
                    heap_node_distances[to] = to_distance_data;
                }
                // Found a shorter Path -> Update weight
                else if (to_weight < query_heap.GetKey(to))
                {
                    // new parent
                    query_heap.GetData(to).parent = node;
                    query_heap.DecreaseKey(to, to_weight);
                    heap_node_distances[to] = to_distance_data;
                }
            }
        }
    }

    // Stalling
    template <bool forward_direction>
    inline bool StallAtNode(const DataFacadeT &facade,
                            const NodeID node,
                            const EdgeWeight weight,
                            QueryHeap &query_heap,
                            NodeDistances & heap_node_distances) const
    {
        for (auto edge : facade.GetAdjacentEdgeRange(node))
        {
            const auto &data = facade.GetEdgeData(edge);
            const bool reverse_flag = ((!forward_direction) ? data.forward : data.backward);
            if (reverse_flag)
            {
                const NodeID to = facade.GetTarget(edge);
                const int edge_weight = data.weight;
                BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");
                if (query_heap.WasInserted(to))
                {
                    if (query_heap.GetKey(to) + edge_weight < weight)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }
};
}
}
}

#endif
