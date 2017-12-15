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
        unsigned target_id; // essentially a row in the distance matrix
        EdgeWeight distance;
        DistanceData distance_data;
        NodeBucket(const unsigned target_id, const EdgeWeight distance, const DistanceData & distance_data)
            : target_id(target_id), distance(distance), distance_data(distance_data)
        {
        }
    };

    // FIXME This should be replaced by an std::unordered_multimap, though this needs benchmarking
    using SearchSpaceWithBuckets = std::unordered_map<NodeID, std::vector<NodeBucket>>;

    using ResultEntry = std::pair<EdgeWeight,DistanceData>;
  public:
    ManyToManyRouting(DataFacadeT *facade, SearchEngineData &engine_working_data)
        : super(facade), engine_working_data(engine_working_data)
    {
    }
    
     std::vector<EdgeWeight> durations(const std::vector<PhantomNode> &phantom_nodes,
                                       const std::vector<std::size_t> &source_indices,
                                       const std::vector<std::size_t> &target_indices) const
    {
      std::vector<ResultEntry> table = operator()(phantom_nodes, source_indices, target_indices);
      std::vector<EdgeWeight> durations;
      std::transform(table.begin(), 
               table.end(), 
               std::back_inserter(durations), 
               [](const ResultEntry& e) { return e.first; });
      return durations;
    }

    std::vector<ResultEntry> operator()(const std::vector<PhantomNode> &phantom_nodes,
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

        engine_working_data.InitializeOrClearFirstThreadLocalStorage(
            super::facade->GetNumberOfNodes());

        QueryHeap &query_heap = *(engine_working_data.forward_heap_1);
        NodeDistances heap_node_distances;

        SearchSpaceWithBuckets search_space_with_buckets;

        unsigned column_idx = 0;
        const auto search_target_phantom = [&](const PhantomNode &phantom) {
            query_heap.Clear();
            heap_node_distances.clear();
            // insert target(s) at distance 0

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
                BackwardRoutingStep(column_idx, query_heap, heap_node_distances, search_space_with_buckets);
            }
            ++column_idx;
        };

        // for each source do forward search
        unsigned row_idx = 0;
        const auto search_source_phantom = [&](const PhantomNode &phantom) {
            query_heap.Clear();
            heap_node_distances.clear();
            // insert target(s) at distance 0

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
                ForwardRoutingStep(row_idx,
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

    void ForwardRoutingStep(const unsigned row_idx,
                            const unsigned number_of_targets,
                            QueryHeap &query_heap,
                            NodeDistances & heap_node_distances,
                            const SearchSpaceWithBuckets &search_space_with_buckets,
                            std::vector<ResultEntry> &result_table) const
    {
        const NodeID node = query_heap.DeleteMin();
        const int source_distance = query_heap.GetKey(node);
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
                const int target_distance = current_bucket.distance;
                auto &current_distance = result_table[row_idx * number_of_targets + column_idx].first;
                // check if new distance is better
                const EdgeWeight new_distance = source_distance + target_distance;
                if (new_distance < 0)
                {
                    const std::pair<EdgeWeight,DistanceData> loop_weight_and_distance = super::GetLoopWeightAndDistance(node);
                    const int & loop_weight = loop_weight_and_distance.first;
                    const int new_distance_with_loop = new_distance + loop_weight;
                    if (loop_weight != INVALID_EDGE_WEIGHT && new_distance_with_loop >= 0 && new_distance_with_loop < current_distance)
                    {
                        const DistanceData new_path_distance_data = source_distance_data + current_bucket.distance_data + loop_weight_and_distance.second;
                        result_table[row_idx * number_of_targets + column_idx] = std::make_pair(new_distance_with_loop, new_path_distance_data);
                    }
                }
                else if (new_distance < current_distance)
                {
                    const DistanceData new_path_distance_data = source_distance_data + current_bucket.distance_data;
                    result_table[row_idx * number_of_targets + column_idx] = std::make_pair(new_distance, new_path_distance_data);
                }
            }
        }
        if (StallAtNode<true>(node, source_distance, query_heap))
        {
            return;
        }
        RelaxOutgoingEdges<true>(node, source_distance, source_distance_data, query_heap, heap_node_distances);
    }

    void BackwardRoutingStep(const unsigned column_idx,
                             QueryHeap &query_heap,
                             NodeDistances & heap_node_distances,
                             SearchSpaceWithBuckets &search_space_with_buckets) const
    {
        const NodeID node = query_heap.DeleteMin();
        const int target_distance = query_heap.GetKey(node);
        const DistanceData target_distance_data = heap_node_distances[node];
        heap_node_distances.erase(node);

        // store settled nodes in search space bucket
        search_space_with_buckets[node].emplace_back(column_idx, target_distance, target_distance_data);

        if (StallAtNode<false>(node, target_distance, query_heap))
        {
            return;
        }

        RelaxOutgoingEdges<false>(node, target_distance, target_distance_data, query_heap, heap_node_distances);
    }

    template <bool forward_direction>
    inline void
    RelaxOutgoingEdges(const NodeID node, const EdgeWeight distance, const DistanceData & distance_data, QueryHeap &query_heap, NodeDistances & heap_node_distances) const
    {
        for (auto edge : super::facade->GetAdjacentEdgeRange(node))
        {
            const auto &data = super::facade->GetEdgeData(edge);
            const bool direction_flag = (forward_direction ? data.forward : data.backward);
            if (direction_flag)
            {
                const NodeID to = super::facade->GetTarget(edge);
                const int edge_weight = data.distance;
                const DistanceData & edge_distance_data = data.distance_data;

                BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");
                const int to_distance = distance + edge_weight;
                const DistanceData to_distance_data = distance_data + edge_distance_data;

                // New Node discovered -> Add to Heap + Node Info Storage
                if (!query_heap.WasInserted(to))
                {
                    query_heap.Insert(to, to_distance, node);
                    heap_node_distances[to] = to_distance_data;
                }
                // Found a shorter Path -> Update distance
                else if (to_distance < query_heap.GetKey(to))
                {
                    // new parent
                    query_heap.GetData(to).parent = node;
                    query_heap.DecreaseKey(to, to_distance);
                    heap_node_distances[to] = to_distance_data;
                }
            }
        }
    }

    // Stalling
    template <bool forward_direction>
    inline bool
    StallAtNode(const NodeID node, const EdgeWeight distance, QueryHeap &query_heap) const
    {
        for (auto edge : super::facade->GetAdjacentEdgeRange(node))
        {
            const auto &data = super::facade->GetEdgeData(edge);
            const bool reverse_flag = ((!forward_direction) ? data.forward : data.backward);
            if (reverse_flag)
            {
                const NodeID to = super::facade->GetTarget(edge);
                const int edge_weight = data.distance;
                BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");
                if (query_heap.WasInserted(to))
                {
                    if (query_heap.GetKey(to) + edge_weight < distance)
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
