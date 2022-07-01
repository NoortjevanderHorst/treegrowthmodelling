//
// Created by noort on 23/01/2022.
//

#ifndef TREEGROWTHMODELLING_GRAPH_GT_H
#define TREEGROWTHMODELLING_GRAPH_GT_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <easy3d/core/types.h>


/// define the vertex properties of custom growing graph
struct SGraphVertexPropGT
{
    easy3d::vec3  coords;                           // 3D coordinates
    std::size_t parent;                             // index parent vertex
    double branch_level;                            // nr of bifurcation steps to root node
    double weight;                                  // weight based on nr of similar vertices in neighbourhood
    easy3d::vec3 normal = {0, 0, 0};   // maximum eigen vector of point from pca
    bool clear_mark = false;                        // mark if vertex should be deleted
    bool delete_mark = false;                       // mark if vertex was deleted
    bool is_main = false;                           // flag vertices that are part of main (corresponding) skeleton
    bool is_lobe_node = false;                      // flag nodes that have lobes connected to them
    bool is_lobe_node_corr = false;                 // specifically note if lobe node came from corresponding skeleton
    bool is_lobe_node_ts = false;                   // specifically note if lobe node came from timestamp (additional)
    int lobe_index = -1;                            // index of lobe cluster the vertex belongs to (-1 if not in lobe, starts at 0)
    // if vertex is used as lobe node: is_lobe_node is true, and lobe_index will say which lobe

    // for cylinder fitting
    double lengthOfSubtree;
    double radius; // used only by the smoothed skeleton
    bool visited;

    // for interpolation
    int inter_target = -1;          // index of next graph node in interpolation
    int inter_base = -1;            // index of previous graph node in interpolation (in that graph's index)
    bool inter_add_mark = false;    // node was added in previous interpolation
    bool inter_delete_mark = false; // node will be deleted in next interpolation
    int inter_merged = -1;          // node will be merged to a shared target (index of target)

    // for interpolation, for establishing a consistent correspondence path between all tips and the root
    // with current graph as target
    int corr_based = false;                 // node has a consistent correspondence path back to the root
    int corr_tip = false;                   // node  was the furthest correspondence on a branch segment
    std::vector<unsigned int> inter_tips;   // list of tips connected in rest of tree to the current node
    bool inter_bifur_mark = false;          // node  continues into multiple tip vertices
    bool inter_bifur_mark_temp = false;     // just for visualising bifur points
};

/// Define the edge properties of custom growing graph
struct SGraphEdgePropGT
{
    double length;                // length [m]
    double length_to_root;        // length of path along graph to root node [m]
    double branch_level;          // nr of bifurcation steps to root node
    std::set<int> correspondence; // indices of timestamps it corresponds with
    bool is_main = false;         // flag edges that are part of main skeleton
    double distance;              // distance to merged main skeleton

    // for cylinder fitting
    double nWeight;
    double nRadius;
    std::vector<int> vecPoints;

    // for interpolation
    std::pair<int, int> inter_target = std::make_pair(-1, -1); // index of next graph edge in interpolation
    std::pair<int, int> inter_base = std::make_pair(-1, -1);  // index of previous graph edge in interpolation (in that graph's index)
    bool inter_add_mark = false;    // edge was added in previous interpolation
    bool inter_delete_mark = false; // edge will be deleted in next interpolation
};


/// Custom boost adjacency graph
class GraphGT : public boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, SGraphVertexPropGT, SGraphEdgePropGT > {
public:
    GraphGT() {}
};

typedef boost::graph_traits<GraphGT>::vertex_descriptor VertexDescriptorGTGraph;
typedef boost::graph_traits<GraphGT>::edge_descriptor EdgeDescriptorGTGraph;
typedef boost::graph_traits<GraphGT>::vertex_iterator VertexIteratorGTGraph;
typedef boost::graph_traits<GraphGT>::edge_iterator EdgeIteratorGTGraph;
typedef boost::graph_traits<GraphGT>::adjacency_iterator AdjacencyIteratorGTGraph;
typedef boost::graph_traits<GraphGT>::out_edge_iterator  EdgeOutIteratorGTGraph;


#endif //TREEGROWTHMODELLING_GRAPH_GT_H
