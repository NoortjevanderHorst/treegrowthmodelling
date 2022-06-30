//
// Created by noort on 13/05/2022.
//

#ifndef EASY3D_GRAPH_BOOST_H
#define EASY3D_GRAPH_BOOST_H


#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <easy3d/core/types.h>
#include <stdlib.h>


struct SGraphVertexPropB
{
    easy3d::vec3 coords;  // from root of tree A ts 0 (changed a little)
    std::size_t parent;
    bool clear_mark = false;    // vertex should be deleted from current graph
    bool delete_mark = false;   // vertex was deleted from current graph
    int i_base = -1;
    int i_target = -1;
    easy3d::vec3 c_base;
    easy3d::vec3 c_target;
    bool insert_mark = false;
    bool copy_mark = false;
    bool inter_delete_mark = false;     // vertex will be deleted in next timestamp graph

    // for cylinder fitting
    double lengthOfSubtree;
    double radius;
    bool visited;
};


struct SGraphEdgePropB
{
    double length;

    // for cylinder fitting
    double nWeight;
    double nRadius;
    std::vector<int> vecPoints;
};


class GraphB : public boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, SGraphVertexPropB, SGraphEdgePropB > {
public:
    GraphB() {}

    unsigned int rootv;
};

typedef boost::graph_traits<GraphB>::vertex_descriptor VertexDescriptorGraphB;
typedef boost::graph_traits<GraphB>::edge_descriptor EdgeDescriptorGraphB;
typedef boost::graph_traits<GraphB>::vertex_iterator VertexIteratorGraphB;
typedef boost::graph_traits<GraphB>::edge_iterator EdgeIteratorGraphB;
typedef boost::graph_traits<GraphB>::adjacency_iterator AdjacencyIteratorGraphB;
typedef boost::graph_traits<GraphB>::out_edge_iterator  EdgeOutIteratorGraphB;


#endif //EASY3D_GRAPH_BOOST_H
