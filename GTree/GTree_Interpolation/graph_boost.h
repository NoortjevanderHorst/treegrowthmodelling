/*
*	Copyright (C) 2022 by
*       Noortje van der Horst (noortje.v.d.horst1@gmail.com)
*       Liangliang Nan (liangliang.nan@gmail.com)
*       3D Geoinformation, TU Delft, https://3d.bk.tudelft.nl
*
*	This file is part of GTree, which implements the 3D tree
*   reconstruction and growth modelling method described in the following thesis:
*   -------------------------------------------------------------------------------------
*       Noortje van der Horst (2022).
*       Procedural Modelling of Tree Growth Using Multi-temporal Point Clouds.
*       Delft University of Technology.
*       URL: http://resolver.tudelft.nl/uuid:d284c33a-7297-4509-81e1-e183ed6cca3c
*   -------------------------------------------------------------------------------------
*   Please consider citing the above thesis if you use the code/program (or part of it).
*
*   GTree is based on the works of Easy3D and AdTree:
*   - Easy3D: Nan, L. (2021).
*       Easy3D: a lightweight, easy-to-use, and efficient C++ library for processing and rendering 3D data.
*       Journal of Open Source Software, 6(64), 3255.
*   - AdTree: Du, S., Lindenbergh, R., Ledoux, H., Stoter, J., & Nan, L. (2019).
*       AdTree: accurate, detailed, and automatic modelling of laser-scanned trees.
*       Remote Sensing, 11(18), 2074.
*
*	GTree is free software; you can redistribute it and/or modify
*	it under the terms of the GNU General Public License Version 3
*	as published by the Free Software Foundation.
*
*	GTree is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*	GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License
*	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef EASY3D_GRAPH_BOOST_H
#define EASY3D_GRAPH_BOOST_H


#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <easy3d/core/types.h>
#include <stdlib.h>


struct SGraphVertexPropB
{
    easy3d::vec3 coords;
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
