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

#ifndef TREEGROWTHMODELLING_LOBE_H
#define TREEGROWTHMODELLING_LOBE_H


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polygon_mesh_processing/measure.h>

#include <easy3d/core/types.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>

#include "graph_gt.h"


// CGAL geometry for convex hull
typedef CGAL::Exact_predicates_inexact_constructions_kernel K_cgal;
typedef K_cgal::Point_3 Point_3_cgal;
typedef CGAL::Surface_mesh<Point_3_cgal> Surface_mesh_cgal;


namespace easy3d {
    class PointCloud;
    class SurfaceMesh;
}


class Lobe {
public:
    Lobe();
    ~Lobe();

    // getters
    const GraphGT& get_subgraph() const { return graph_; }
    VertexDescriptorGTGraph get_connector_node() const { return connector_node_; }
    int get_index() const { return lobe_index_; }
    const Surface_mesh_cgal& get_convex_hull() const { return convex_hull_; }
    const easy3d::PointCloud& get_pointcloud() const { return points_easy3d_; }

    // setters
    void set_connector_node(VertexDescriptorGTGraph vert) { connector_node_ = vert; }
    void set_index(int idx) { lobe_index_ = idx; }
    void set_skeleton(GraphGT graph) { graph_ = graph; }

    bool build_lobe_hulls(easy3d::SurfaceMesh *mesh);

private:
    GraphGT graph_; // subgraph of lobe
    VertexDescriptorGTGraph connector_node_;  // vertex index from GSkeleton->corresponding_ graph that this lobe connects to
    int lobe_index_;    // index number of lobe

    easy3d::PointCloud points_easy3d_;

    std::vector<Point_3_cgal> points_cgal_;
    Surface_mesh_cgal convex_hull_;

    // todo: perhaps kd tree of vertices will be useful?



};


#endif //TREEGROWTHMODELLING_LOBE_H
