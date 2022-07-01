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

#include "lobe.h"


Lobe::Lobe(){
    points_cgal_.clear();
    lobe_index_ = -1;
};


Lobe::~Lobe(){
    if (points_cgal_.size() > 0)
        points_cgal_.clear();
};


bool Lobe::build_lobe_hulls(easy3d::SurfaceMesh *mesh){
    // make set of CGAL compatible points
   std::vector<Point_3_cgal> cluster_points;
    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(graph_);
    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
        Point_3_cgal pt = {graph_[*vit].coords.x, graph_[*vit].coords.y, graph_[*vit].coords.z};
        cluster_points.push_back(pt);
        points_easy3d_.add_vertex(graph_[*vit].coords);
    }

    // compute hull
    Surface_mesh_cgal hull;
    CGAL::convex_hull_3(cluster_points.begin(), cluster_points.end(), hull);

    if (hull.num_vertices() == 0 || hull.num_faces() == 0){
        std::cout << "ERROR: failed to construct lobe convex hull" << std::endl;
        return false;
    }

    // filter out lobes that are too small
    // todo: other conditions?
    double vol = CGAL::Polygon_mesh_processing::volume(hull);   // todo: make parameter
    if (vol < 1){
        return false;
    }

    // make mesh
    for (auto fit : faces(hull)){
        std::vector<easy3d::SurfaceMesh::Vertex> verts;

        Surface_mesh_cgal::Halfedge_index he_begin = hull.halfedge(fit);
        for (Surface_mesh_cgal::Halfedge_index het : CGAL::halfedges_around_face(he_begin, hull)){
            Point_3_cgal pt = hull.point(target(het, hull));
            easy3d::vec3 pt_curr = {static_cast<float>(pt.x()), static_cast<float>(pt.y()), static_cast<float>(pt.z())};
            easy3d::SurfaceMesh::Vertex v_curr = mesh->add_vertex(pt_curr);
            verts.push_back(v_curr);
        }
        mesh->add_triangle(verts[0], verts[1], verts[2]); // assumed only triangles exist in polyhedron
    }

    // store mesh internally
    convex_hull_ = hull;

    return true;
}
