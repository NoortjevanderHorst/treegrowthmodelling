//
// Created by noort on 23/01/2022.
//

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
//        std::cout << "constructed hull has small volume, skipping current hull" << std::endl;
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
