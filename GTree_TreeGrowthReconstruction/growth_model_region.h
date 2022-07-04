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

#ifndef TREEGROWTHMODELLING_GROWTH_MODEL_REGION_H
#define TREEGROWTHMODELLING_GROWTH_MODEL_REGION_H


#include <easy3d/core/types.h>
#include <easy3d/core/point_cloud.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <3rd_party/kdtree/ETH_Kd_Tree/vector3D.h>
#include <3rd_party/kdtree/ETH_Kd_Tree/kdTree.h>
#include <3rd_party/nlohmann/json.hpp>

#include "graph_gt.h"

using kdtree::Vector3D;

// CGAL geometry (also defined in Lobe, except envelope)
typedef CGAL::Exact_predicates_inexact_constructions_kernel K_cgal;
typedef K_cgal::Point_3 Point_3_cgal;
typedef CGAL::Surface_mesh<Point_3_cgal> Surface_mesh_cgal;

// R-tree from Boost
typedef boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian> Point_3_boost;
typedef boost::geometry::index::rtree<Point_3_boost, boost::geometry::index::quadratic<16> > RTree;

using kdtree::KdTree;

class GModelRegion {
public:
    GModelRegion(easy3d::vec3 root, easy3d::vec3 root_dir);
    GModelRegion(GModelRegion const &other);
    ~GModelRegion();

    //-- getters
    const GraphGT& get_graph() { return graph_; }

    //-- setters
    void set_graph(GraphGT graph) {graph_ = graph; }

    //-- growth functions

    /// Main controller function for running space colonization/region growing growth model.
    bool model_growth(const easy3d::PointCloud* points_in, const Surface_mesh_cgal* lobe_hull);

    /// Read the axiom once and apply the growth rules/constraints to form updated L-system.
    bool grow_iteration();

    /// (Recursively) read the passed L-string.
    std::string read_line(std::string line);

    /// Kill (remove) attraction points around a confirmed newly grown node, returns number of attraction points killed.
    int kill_zone(easy3d::vec3 node);

    /// Check if a given potential bud is viable or not: definition of hard growth constraints.
    bool is_viable(easy3d::vec3 pot_loc);

    /** Use perception cone on attraction point set to find optimal direction for growth.
     * Input: apical node.
     * Output: relative growth vector.
     */
    easy3d::vec3 compute_growth_direction(easy3d::vec3 p_parent);

    /// Correct growth direction for lateral nodes with potential found attraction points (perception cone) + hull influence.
    easy3d::vec3 compute_growth_direction_lateral(easy3d::vec3 pot_pt, easy3d::vec3 p_parent);

    /// Find the relative angles and distance between current position and given 3d point (next), returned as (rotation(y), roll(z), distance(f)).
    std::tuple<double, double, double> points_to_movement(easy3d::vec3 p_parent, easy3d::vec3 p_next);

    /// Convert relative movement angles/distance from one node to the next into a string of L-system commands.
    std::string movement_to_string(std::tuple<double, double, double> movement);

    /// Computes the absolute 3D location of a new node, if the given movement command were to be carried out from the current location and rotation.
    easy3d::vec3 compute_potential_location(std::string movement);

    /// Get angle between vector and X-axis, Z plane.
    double get_z_angle(easy3d::vec3 vec);

    /// Get angle between vector and X-axis, Y plane.
    double get_y_angle(easy3d::vec3 vec);

    //-- movement

    /// Move forward at the current heading.
    void go_forward(float distance);

    /// Rotate around the Y-axis, angle in [radians].
    void go_rotate(float angle);

    /// Rotate (roll) around the Z-axis, angle in [radians].
    void go_roll(float angle);

    /// Sdd a vertex (with it's correct parent index) to the current skeleton graph.
    void add_vertex(unsigned int n_parent);

private:
    // storage
    std::string axiom_;                         // current L-system starter string
    GraphGT graph_;                             // current generated skeleton graph (?)
    int num_iterations_ = 0;                    // number of iterations that were grown

    easy3d::PointCloud points_;                 // input point cloud
    Surface_mesh_cgal hull_mesh_;               // surface mesh of convex hull of lobe
    easy3d::vec3 translation_;                  // translation to original location (growth starts from (0, 0, 0)
    easy3d::vec3 rotation_;                     // rotation to original location (growth starts with [X?] is forward)
    easy3d::vec3 start_loc_;                    // position of lobe connector point
    easy3d::Mat<3, 3, float> start_plane_;      // direction of lobe connector point (needed to reset after growth iterations)

    // index
    int num_attr_points_ = 0;                   // number of current attraction points (size of kd_points_in)
    Vector3D* kd_points_in_;
    KdTree* kdtree_in_;                         // index of vertices of input lobe point cloud
    RTree rtree_out_;                           // index of vertices of created skeleton graph

    // location
    easy3d::vec3 loc_;                          // current absolute coordinates
    easy3d::Mat<3, 3, float> plane_;            // current absolute rotation, a 2d plane in a 3d space

    // default movement values
    // todo: make editable by user?
    float def_forward_ = 0;                     // [m]
    float def_rotate_ = 0;                      // [radians]
    float def_roll_ = 0;                        // [radians]

    int accuracy_ = 0;                          // accuracy of float values in string (nr decimals)

    float max_internode_length_ = 0;            // max allowed length for branch parts
    float min_internode_length_ = 0;
};


#endif //TREEGROWTHMODELLING_GROWTH_MODEL_REGION_H
