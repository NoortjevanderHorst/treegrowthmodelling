//
// Created by noort on 17/03/2022.
//

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
#include <3rd_party/kd_tree/Vector3D.h>
#include <3rd_party/kd_tree/KdTree.h>
#include <3rd_party/nlohmann/json.hpp>

#include "graph_gt.h"


// CGAL geometry (also defined in Lobe, except envelope)
typedef CGAL::Exact_predicates_inexact_constructions_kernel K_cgal;
typedef K_cgal::Point_3 Point_3_cgal;
typedef CGAL::Surface_mesh<Point_3_cgal> Surface_mesh_cgal;

// R-tree from Boost
typedef boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian> Point_3_boost;
typedef boost::geometry::index::rtree<Point_3_boost, boost::geometry::index::quadratic<16> > RTree;


class GModelRegion {
public:
    GModelRegion(easy3d::vec3 root, easy3d::vec3 root_dir);
    GModelRegion(GModelRegion const &other);
    ~GModelRegion();

    // getters
    const GraphGT& get_graph() { return graph_; }

    // setters
    void set_graph(GraphGT graph) {graph_ = graph; }

    // growth functions
    bool model_growth(const easy3d::PointCloud* points_in, const Surface_mesh_cgal* lobe_hull);
    bool grow_iteration();
    std::string read_line(std::string line);
    int kill_zone(easy3d::vec3 node);
    bool is_viable(easy3d::vec3 pot_loc);
    bool is_viable_lateral(easy3d::vec3 pot_loc, easy3d::vec3 p_parent);
    easy3d::vec3 compute_growth_direction(easy3d::vec3 p_parent);
    easy3d::vec3 compute_growth_direction_lateral(easy3d::vec3 pot_pt, easy3d::vec3 p_parent);
    std::tuple<double, double, double> points_to_movement(easy3d::vec3 p_parent, easy3d::vec3 p_next);
    std::string movement_to_string(std::tuple<double, double, double> movement);
    easy3d::vec3 compute_potential_location(std::string movement);
    double get_z_angle(easy3d::vec3 vec);
    double get_y_angle(easy3d::vec3 vec);

    // movement
    void go_forward(float distance);
    void go_rotate(float angle);
    void go_roll(float angle);
    void add_vertex(unsigned int n_parent);

private:
    // storage
    std::string axiom_;                         // current L-system starter string
//    std::map<std::string, std::string> rules_;  // L-system growth rules
    GraphGT graph_;                             // current generated skeleton graph (?)
    int num_iterations_ = 0;                    // number of iterations that were grown

    easy3d::PointCloud points_;                 // input point cloud
    Surface_mesh_cgal hull_mesh_;               // surface mesh of convex hull of lobe
    easy3d::vec3 translation_;                  // translation to original location (growth starts from (0, 0, 0)
    easy3d::vec3 rotation_;                     // rotation to original location (growth starts with [X?] is forward)
    easy3d::vec3 start_loc_;                    // position of lobe connector point
    easy3d::Mat<3, 3, float> start_plane_;      // direction of lobe connector point (needed to reset after growth iterations)
    // todo: kd tree (or R-tree?) of input points (+ new points)

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

    // parameters
    // todo: statistics parameters (angle, etc.)
    // todo: additional parameters (for region growing)

    std::map<std::string, float> stats_rg_;
    // todo: use dataframe, process in main growth model (so not more than 1 is stored, these could get big...)

};


#endif //TREEGROWTHMODELLING_GROWTH_MODEL_REGION_H
