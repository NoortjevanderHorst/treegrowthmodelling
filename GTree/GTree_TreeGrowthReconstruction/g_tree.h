//
// Created by noort on 23/01/2022.
//

#ifndef TREEGROWTHMODELLING_G_TREE_H
#define TREEGROWTHMODELLING_G_TREE_H


#include "skeleton_grow.h" // Lobe already imported in skeleton
#include "growth_model_region.h"


class GTree {
public:
    GTree();
    ~GTree();

    // todo: proper access methods

    bool add_ts_skeletons(std::vector<GSkeleton*> skels);   // store skeletons of timestamps
    bool construct_merged_cloud();  // reconstruct point cloud of all timestamps merged together
    bool add_merged_skeleton(GSkeleton* skel);  // store skeleton of merged model

    // representation
    bool compute_correspondence();
    bool compute_main_edges();
    bool constrain_ts_skeletons();
    bool compute_lobe_hulls();
    bool add_grown_lobes_to_corr();

    // growing
    bool grow_lobes();
    bool grow();
    bool interpolate();

    // utility
    easy3d::vec3 get_closest_point_on_line(easy3d::vec3 point, easy3d::vec3 line_left, easy3d::vec3 line_right);

    // getters
    easy3d::PointCloud* get_merged_cloud() const {return cloud_merged_;}
    GSkeleton* get_merged_skeleton() const {return skeleton_merged_;}
    int get_num_timestamps() const {return skeletons_.size();}
    std::vector<easy3d::SurfaceMesh*> get_lobe_meshes() const {return lobe_meshes_;}
    std::vector<std::pair<int, std::vector<Lobe*> > > get_lobes() const {return lobes_;}
    std::vector<easy3d::SurfaceMesh*> get_branches_meshes() const {return branches_meshes_;}

private:
    std::vector<std::pair<int, GSkeleton*> > skeletons_;    // store skeletons of all timestamps
    std::vector<std::pair<int, std::vector<Lobe*> > > lobes_;   // store lobe sets per timestamp
    // todo: vectors because expect to do mostly looping, is faster in vector than map (change if end up mostly indexing)
    std::vector<easy3d::SurfaceMesh*> lobe_meshes_; // vector of meshes per ts lobe set
    // todo: this could be integrated into the Lobe class, but it depends on what exactly we need later on from the lobes...
    std::vector<easy3d::SurfaceMesh*> branches_meshes_; // storing meshes of ts reconstructed branches
//    std::vector<easy3d::SurfaceMesh*> leaves_meshes_; // storing meshes of ts reconstructed leaves
    // todo: need to store leafs here as well?

    // store merged
    easy3d::PointCloud* cloud_merged_;
    GSkeleton* skeleton_merged_;
    easy3d::SurfaceMesh* branches_merged_;
//    easy3d::surfaceMesh* leaves_merged_;
    Vector3D* kd_points_merged_;
    KdTree* kdtree_merged_;

};


#endif //TREEGROWTHMODELLING_G_TREE_H
