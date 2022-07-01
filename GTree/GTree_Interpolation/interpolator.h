//
// Created by noort on 12/05/2022.
//

#ifndef EASY3D_INTERPOLATOR_H
#define EASY3D_INTERPOLATOR_H


#include <fstream>
#include <iostream>
#include <utility>
#include <vector>
#include <sstream>

#include <easy3d/core/spline_curve_fitting.h>
#include <easy3d/core/spline_curve_interpolation.h>

#include "graph_boost.h"


class Interpolator {
public:
    Interpolator();
    ~Interpolator();

    /// Import correspondence operation path files.
    void open_path_files(std::vector<std::string> path_files);

    /// Compute vertex and edge correspondences, construct intermediary graphs that can be morphed between 2 timestamp graphs.
    void compute_correspondence();

    /// Process edge correspondence operations.
    void correspond_edges(std::map<VertexDescriptorGraphB, VertexDescriptorGraphB> idx_map, int ts);

    /// Compute all intermediary positions (amount = nr_steps) of the nodes of the intermediary graphs.
    void interpolate();

    // original timestamp graphs
    std::vector<GraphB> graphs_ts;
    std::vector<GraphB> graphs_inter_;

    int nr_timestamps = 3; // todo: set properly
    int nr_steps = 5; // number of interpolation steps in between timestamps

    std::vector<std::map<VertexDescriptorGraphB, std::vector<easy3d::vec3> > > interpos;

private:
    // correspondence paths
    std::vector<std::vector<std::pair<int, int> >> paths_v_;
    std::vector<std::vector<std::pair<std::pair<int, int>, std::pair<int, int> > >> paths_e_;

};


#endif //EASY3D_INTERPOLATOR_H
