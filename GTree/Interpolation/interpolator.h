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

    void open_path_files(std::vector<std::string> path_files);
    void compute_correspondence();
    void correspond_edges(std::map<VertexDescriptorGraphB, VertexDescriptorGraphB> idx_map, int ts);
    void interpolate();

    // original timestamp graphs
    std::vector<GraphB> graphs_ts;
//    GraphB graph_ts0;
//    GraphB graph_ts1;
//    GraphB graph_ts2;
    std::vector<GraphB> graphs_inter_;

//    GraphB get_graph_01() const { return graph_inter_01_; }
//    GraphB get_graph_12() const { return graph_inter_12_; }
//    GraphB get_graph_inter(int ts) const { return graphs_inter_[ts]; }
    void set_graph_ts(int ts, GraphB g) { graphs_ts[ts] = g; }

    int nr_timestamps = 3; // todo: set properly
    int nr_steps = 5; // number of interpolation steps in between timestamps

    std::vector<std::map<VertexDescriptorGraphB, std::vector<easy3d::vec3> > > interpos;

private:
    // correspondence paths
//    std::vector<std::pair<int, int> > path_1_v_;                                    // vertex steps from ts 0 to ts 1
//    std::vector<std::pair<std::pair<int, int>, std::pair<int, int> > > path_1_e_;   // edge steps from ts 0 to ts 1
//    std::vector<std::pair<int, int> > path_2_v_;                                    // vertex steps from ts 1 to ts 2
//    std::vector<std::pair<std::pair<int, int>, std::pair<int, int> > > path_2_e_;   // edge steps from ts 1 to ts 2
    std::vector<std::vector<std::pair<int, int> >> paths_v_;
    std::vector<std::vector<std::pair<std::pair<int, int>, std::pair<int, int> > >> paths_e_;


    // intermediary graphs
//    GraphB graph_inter_01_;
//    GraphB graph_inter_12_;
//    std::vector<GraphB> graphs_inter_;

};


#endif //EASY3D_INTERPOLATOR_H
