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
