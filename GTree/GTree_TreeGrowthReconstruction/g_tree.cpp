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

#include "g_tree.h"


GTree::GTree() : kd_points_merged_(nullptr), kdtree_merged_(nullptr), skeleton_merged_(nullptr), cloud_merged_(nullptr){
    lobes_.clear();
    lobe_meshes_.clear();
    skeletons_.clear();
}


GTree::~GTree(){
    if (kdtree_merged_)
        delete kdtree_merged_;
    if (kd_points_merged_)
        delete kd_points_merged_;
    if (skeleton_merged_)
        delete skeleton_merged_;
    if (cloud_merged_)
        delete cloud_merged_;
    if (lobes_.size() > 0){
        lobes_.clear();
    }
    if (lobe_meshes_.size() > 0){
        lobe_meshes_.clear();
    }
    if (skeletons_.size() > 0){
        skeletons_.clear();
    }
}


bool GTree::add_ts_skeletons(std::vector<GSkeleton*> skels) {
    if (skels.empty()){
        std::cout << "ERROR: could not add skeletons to GTree, no skeletons given" << std::endl;
        return false;
    }

    int count = 0;
    if (skeletons_.empty()){
        count = 0;
    } else {
        count = skeletons_.size();
    }

    for (GSkeleton* skel : skels){
        skeletons_.emplace_back(count, skel);
        count++;
    }

    std::cout << "\nskeleton(s) added to GTree" << std::endl;
    for (auto si : skeletons_){
        std::cout << "index " << si.first << ", nr vertices: " << boost::num_vertices(si.second->get_mst()) << std::endl;
    }

    return true;
}


bool GTree::construct_merged_cloud() {
    // clean/init
    if (cloud_merged_)
        delete cloud_merged_;
    cloud_merged_ = new easy3d::PointCloud;

    std::cout << "\nstarting merged construction" << std::endl;
    for (auto skel : skeletons_){
        std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(skel.second->get_mst());
        for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit) {
            cloud_merged_->add_vertex(skel.second->get_mst()[*vit].coords);
        }

        std::cout << "points inserted, merged cloud has " << cloud_merged_->n_vertices() << " points" << std::endl;
    }

    std::cout << "final cloud has " << cloud_merged_->n_vertices() << " points" << std::endl;

    // set translation
    easy3d::PointCloud::ModelProperty<easy3d::Vec<3, double> > tp = cloud_merged_->model_property<easy3d::dvec3>("translation");
    auto trans = skeletons_[0].second->get_translation();
    tp[0] = static_cast<const easy3d::Vec<3, double>>(trans);

    return true;
}


bool GTree::add_merged_skeleton(GSkeleton* skel){
    if (skeleton_merged_)
        delete skeleton_merged_;
    skeleton_merged_ = skel;
    if (!skeleton_merged_) {
        std::cout << "ERROR: merged skeleton adding failed" << std::endl;
        return false;
    }
    return true;
}


bool GTree::compute_correspondence() {
    // todo: check if reconstruction of merged took place & timestamps were imported

    skeleton_merged_->set_corresponding(const_cast<GraphGT *>(&(skeleton_merged_->get_simplified())));

    GraphGT* graph = const_cast<GraphGT *>(&(skeleton_merged_->get_corresponding()));  // still don't get what happens here
    std::pair<EdgeIteratorGTGraph, EdgeIteratorGTGraph> et = edges(*graph);
    for (EdgeIteratorGTGraph eit = et.first; eit != et.second; ++eit){
        VertexDescriptorGTGraph v_source = source(*eit, *graph);
        VertexDescriptorGTGraph v_target = target(*eit, *graph);

        // find query zone
        double dist_radius = (*graph)[*eit].length / 2.0;
        double tolerance = 0.1; // todo: make editable parameter
        int density_needed = 1;     // how many points are needed to be close enough

        // kd-tree query of all points within the query zone
        Vector3D p_source = {(*graph)[v_source].coords.x, (*graph)[v_source].coords.y, (*graph)[v_source].coords.z};
        Vector3D p_target = {(*graph)[v_target].coords.x, (*graph)[v_target].coords.y, (*graph)[v_target].coords.z};

        KdTree* kdTree;
        for (auto timestamp : skeletons_){
            kdTree = timestamp.second->get_kdtree();

            kdTree->queryRange(p_source, 0.1, true);
            int hits_source = kdTree->getNOfFoundNeighbours();
            kdTree->queryRange(p_target, 0.1, true);
            int hits_target = kdTree->getNOfFoundNeighbours();

            if ((hits_source + hits_target) > density_needed){
                (*graph)[*eit].correspondence.insert(timestamp.first);
            }

            // todo: also a check with distance to line, to get better aligned search area
        }
    }

    std::cout << "\t-- done computing correspondence." << std::endl;

    return true;
}


bool GTree::compute_main_edges(){
    GraphGT* graph = const_cast<GraphGT *>(&(skeleton_merged_->get_corresponding()));

    // first, set all definite main flags
    std::vector<EdgeDescriptorGTGraph> main_detected;

    std::pair<EdgeIteratorGTGraph, EdgeIteratorGTGraph> et = edges(*graph);
    for (EdgeIteratorGTGraph eit = et.first; eit != et.second; ++eit){
        // edge is connected to root
        if ((target(*eit, *graph) == skeleton_merged_->get_root()) ||
            (source(*eit, *graph) == skeleton_merged_->get_root())){
            (*graph)[*eit].is_main = true;
            main_detected.push_back(*eit);
        }
        // edge is supported by all timestamps
        else if ((*graph)[*eit].correspondence.size() == skeletons_.size()){
            // branching level check (%)
            double level_cutoff = 0.6;
            double level_perc_curr = (*graph)[*eit].branch_level / skeleton_merged_->get_max_branching_level();
            if (level_perc_curr <= level_cutoff) {
                // leaf check (no original leaves can be main)
                // (leaf check should not hit root because previous check)
                if (!(degree(target(*eit, *graph), *graph) <= 1 || degree(source(*eit, *graph), *graph) <= 1)){
                    (*graph)[*eit].is_main = true;
                    main_detected.push_back(*eit);
                }
            }
        }
    }

    // connect all edges flagged as main into single skeleton
    while (!main_detected.empty()){
        // pop current edge from found main edge stack
        EdgeDescriptorGTGraph e_curr = main_detected.back();
        main_detected.pop_back();
        // find first vertex of edge
        VertexDescriptorGTGraph v_start = source(e_curr, *graph);
        if ((*graph)[v_start].parent == target(e_curr, *graph)){
            v_start = target(e_curr, *graph);
        }

        // traverse back to root, set all as main
        // todo: many redundant iterations, could be improved...
        VertexDescriptorGTGraph v_parent = (*graph)[v_start].parent;
        while (v_parent != skeleton_merged_->get_root()){
            EdgeDescriptorGTGraph e_parent = edge(v_start, v_parent, *graph).first;
            (*graph)[e_parent].is_main = true;
            v_start = v_parent;
            v_parent = (*graph)[v_start].parent;
        }
    }

    // now, simplify/advanced processing
    if (!skeleton_merged_->build_corresponding_merged()) {
        std::cout << "ERROR: could not build base merged correspondence skeleton" << std::endl;
        return false;
    }

    std::cout << "\t-- done computing main skeleton." << std::endl;
    return true;
}


bool GTree::constrain_ts_skeletons(){
    for (auto skel : skeletons_){
        // make corresponding_ skeleton (delaunay/mst constrained with main)
        skel.second->build_corresponding(skeleton_merged_->get_corresponding(), skeleton_merged_->get_root());
        // also make/store the timestamp specific graph
        skel.second->build_ts_main(skeleton_merged_->get_corresponding());

        // make lobe structures/flag lobe vertices in corresponding_ skeleton
        std::vector<Lobe*> lobes_curr;
        skel.second->detect_lobe_points(lobes_curr);
        lobes_.emplace_back(skel.first, lobes_curr);
    }
    return true;
}


bool GTree::compute_lobe_hulls() {
    if (lobes_.empty()){
        std::cout << "ERROR: could not make convex hulls, no lobes exist" << std::endl;
        return false;
    }

    for (auto lobeset : lobes_){
        auto mesh = new easy3d::SurfaceMesh;
        int cnt = 0;
        for (auto lobe : lobeset.second){
            lobe->build_lobe_hulls(mesh);
            cnt++;
        }
        lobe_meshes_.push_back(mesh);
    }

    return true;
}


bool GTree::add_grown_lobes_to_corr(){
    for (auto skel : skeletons_){
        std::cout << "adding grown structure to skel: " << skel.first << std::endl;

        GraphGT* graph_corr = const_cast<GraphGT *>(&(skel.second->get_corresponding()));

        // delete all non-main edges
        std::pair<VertexIteratorGTGraph , VertexIteratorGTGraph> vt = vertices(*graph_corr);
        for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
            if (!((*graph_corr)[*vit].delete_mark || *vit == skel.second->get_root_corr())){
                EdgeDescriptorGTGraph e_to_parent = edge((*graph_corr)[*vit].parent, *vit, (*graph_corr)).first;
                if (!(*graph_corr)[e_to_parent].is_main){
                    (*graph_corr)[*vit].clear_mark = true;
                }
            }
        }

        // manual clean_graph() because of root
        for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
            if ((*graph_corr)[*vit].clear_mark){
                // do not delete the root or vertices already deleted previously
                if (*vit != skel.second->get_root_corr() && (!(*graph_corr)[*vit].delete_mark)){
                    // if vertex is not a leaf, reroute connecting edges around it
                    if (degree(*vit, (*graph_corr)) >= 2){
                        VertexDescriptorGTGraph parent = (*graph_corr)[*vit].parent;
                        std::pair<AdjacencyIteratorGTGraph, AdjacencyIteratorGTGraph> avt = adjacent_vertices(*vit, (*graph_corr));
                        for (AdjacencyIteratorGTGraph ait = avt.first; ait != avt.second; ++ait){
                            // process all successive vertices
                            if (*ait != parent){
                                (*graph_corr)[*ait].parent = parent;

                                SGraphEdgePropGT e_new;
                                e_new.length = (*graph_corr)[*ait].coords.distance2((*graph_corr)[parent].coords);
                                // set new edge as main if deleted edge to next was also main
                                if ((*graph_corr)[edge(*vit, *ait, (*graph_corr)).first].is_main){
                                    e_new.is_main = true;
                                }
                                add_edge(parent, *ait, e_new, (*graph_corr));
                            }
                        }
                    }

                    // reset properties
                    (*graph_corr)[*vit].parent = *vit;
                    (*graph_corr)[*vit].branch_level = -1;
                    (*graph_corr)[*vit].normal = {0, 0, 0};
                    (*graph_corr)[*vit].weight = -1;

                    boost::clear_vertex(*vit, (*graph_corr));

                    (*graph_corr)[*vit].delete_mark = true;
                }
            }
        }

        // add skeleton per lobe
        auto lobeset = lobes_[skel.first];
        for (Lobe* lobe : lobeset.second){
            GraphGT subgraph = lobe->get_subgraph();
            // make sure only non-ignored lobes / lobes that could be grown are used
            if (num_vertices(subgraph) > 1){
                std::vector<VertexDescriptorGTGraph> wait_list;
                wait_list.push_back(0); // connector node always first added to lobe graph
                std::map<VertexDescriptorGTGraph, VertexDescriptorGTGraph> idx_map; // lobe index <--> corr skel index
                idx_map[0] = lobe->get_connector_node();

                while (!wait_list.empty()){
                    VertexDescriptorGTGraph v_curr = wait_list.back();
                    wait_list.pop_back();

                    // add all subtree nodes and edges to the corresponding skeleton graph
                    for (VertexDescriptorGTGraph v_child: make_iterator_range(adjacent_vertices(v_curr, subgraph))){
                        if (v_child != subgraph[v_curr].parent){
                            SGraphVertexPropGT v_new;
                            v_new.coords = subgraph[v_child].coords;
                            v_new.lobe_index = subgraph[v_child].lobe_index;
                            v_new.parent = idx_map[v_curr];

                            VertexDescriptorGTGraph v_added_idx = add_vertex(v_new, *graph_corr);
                            idx_map[v_child] = v_added_idx;

                            EdgeDescriptorGTGraph e_lobe = edge(v_curr, v_child, subgraph).first;
                            SGraphEdgePropGT e_new;
                            e_new.length = subgraph[e_lobe].length;
                            e_new.length_to_root = subgraph[e_lobe].length_to_root;

                            add_edge(idx_map[v_curr], v_added_idx, e_new, *graph_corr);

                            wait_list.push_back(v_child);
                        }
                    }
                }
            }
        }
    }

    return true;
}


bool GTree::grow_lobes() {
    if (lobes_.empty()){
        std::cout << "ERROR: could not grow in lobes, no lobes exist" << std::endl;
        return false;
    }

    for (auto lobeset : lobes_){
        std::cout << "Growing lobes of timestamp " << lobeset.first << " ... ";

        int cnt = 0;
        for (auto lobe : lobeset.second){
            // todo: find all original point cloud points within convex hull

            easy3d::PointCloud *pc = const_cast<easy3d::PointCloud *>(&lobe->get_pointcloud());
            Surface_mesh_cgal hull = lobe->get_convex_hull();

            // only process lobe if big enough
            // todo: other constraints? lobe processing (bigger, smoother, more orb-shaped?)
            if (CGAL::Polygon_mesh_processing::volume(hull) >= 1) {
                easy3d::vec3 root = skeletons_[lobeset.first].second->get_corresponding()[lobe->get_connector_node()].coords;
                easy3d::vec3 dir = root - skeletons_[lobeset.first].second->get_corresponding()[
                        skeletons_[lobeset.first].second->get_corresponding()[lobe->get_connector_node()].parent
                ].coords;

                GModelRegion gm_region(root, dir);
                gm_region.model_growth(pc, &hull);

                lobe->set_skeleton(gm_region.get_graph());
            } else {
                // assign cleared graph as skeleton (otherwise still set as part from MST)
                GraphGT lobe_graph = lobe->get_subgraph();
                lobe_graph.clear();
                lobe->set_skeleton(lobe_graph);
            }
            cnt++;
        }
        std::cout << "done." << std::endl;
    }

    // add generated lobe skeleton to corresponding skeleton ts
    add_grown_lobes_to_corr();

    return true;
}


bool GTree::interpolate(){
    // going backwards so current base knows alterations that will be done to target in next interpolation
    for (int idx_skel = (skeletons_.size() - 2); idx_skel >= 0; -- idx_skel){

        std::cout << "computing correspondences for base idx: " << idx_skel << ", target idx: " << idx_skel + 1 << std::endl;

        GraphGT* g_base = const_cast<GraphGT *>(&(skeletons_[idx_skel].second->get_ts_main()));
        GraphGT* g_target = const_cast<GraphGT *>(&(skeletons_[idx_skel + 1].second->get_ts_main()));
        VertexDescriptorGTGraph rootv_base = skeletons_[idx_skel].second->get_root();
        VertexDescriptorGTGraph rootv_target = skeletons_[idx_skel + 1].second->get_root();
        (*g_target)[rootv_target].corr_based = true;

        // find all non-deleted vertices of target
        std::map<unsigned int, VertexDescriptorGTGraph> idx_map;
        int cnt = 0;
        std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt_t = vertices(*g_target);
        for (VertexIteratorGTGraph vit = vt_t.first; vit != vt_t.second; ++vit){
            if (!((*g_target)[*vit].delete_mark)){
                idx_map[cnt] = *vit;
                cnt++;
            }
        }

        // make kd tree of target nodes
        int nr_points = cnt;
        auto kd_pts = new Vector3D[nr_points];

        for (auto t_pts: idx_map){
            kd_pts[t_pts.first].x = (*g_target)[t_pts.second].coords.x;
            kd_pts[t_pts.first].y = (*g_target)[t_pts.second].coords.y;
            kd_pts[t_pts.first].z = (*g_target)[t_pts.second].coords.z;
        }

        auto kdtree_target = new KdTree(kd_pts, nr_points, 16);

        // parameters
        // todo: paramter access
        // distance within which points in target are deemed close enough to base to correspond
        float dist_tol_upper = 1.0; // conditionally correspond
        float dist_tol_lower = 0.2; // immediately correspond
        float angle_tol = 45 * M_PI / 180;  // allowed angle diff node-parent <> target corr edge [rad]

        // set nn search to limited nr of neighbours
        kdtree_target->setNOfNeighbours(3);

        // set roots to correspond
        (*g_base)[rootv_base].inter_target = rootv_target;
        (*g_target)[rootv_target].inter_base = rootv_base;

        // search for initial correspondences
        std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt_base = vertices(*g_base);
        for (VertexIteratorGTGraph vit = vt_base.first; vit != vt_base.second; ++vit){
            // search all base vertices that are main and not deleted
            if (*vit != rootv_base && !((*g_base)[*vit].delete_mark)) { // make sure root is not reassigned
                Vector3D p_base = {(*g_base)[*vit].coords.x, (*g_base)[*vit].coords.y, (*g_base)[*vit].coords.z};
                kdtree_target->queryPosition(p_base);
                int nr_nbors = kdtree_target->getNOfFoundNeighbours();

                // select best candidate out of closest neighbours
                if (nr_nbors > 0) {
                    VertexDescriptorGTGraph v_corr;
                    bool corr_found = false;
                    bool close_corr_found = false;
                    double dist_corr = 999;
                    double angle_corr = 999;
                    for (int i = 0; i < nr_nbors; ++i) {
                        unsigned int i_nbor = kdtree_target->getNeighbourPositionIndex(i);
                        VertexDescriptorGTGraph v_nbor = idx_map[i_nbor];
                        double dist_diff = ((*g_target)[v_nbor].coords - (*g_base)[*vit].coords).length();

                        // see if they are very close to each other (immediately correspond)
                        if (dist_diff <= dist_tol_lower) {
                            if (dist_diff < dist_corr) {
                                // process doubles: do not assign if worse
                                if ((*g_target)[v_nbor].inter_base != -1) {
                                    VertexDescriptorGTGraph old_base = (*g_target)[v_nbor].inter_base;
                                    double dist_diff_old = ((*g_target)[v_nbor].coords -
                                                            (*g_base)[old_base].coords).length();

                                    // new is much closer
                                    // new has much more similar degree
                                    if (dist_diff_old - dist_diff > dist_tol_lower ||
                                        (abs(int(degree(old_base, *g_base) - degree(v_nbor, *g_target))) -
                                         abs(int(degree(*vit, *g_base) - degree(v_nbor, *g_target))) > 2)) {
                                        // set new
                                        v_corr = v_nbor;
                                        corr_found = true;
                                        close_corr_found = true;
                                        dist_corr = dist_diff;
                                    }
                                }
                                    // target vert has not been corresponded to yet
                                else {
                                    v_corr = v_nbor;
                                    corr_found = true;
                                    close_corr_found = true;
                                    dist_corr = dist_diff;
                                }
                            }
                        }
                        // see if they are close enough to correspond within additional constraints
                        else if (dist_diff <= dist_tol_upper) {
                            easy3d::vec3 e_base = (*g_base)[(*g_base)[*vit].parent].coords - (*g_base)[*vit].coords;
                            easy3d::vec3 e_target = (*g_target)[(*g_target)[v_nbor].parent].coords - (*g_target)[v_nbor].coords;
                            double angle_diff = acos(dot(e_base, e_target) / (length(e_base) * length(e_target)));

                            if (!close_corr_found && angle_diff < angle_tol && angle_diff < angle_corr) {
                                // process doubles: do not assign if worse
                                if ((*g_target)[v_nbor].inter_base != -1) {
                                    VertexDescriptorGTGraph old_base = (*g_target)[v_nbor].inter_base;
                                    double dist_diff_old = ((*g_target)[v_nbor].coords -
                                                            (*g_base)[old_base].coords).length();
                                    easy3d::vec3 e_base_old =
                                            (*g_base)[(*g_base)[old_base].parent].coords - (*g_base)[old_base].coords;
                                    double angle_diff_old = acos(
                                            dot(e_base_old, e_target) / (length(e_base_old) * length(e_target)));

                                    // new is much closer
                                    // new is much better angle
                                    // new has much more similar degree
                                    if (dist_diff_old - dist_diff > dist_tol_lower ||
                                        angle_diff_old - angle_diff > (10 * M_PI / 180) ||
                                        (abs(int(degree(old_base, *g_base) - degree(v_nbor, *g_target))) -
                                         abs(int(degree(*vit, *g_base) - degree(v_nbor, *g_target))) > 2)) {
                                        // set new
                                        v_corr = v_nbor;
                                        corr_found = true;
                                        dist_corr = dist_diff;
                                    }
                                }
                                    // target vert has not been corresponded to yet
                                else {
                                    v_corr = v_nbor;
                                    corr_found = true;
                                    angle_corr = angle_diff;
                                }
                            }
                        }
                    }

                    // set indices if found
                    if (corr_found) {
                        // remove old if it was a double
                        if ((*g_target)[v_corr].inter_base != -1) {
                            (*g_base)[(*g_target)[v_corr].inter_base].inter_target = -1;
                        }
                        // set new correspondence
                        (*g_target)[v_corr].inter_base = *vit;
                        (*g_base)[*vit].inter_target = v_corr;
                    }
                }
            }
        }


        //--- walk up from root and improve

        std::vector<VertexDescriptorGTGraph> branch_children;
        branch_children.push_back(rootv_base);

        std::vector<VertexDescriptorGTGraph> branch_based;
        branch_based.push_back(rootv_target);
        std::vector<VertexDescriptorGTGraph> branch_split_target = branch_based;
        std::vector<VertexDescriptorGTGraph> branch_fill_target = branch_based;

        // correspond unassigned children of corresponding verts
        while (!branch_children.empty()){
            VertexDescriptorGTGraph v_curr = branch_children.back();
            branch_children.pop_back();

            // see if curr has correspondence, close child correspondences, see if steps can be taken
            if ((*g_base)[v_curr].inter_target != -1){
                for (VertexDescriptorGTGraph v_child : make_iterator_range(adjacent_vertices(v_curr, *g_base))) {
                    // child is not parent and is part of main skeleton
                    if (v_child != (*g_base)[v_curr].parent && !((*g_base)[v_child].delete_mark)) {
                        // try to find a suitable candidate for the child
                        int v_candi = -1;

                        //- if child is leaf, see if target also has leaf children, assign one with smallest angle
                        if (degree(v_child, *g_base) == 1) {
                            // if it already corresponds, see if correct, remove if not
                            if ((*g_base)[v_child].inter_target != -1) {
                                if ((*g_target)[(*g_base)[v_child].inter_target].parent != (*g_base)[v_curr].inter_target) {
                                    // todo: allow for one/a few extra steps
                                    (*g_target)[(*g_base)[v_child].inter_target].inter_base = -1;
                                    (*g_target)[(*g_base)[v_child].inter_target].inter_add_mark = false;
                                    (*g_base)[v_child].inter_target = -1;
                                }
                            }

                            // see if child is lone
                            if ((*g_base)[v_child].inter_target == -1) {
                                double angle_corr = 999;
                                for (VertexDescriptorGTGraph v_child_target: make_iterator_range(adjacent_vertices((*g_base)[v_curr].inter_target, *g_target))) {
                                    // child is not parent and is part of main skeleton
                                    if (v_child_target != (*g_target)[v_child_target].parent && !((*g_target)[v_child_target].delete_mark)) {
                                        // assign a leaf immediately if found (even if it already corresponds)
                                        if (degree(v_child_target, *g_target) == 1) {
                                            v_candi = v_child_target;
                                            break;  // no need to search further
                                        }
                                        // if target is not leaf, assign if the angle between the two edges is small, leave otherwise
                                        else {
                                            // only assign lone target vertices
                                            // todo: can incorporate already assigned ones if it is a better match
                                            if ((*g_target)[v_child_target].inter_base == -1) {
                                                easy3d::vec3 e_base = (*g_base)[v_curr].coords - (*g_base)[v_child].coords;
                                                easy3d::vec3 e_target = (*g_target)[(*g_base)[v_curr].inter_target].coords -
                                                        (*g_target)[v_child_target].coords;
                                                double angle_diff = acos(dot(e_base, e_target) /
                                                                         (length(e_base) * length(e_target)));
                                                // assign smallest angle, if small enough
                                                if (angle_diff <= (35 * M_PI / 180) && angle_diff < angle_corr) {
                                                    v_candi = v_child_target;
                                                    angle_corr = angle_diff;
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                        } else {
                            // todo: non-leafs
                        }

                        // assign candidate if found
                        if (v_candi != -1){
                            // delete previous assignment if there was one
                            if ((*g_target)[v_candi].inter_base != -1){
                                (*g_base)[(*g_target)[v_candi].inter_base].inter_target = -1;
                            }
                            (*g_base)[v_child].inter_target = v_candi;
                            (*g_target)[v_candi].inter_base = v_child;

                        }

                        // if child is already corresponding, check if corr is child of leaf, REMOVE IF NOT
                        // - also assign better one if possible

                        // if child is not corresponding, see if any non-corresponding children of target qualify
                        // - if not, check if any corresponding children of target correct to a non-child of base
                        //      - REMOVE IF SO
                        //      - see if these are a match


                    }
                }
            }

            // add all children to queue
            for (VertexDescriptorGTGraph v_child : make_iterator_range(adjacent_vertices(v_curr, *g_base))){
                if (v_child != (*g_base)[v_curr].parent) {
                    if (!((*g_base)[v_child].delete_mark)) {
                        branch_children.push_back(v_child);
                    }
                }
            }
        }

        // make sure every correspondence detected can be walked back to target root without correspondence inconsistencies
        while (!branch_based.empty()){
            VertexDescriptorGTGraph v_curr = branch_based.back();
            branch_based.pop_back();

            // current vertex could be a tip
            if ((*g_target)[v_curr].inter_base != -1){

                // see if current vertex really is a tip
                bool tip_found = false;

                if (degree(v_curr, *g_target) == 1 && v_curr != rootv_target){
                    tip_found = true;
                } else {

                    std::vector<VertexDescriptorGTGraph> child_waitlist;
                    bool corr_found = false;

                    for (VertexDescriptorGTGraph v_child : make_iterator_range(adjacent_vertices(v_curr, *g_target))) {
                        if (v_child != (*g_target)[v_curr].parent && !((*g_target)[v_child].delete_mark)) {
                            child_waitlist.push_back(v_child);
                        }
                    }


                    while (!child_waitlist.empty() && !corr_found){
                        VertexDescriptorGTGraph v_child_curr = child_waitlist.back();
                        child_waitlist.pop_back();

                        // if child is not corresponding, add children to waitlist
                        if ((*g_target)[v_child_curr].inter_base == -1){

                            for (VertexDescriptorGTGraph v_child_child : make_iterator_range(adjacent_vertices(v_child_curr, *g_target))){
                                if (v_child_child != (*g_target)[v_child_curr].parent && !((*g_target)[v_child_child].delete_mark)) {
                                    child_waitlist.push_back(v_child_child);
                                }
                            }
                        }
                        // if it corresponds, v curr cannot be tip
                        else {
                            corr_found = true;
                        }
                    }

                    // if no corrs were found in all children, v curr is a tip
                    if (!corr_found){
                        tip_found = true;
                    }
                }

                // if v_curr is tip, walk back to first based vertex (to root)
                if (tip_found){
                    (*g_target)[v_curr].corr_tip = true;

                    // take steps back until a based vertex is reached
                    std::vector<VertexDescriptorGTGraph> back_list;
                    back_list.push_back(v_curr);

                    while (!back_list.empty()){
                        VertexDescriptorGTGraph v_step = back_list.back();
                        back_list.pop_back();
                        // (back list always has just 1 vertex, loop will just stop if next parent is based (not added))

                        // see if the step vertex qualifies as bifur point
                        if (degree(v_step, *g_target) > 2 && v_step != v_curr){
                            if (!((*g_target)[v_step].inter_tips.empty())){
                                (*g_target)[v_step].inter_bifur_mark = true;
                            }
                        }

                        // assign tip it leads to to vertex
                        (*g_target)[v_step].inter_tips.push_back(v_curr);

                        // next step back is not based
                        if (!((*g_target)[v_step].corr_based)){

                            //- fix any possible issues

                            // if it corresponds, check if correct
                            if ((*g_target)[v_step].inter_base != -1){
                                VertexDescriptorGTGraph v_step_base = (*g_target)[v_step].inter_base;

                                VertexDescriptorGTGraph v_next_corr_target = (*g_target)[v_step].parent;
                                VertexDescriptorGTGraph v_next_corr_base = (*g_base)[v_step_base].parent;

                                // store intermediary vertices
                                std::vector<VertexDescriptorGTGraph> path_target;
                                std::vector<VertexDescriptorGTGraph> path_base;

                                while ((*g_target)[v_next_corr_target].inter_base == -1){
                                    v_next_corr_target = (*g_target)[v_next_corr_target].parent;
                                    path_target.push_back((*g_target)[v_next_corr_target].parent);
                                }
                                while ((*g_base)[v_next_corr_base].inter_target == -1){
                                    v_next_corr_base = (*g_base)[v_next_corr_base].parent;
                                    path_base.push_back((*g_base)[v_next_corr_base].parent);
                                }

                                // if they're not the same, correspondence is not correct
                                if ((*g_target)[v_next_corr_target].inter_base != v_next_corr_base){
                                    // delete target edge to rest of branch
                                    remove_edge(v_step_base, (*g_base)[v_step_base].parent, *g_base);

                                    // make base edge to first target correspondence instead
                                    (*g_base)[v_step_base].parent = (*g_target)[v_next_corr_target].inter_base;
                                    add_edge((*g_target)[v_next_corr_target].inter_base, v_step_base, *g_base);

                                    // update base path
                                    path_base.clear();
                                    if ((*g_base)[v_step_base].parent != (*g_target)[v_next_corr_target].inter_base){
                                        VertexDescriptorGTGraph v_path = (*g_base)[v_step_base].parent;
                                        while (v_path != (*g_target)[v_next_corr_target].inter_base){
                                            path_base.push_back(v_path);
                                            v_path = (*g_base)[v_path].parent;
                                        }
                                    }
                                }

                                // todo: paths not used here...
                            }

                            //- take another step back

                            back_list.push_back((*g_target)[v_step].parent);
                            (*g_target)[v_step].corr_based = true;
                        }
                    }
                }
            }

            // continue path
            for (VertexDescriptorGTGraph v_child : make_iterator_range(adjacent_vertices(v_curr, *g_target))){
                if (v_child != (*g_target)[v_curr].parent) {
                    if (!((*g_target)[v_child].delete_mark)) {
                        branch_based.push_back(v_child);
                    }
                }
            }
        }

        // fix bifurcation points in target that do not have a correspondence yet
        // (branches need to connect to a corresponding splitting point in both timestamps...)
        while (!branch_split_target.empty()){
            VertexDescriptorGTGraph v_curr = branch_split_target.back();
            branch_split_target.pop_back();

            // check all empty bifurcation points with multiple tips attached
            if ((*g_target)[v_curr].inter_base == -1 && (*g_target)[v_curr].inter_bifur_mark && v_curr != rootv_target){

                (*g_target)[v_curr].inter_bifur_mark_temp = true;

                VertexDescriptorGTGraph v_bifur_base;

                for (int i_tip = 0; i_tip < (*g_target)[v_curr].inter_tips.size(); ++i_tip){
                    // find corresponding vertices for tip current and the next corresponding parent
                    VertexDescriptorGTGraph v_tip = (*g_target)[v_curr].inter_tips[i_tip];
                    VertexDescriptorGTGraph v_tip_base = (*g_target)[v_tip].inter_base;
                    VertexDescriptorGTGraph v_next_corr_target = v_tip;
                    VertexDescriptorGTGraph v_step_target = (*g_target)[v_tip].parent;

                    while (v_step_target != v_curr){
                        if ((*g_target)[v_step_target].inter_base != -1){
                            v_next_corr_target = v_step_target;
                        }
                        v_step_target = (*g_target)[v_step_target].parent;
                    }

                    VertexDescriptorGTGraph v_next_corr_base = (*g_target)[v_next_corr_target].inter_base;

                    // for first tip, make/find a shared bifur vertex in base
                    if (i_tip == 0){
                        // parent of base tip is unassigned: use
                        if ((*g_base)[(*g_base)[v_next_corr_base].parent].inter_target == -1){
                            v_bifur_base = (*g_base)[v_next_corr_base].parent;
                            (*g_target)[v_curr].inter_base = v_bifur_base;
                            (*g_base)[v_bifur_base].inter_target = v_curr;
                        }
                        // make intermediary vertex
                        else {
                            // add intermediary vertex
                            // todo: put at closest to target midpoint (for now just middle of base edge)
                            VertexDescriptorGTGraph v_parent_base = (*g_base)[v_next_corr_base].parent;
                            SGraphVertexPropGT v_new;
                            v_new.parent = v_parent_base;
                            v_new.coords = ((*g_base)[v_parent_base].coords + (*g_base)[v_next_corr_base].coords) / 2;
                            v_new.is_main = true;
                            v_new.inter_target = v_curr;
                            VertexDescriptorGTGraph v_mid = add_vertex(v_new, *g_base);

                            // update parent of child
                            (*g_base)[v_next_corr_base].parent = v_mid;

                            // correspond back to new vert in target
                            (*g_target)[v_curr].inter_base = v_mid;

                            // remove old edge
                            remove_edge(v_parent_base, v_next_corr_base, *g_base);

                            // add new edges
                            auto ea_0 = add_edge(v_parent_base, v_mid, *g_base);
                            auto ea_1 = add_edge(v_mid, v_next_corr_base, *g_base);
                            (*g_base)[ea_0.first].is_main = true;
                            (*g_base)[ea_1.first].is_main = true;

                            v_bifur_base = v_mid;
                        }

                    }
                    // for the rest of the tips, remove edges to old base and attach to new bifur vertex
                    else {
                        // remove old edge
                        remove_edge((*g_base)[v_next_corr_base].parent, v_next_corr_base, *g_base);

                        // add new edge
                        auto ea_0 = add_edge(v_bifur_base, v_next_corr_base,  *g_base);
                        (*g_base)[ea_0.first].is_main = true;

                        (*g_base)[v_next_corr_base].parent = v_bifur_base;
                    }

                }
            }

            // continue path
            for (VertexDescriptorGTGraph v_child : make_iterator_range(adjacent_vertices(v_curr, *g_target))){
                if (v_child != (*g_target)[v_curr].parent) {
                    if (!((*g_target)[v_child].delete_mark)) {
                        branch_split_target.push_back(v_child);
                    }
                }
            }
        }

        // connect every vert of target to one of base (fill gaps in correspondence/path)
        while (!branch_fill_target.empty()){
            VertexDescriptorGTGraph v_curr = branch_fill_target.back();
            branch_fill_target.pop_back();

            // if corresponds, find previous corresponding, assign/make/merge any intermediates where needed
            if ((*g_target)[v_curr].inter_base != -1 && v_curr != rootv_target){
                VertexDescriptorGTGraph v_curr_base = (*g_target)[v_curr].inter_base;

                VertexDescriptorGTGraph v_next_corr_target = (*g_target)[v_curr].parent;
                VertexDescriptorGTGraph v_next_corr_base = (*g_base)[v_curr_base].parent;

                // store intermediary vertices
                std::vector<VertexDescriptorGTGraph> path_target;
                std::vector<VertexDescriptorGTGraph> path_base;

                while ((*g_target)[v_next_corr_target].inter_base == -1){
                    path_target.push_back(v_next_corr_target);
                    v_next_corr_target = (*g_target)[v_next_corr_target].parent;
                }
                while ((*g_base)[v_next_corr_base].inter_target == -1){
                    path_base.push_back(v_next_corr_base);
                    v_next_corr_base = (*g_base)[v_next_corr_base].parent;
                }

                // establish correspondences for existing base vertices
                if (!path_base.empty() && !path_target.empty()){
                    // base is smaller than target: choose target verts
                    if (path_base.size() <= path_target.size()){
                        // add more correspondences for all non-assigned base verts
                        int i_offset = 0;
                        for (int i_base = 0; i_base < path_base.size(); ++i_base){
                            // walk target path until closest vert is found
                            int closest_i = i_base + i_offset;
                            int i_curr = closest_i;
                            double dist_min = 999;
                            while ((i_curr + (path_base.size() - i_base)) < path_target.size()){
                                double dist_curr = ((*g_base)[path_base[i_base]].coords -
                                                    (*g_target)[path_target[i_curr]].coords).length();
                                if (dist_curr < dist_min){
                                    dist_min = dist_curr;
                                    closest_i = i_curr;
                                }
                                i_curr ++;
                            }
                            i_offset = closest_i - i_base;

                            // set correspondence
                            (*g_base)[path_base[i_base]].inter_target = path_target[closest_i];
                            (*g_target)[path_target[closest_i]].inter_base = path_base[i_base];
                        }
                    }
                    // base is larger than target: choose base verts
                    else {
                        int i_offset = 0;
                        for (int i_target = 0; i_target < path_target.size(); ++i_target){
                            // walk target path until closest vert is found
                            int closest_i = i_target + i_offset;
                            int i_curr = closest_i;
                            double dist_min = 999;
                            while ((i_curr + (path_target.size() - i_target)) < path_base.size()){
                                double dist_curr = ((*g_target)[path_target[i_target]].coords -
                                                    (*g_base)[path_base[i_curr]].coords).length();
                                if (dist_curr < dist_min){
                                    dist_min = dist_curr;
                                    closest_i = i_curr;
                                }
                                i_curr ++;
                            }
                            i_offset = closest_i - i_target;

                            // set correspondence
                            (*g_base)[path_base[closest_i]].inter_target = path_target[i_target];
                            (*g_target)[path_target[i_target]].inter_base = path_base[closest_i];
                        }
                    }
                }

                if (!path_target.empty()){
                    // base vertices need to be added
                    if (path_base.size() <= (path_target.size())){
                        // add new base vertices for the rest
                        for (int i_target = 0; i_target < path_target.size(); ++i_target){
                            VertexDescriptorGTGraph v_path_target = path_target[i_target];

                            if ((*g_target)[v_path_target].inter_base == -1){
                                VertexDescriptorGTGraph v_next;
                                if (i_target == 0){
                                    v_next = v_curr;
                                } else {
                                    v_next = path_target[i_target - 1];
                                }

                                VertexDescriptorGTGraph v_next_base = (*g_target)[v_next].inter_base;
                                VertexDescriptorGTGraph v_parent_base = (*g_base)[v_next_base].parent;

                                // get coordinates of intermediary point
                                easy3d::vec3 p_closest = get_closest_point_on_line((*g_target)[v_path_target].coords,
                                                                                   (*g_base)[v_parent_base].coords,
                                                                                   (*g_base)[v_next_base].coords);

                                // add intermediary vertex
                                SGraphVertexPropGT v_new;
                                v_new.parent = v_parent_base;
                                v_new.coords = p_closest;
                                v_new.is_main = true;
                                v_new.inter_target = v_path_target;
                                VertexDescriptorGTGraph v_mid = add_vertex(v_new, *g_base);

                                // update parent of child
                                (*g_base)[v_next_base].parent = v_mid;

                                // correspond back to new vert in target
                                (*g_target)[v_path_target].inter_base = v_mid;

                                // remove old edge
                                remove_edge(v_parent_base, v_next_base, *g_base);

                                // add new edges
                                auto ea_0 = add_edge(v_parent_base, v_mid, *g_base);
                                auto ea_1 = add_edge(v_mid, v_next_base, *g_base);
                                (*g_base)[ea_0.first].is_main = true;
                                (*g_base)[ea_1.first].is_main = true;
                            }
                        }
                    }

                    // base vertices need to be collapsed
                    else if (path_base.size() > path_target.size()){
                        // simplify base edge to delete vertex
                        for (int i_base = 0; i_base < path_base.size(); ++i_base){

                            // make sure root does not ever get deleted
                            // vertices just previously matched with target should also not be deleted
                            if (path_base[i_base] != rootv_base && (*g_base)[path_base[i_base]].inter_target == -1) {
                                VertexDescriptorGTGraph v_path_base = path_base[i_base];
                                VertexDescriptorGTGraph v_parent_base = (*g_base)[v_path_base].parent;

                                // reroute ALL edges around the to be deleted vertex (connect all to parent)
                                for (VertexDescriptorGTGraph v_child: make_iterator_range(
                                        adjacent_vertices(v_path_base, *g_base))) {
                                    if (v_child != v_parent_base) {
                                        // update parent
                                        (*g_base)[v_child].parent = v_parent_base;

                                        // add new edge
                                        auto e_new = add_edge(v_parent_base, v_child, *g_base);
                                        (*g_base)[e_new.first].is_main = true;
                                    }
                                }

                                // delete vertex
                                clear_vertex(v_path_base, *g_base);
                                (*g_base)[v_path_base].delete_mark = true;
                            }

                        }
                    }
                }
                // target path is empty but base is not: also collapse these vertices
                else {
                    if (!path_base.empty()){
                        // simplify base edge to delete vertex
                        for (int i_base = 0; i_base < path_base.size(); ++i_base){
                            if (path_base[i_base] != rootv_base) {

                                VertexDescriptorGTGraph v_path_base = path_base[i_base];
                                VertexDescriptorGTGraph v_parent_base = (*g_base)[v_path_base].parent;

                                // reroute ALL edges around the to be deleted vertex (connect all to parent)
                                for (VertexDescriptorGTGraph v_child: make_iterator_range(
                                        adjacent_vertices(v_path_base, *g_base))) {
                                    if (v_child != v_parent_base) {
                                        // update parent
                                        (*g_base)[v_child].parent = v_parent_base;

                                        // add new edge
                                        auto e_new = add_edge(v_parent_base, v_child, *g_base);
                                        (*g_base)[e_new.first].is_main = true;
                                    }
                                }

                                // delete vertex
                                clear_vertex(v_path_base, *g_base);
                                (*g_base)[v_path_base].delete_mark = true;
                            }
                        }
                    }
                }
            }

            // continue path
            for (VertexDescriptorGTGraph v_child : make_iterator_range(adjacent_vertices(v_curr, *g_target))){
                if (v_child != (*g_target)[v_curr].parent) {
                    if (!((*g_target)[v_child].delete_mark)) {
                        branch_fill_target.push_back(v_child);
                    }
                }
            }
        }


        //--- flag everything correctly

        // mark vertices in target without correspondence as added
        std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt_target = vertices(*g_target);
        for (VertexIteratorGTGraph vit = vt_target.first; vit != vt_target.second; ++vit){
            // make sure only main existing vertices get marked as added
            if (!((*g_target)[*vit].delete_mark)) {
                if ((*g_target)[*vit].inter_base == -1) {
                    (*g_target)[*vit].inter_add_mark = true;
                }
            }
        }

        // mark vertices in base without correspondence as to be deleted
        for (VertexIteratorGTGraph vit = vt_base.first; vit != vt_base.second; ++vit){
            // make sure only main existing vertices get marked as added
            if (!((*g_base)[*vit].delete_mark)) {
                if ((*g_base)[*vit].inter_target == -1) {
                    (*g_base)[*vit].inter_delete_mark = true;
                }
            }
        }

        // source edges: see if correspond, delete otherwise
        std::pair<EdgeIteratorGTGraph, EdgeIteratorGTGraph> edb = boost::edges(*g_base);
        for (EdgeIteratorGTGraph eit = edb.first; eit != edb.second; ++eit) {
            VertexDescriptorGTGraph v_source_b = source(*eit, *g_base);
            VertexDescriptorGTGraph v_target_b = target(*eit, *g_base);

            int v_source_t = (*g_base)[v_source_b].inter_target;
            int v_target_t = (*g_base)[v_target_b].inter_target;

            // see if there is a double vert (merged), find correct target if so
            if ((*g_base)[v_source_b].inter_merged != -1){
                v_source_t = (*g_base)[v_source_b].inter_merged;
            }
            if ((*g_base)[v_target_b].inter_merged != -1){
                v_target_t = (*g_base)[v_target_b].inter_merged;
            }

            bool corr_found = false;

            // both base verts have correspondence & edge also exists in target graph
            if (v_source_t != -1 && v_target_t != -1){
                auto et = edge(v_source_t, v_target_t, *g_target);
                if (et.second) {
                    (*g_target)[et.first].inter_base = std::make_pair(v_source_b, v_target_b);
                    (*g_base)[*eit].inter_target = std::make_pair(v_source_t, v_target_t);
                    corr_found = true;
                }
            }
            // edge should be deleted
            if (!corr_found) {
                (*g_base)[*eit].inter_delete_mark = true;
            }
        }

        // target edges: see if they were corresponded, add otherwise
        std::pair<EdgeIteratorGTGraph, EdgeIteratorGTGraph> edt = boost::edges(*g_target);
        for (EdgeIteratorGTGraph eit = edt.first; eit != edt.second; ++eit) {
            VertexDescriptorGTGraph vsor_t = boost::source(*eit, *g_target);
            VertexDescriptorGTGraph vtar_t = boost::target(*eit, *g_target);

            // edge did not correspond
            if (std::get<0>((*g_target)[*eit].inter_base) == -1){
                (*g_target)[*eit].inter_add_mark = true;
            }
        }
    }

    return true;
}



easy3d::vec3 GTree::get_closest_point_on_line(easy3d::vec3 point, easy3d::vec3 line_left, easy3d::vec3 line_right){
//    vec3 pt_a = point;
//    vec3 pt_b = line_right;
//    vec3 pt_c = line_left;
//         a
//        /
//    c---------b

    easy3d::vec3 p_res;

    easy3d::vec3 dir_line = (line_left - line_right).normalize();
    easy3d::vec3 line_ab = point - line_right;
    double dist_ab = dot(line_ab, dir_line);
    easy3d::vec3 a_proj_bc = line_right + (dist_ab * dir_line);

    // clamp to line ends
    double dist_line = length(line_right - line_left);
    double dist_left = length(a_proj_bc - line_left);
    double dist_right = length(line_right - a_proj_bc);

    // projected point is outside the line points
    if (dist_left > dist_line || dist_right > dist_line){
        // closer to right point
        if (dist_right < dist_left) {
            p_res = line_right;
        }
        // closer to left point
        else {
            p_res = line_left;
        }
    }
    // projected point is within the line points
    else {
        p_res = a_proj_bc;
    }

    return p_res;
}