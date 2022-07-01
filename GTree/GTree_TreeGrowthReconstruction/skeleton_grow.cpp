//
// Created by noort on 23/01/2022.
//

#include "skeleton_grow.h"


using namespace easy3d;
using FloatType = float;    // todo: remove if not using quickhull


/*-------------------------------------------------------------*/
/*---------------------------INIT------------------------------*/
/*-------------------------------------------------------------*/

GSkeleton::GSkeleton() : kd_points_(nullptr), kdtree_(nullptr), kd_points_corr_(nullptr), kdtree_corr_(nullptr) {
    pc_points_.clear();

    max_branching_level_ = 0;
    max_weight_ = 0;
    num_lobes_ = 0;
    max_dist_skel_ = 0;
}


GSkeleton::~GSkeleton() {
    if (kdtree_)
        delete kdtree_;
    if (kd_points_)
        delete kd_points_;
    if (pc_points_.size() > 0)
        pc_points_.clear();
    if (kdtree_corr_)
        delete kdtree_corr_;
    if (kd_points_corr_)
        delete kd_points_corr_;
}


/*-------------------------------------------------------------*/
/*-------------------------CONTROL-----------------------------*/
/*-------------------------------------------------------------*/

bool GSkeleton::reconstruct_skeleton(const PointCloud* cloud, SurfaceMesh* mesh_result) {
    std::cout << "\nSkeleton reconstruction:" << std::endl;
    std::cout << "\tloading points" << std::endl;

    if (!load_points(cloud)) {
        std::cerr << "ERROR: failed initial point registration step" << std::endl;
        return false;
    }

    std::cout << "\tbuilding delaunay" << std::endl;

    if (!build_delaunay()) {
        std::cerr << "ERROR: failed Delaunay triangulation step" << std::endl;
        return false;
    }

    std::cout << "\tbuilding mst" << std::endl;

    if (!build_mst()) {
        std::cerr << "ERROR: failed initial Minimum Spanning Tree step" << std::endl;
        return false;
    }

    std::cout << "\tsimplifying" << std::endl;

    if(!build_simplified()){
        std::cout << "ERROR: failed MST simplification step" << std::endl;
        return false;
    }


    std::cout << "Skeleton reconstruction complete." << std::endl;

    return true;
}


bool GSkeleton::reconstruct_branches(const PointCloud *cloud, SurfaceMesh* mesh_result, GraphGT* graph, VertexDescriptorGTGraph rootv){
    auto cylfit = new CylFit;
    cylfit->obtain_initial_radius(cloud);
    cylfit->construct_kd_tree(cloud);
    cylfit->set_skeleton(graph);
    cylfit->set_root(rootv);

    cylfit->reconstruct_branches(mesh_result);

    // todo: leaves

    return true;
}


/*-------------------------------------------------------------*/
/*--------------------BUILD STRUCTURES-------------------------*/
/*-------------------------------------------------------------*/

bool GSkeleton::load_points(const PointCloud *cloud){
    /// load point cloud points as vector of 3d vertices for easy access

    // check if cloud exists
    // todo: also add check for minimum number of points
    if (!cloud) {
        std::cout << "ERROR: point cloud does not exist" << std::endl;
        return false;
    }

    PointCloud::VertexProperty<vec3> points = cloud->get_vertex_property<vec3>("v:point");
    for (auto v : cloud->vertices()){
        vec3 pt_new = {points[v].x, points[v].y, points[v].z};
        pc_points_.push_back(pt_new);
    }
    return true;
}


bool GSkeleton::build_delaunay(){
    //initialize
    delaunay_.clear();

    if (pc_points_.empty()){
        std::cout << "ERROR: cannot build Delauney triangulation, no points were registered." << std::endl;
        return false;
    }

    int nr_points = pc_points_.size();

    // build Delauney triangulation graph
    // --vertices
    for (int i = 0; i < nr_points; i++) {
        SGraphVertexPropGT delaunay_pt_new;
        delaunay_pt_new.coords = vec3(pc_points_[i].x, pc_points_[i].y, pc_points_[i].z);
        boost::add_vertex(delaunay_pt_new, delaunay_);
    }

    // --edges
    tetgenio tet_in, tet_out;
    tet_in.numberofpoints = nr_points;
    tet_in.pointlist = new double[tet_in.numberofpoints * 3];
    int count = 0;
    for (auto pc_pt : pc_points_) {
        tet_in.pointlist[count * 3 + 0] = pc_pt.x;
        tet_in.pointlist[count * 3 + 1] = pc_pt.y;
        tet_in.pointlist[count * 3 + 2] = pc_pt.z;
        ++count;
    }

    // --triangulation
    tetgenbehavior tetgen_args_;
    tetgen_args_.parse_commandline((char *) ("Qn"));
    ::tetrahedralize(&tetgen_args_, &tet_in, &tet_out);

    for (long nTet = 0; nTet < tet_out.numberoftetrahedra; nTet++) {
        long tet_first = nTet * tet_out.numberofcorners;
        for (long i = tet_first; i < tet_first + tet_out.numberofcorners; i++) {
            for (long j = i + 1; j < tet_first + tet_out.numberofcorners; j++) {
                VertexDescriptorGTGraph p_source = vertex(tet_out.tetrahedronlist[i], delaunay_);
                VertexDescriptorGTGraph p_target = vertex(tet_out.tetrahedronlist[j], delaunay_);

                SGraphEdgePropGT edge_new_tetra;
                edge_new_tetra.length = delaunay_[p_target].coords.distance2(delaunay_[p_source].coords);

                add_edge(p_source, p_target, edge_new_tetra, delaunay_);
            }
        }
    }

    return true;
}


bool GSkeleton::build_mst() {
    // initialize
    mst_.clear();

    // copy vertices of delaunay into mst graph
    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(delaunay_);
    VertexDescriptorGTGraph rootv_curr = *(vt.first);
    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
        SGraphVertexPropGT vert_new;
        vert_new.coords = delaunay_[*vit].coords;
        // todo: initialize parent etc? so far unused
        add_vertex(vert_new, mst_);

        // find root vertex
        if (vert_new.coords.z < delaunay_[rootv_curr].coords.z){
            rootv_curr = *vit;
        }
    }
    // set root
    rootv_ = rootv_curr;

    // Dijkstra's Shortest Path
    std::vector<VertexDescriptorGTGraph> dijkstra_predecessor_res(num_vertices(delaunay_)); // shortest path edges
    std::vector<double> dijkstra_distance_res(num_vertices(delaunay_)); // distance vertices to root in weight
    dijkstra_shortest_paths(delaunay_,
                            rootv_,
                            weight_map(get(&SGraphEdgePropGT::length, delaunay_))
                            .distance_map(&dijkstra_distance_res[0])
                            .predecessor_map(&dijkstra_predecessor_res[0])
                            );

    // add shortest path edges into MST graph
    for (unsigned int seit = 0; seit < dijkstra_predecessor_res.size(); ++seit){
        unsigned int vert_curr = vertex(seit, mst_);
        unsigned int parent_curr = dijkstra_predecessor_res.at(seit);

        if (vert_curr != parent_curr){  // root check
            SGraphEdgePropGT edge_new_mst;
            EdgeDescriptorGTGraph edge_delaunay = edge(parent_curr,vert_curr,delaunay_).first;
            edge_new_mst.length = delaunay_[edge_delaunay].length;
            edge_new_mst.length_to_root = dijkstra_distance_res.at(seit);

            boost::add_edge(parent_curr, vert_curr, edge_new_mst, mst_);
        }
        // set parent of vertex
        mst_[vert_curr].parent = parent_curr;
    }

    if (!compute_branching_levels(mst_)){
        std::cout << "ERROR: cloud not compute branching levels" << std::endl;
        return false;
    }

    if (!compute_importance(mst_)){
        std::cout << "ERROR: could not compute vertex importance weight" << std::endl;
        return false;
    }

    return true;
}


bool GSkeleton::build_KdTree(GraphGT graph){
    // todo: multiple per skeleton? For now just input graph

    // clear previous data
    if (kdtree_)
        delete kdtree_;
    if (kd_points_)
        delete kd_points_;

    // initialize Kd-tree
    int nr_points = num_vertices(graph);
    kd_points_ = new Vector3D[nr_points];
    // todo: with deleted vertices, not very efficient... but somehow filtering on delete_mark breaks stuff

    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(graph);
    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
        kd_points_[*vit].x = graph[*vit].coords.x;
        kd_points_[*vit].y = graph[*vit].coords.y;
        kd_points_[*vit].z = graph[*vit].coords.z;
    }
    // todo: if tree is changed a lot later on, an R-tree would be better here
    // but the boost one has no distance query, only nearest neighbours
    kdtree_ = new KdTree(kd_points_, nr_points, 16);

    if ((!kdtree_) || (!kd_points_) ){
        std::cout << "ERROR: kd-tree reconstruction failed" << std::endl;
        return false;
    }

    return true;
}


bool GSkeleton::build_simplified(){
    simplified_.clear();

    // initialize with MST
    simplified_ = mst_;

    // line simplification on all edges
    std::vector<VertexDescriptorGTGraph> start_branch;
    start_branch.push_back(rootv_);
    simplify_branch(simplified_, start_branch);

    // remove (clear) vertices that were selected for cleaning
    clean_graph(simplified_);

    return true;
}


bool GSkeleton::build_corresponding(GraphGT graph_corr, VertexDescriptorGTGraph root_vert){
    // lot of code in 1 method, is not in separate methods because that's about 3x as slow

    if (num_vertices(simplified_) <= 1){
        std::cout << "ERROR computing corresponding main skeleton, simplified skeleton does not exist" << std::endl;
        return false;
    }
    // initialize
    corresponding_.clear();
    corresponding_ = simplified_;   // copy
    ts_main_ = simplified_; // copy also to timestamp-specific (so not constrained with average) for later use
    rootv_corr_ = rootv_; // save root of delaunay/mst/simplified/tsmain skeletons (only corresponding is different)

    // ensure kdtree is set to simplified
    build_KdTree(corresponding_);

    //--- flag all vertices close to any of the edges of main skeleton
    float dist_bounds;                   // distance within which corresponding points could be found
    float dist_tolerance_lower = 0.2;    // distance which is considered close enough to edge to belong to it
    float dist_tolerance_upper = 0.5;    // distance which is considered close enough to edge to belong to it, if direction is comparable
    float angle_tolerance = 30;          // maximum angle between point normal and edge direction for dir to be comparable

    GraphGT delaunay_constrained = GraphGT(); // for putting vertices in already when found for efficiency
    std::map<VertexDescriptorGTGraph, VertexDescriptorGTGraph> indices_corr;    // verts graph_corr --> delaunay_constrained
    std::map<VertexDescriptorGTGraph, VertexDescriptorGTGraph> indices_dela;    // verts  delaunay_constrained --> graph_corr
    int nr_main_edges = 0;

    // process new root
    SGraphVertexPropGT v_root;
    v_root.coords = graph_corr[root_vert].coords;
    v_root.is_main = true;
    add_vertex(v_root, delaunay_constrained);
    indices_corr[root_vert] = 0;
    indices_dela[0] = root_vert;

    // todo: remove all ts vertices below the bifurcation point?

    // find corresponding ts vertices
    std::pair<EdgeIteratorGTGraph, EdgeIteratorGTGraph> et = edges(graph_corr);
    for (EdgeIteratorGTGraph eit = et.first; eit != et.second; ++eit){
        // only use corresponding skeleton that was previously flagged as main
        if (graph_corr[*eit].is_main) {
            VertexDescriptorGTGraph v_source = source(*eit, graph_corr);
            VertexDescriptorGTGraph v_target = target(*eit, graph_corr);

            // store main edges and vertices for later
            SGraphVertexPropGT vnew;
            vnew.coords = graph_corr[v_target].coords;
            vnew.is_main = true;    // flag to find constraining edges later
            vnew.is_lobe_node_corr = graph_corr[v_target].is_lobe_node;
            indices_corr[v_target] = num_vertices(delaunay_constrained);
            indices_dela[num_vertices(delaunay_constrained)] = v_target;
            add_vertex(vnew, delaunay_constrained);

            nr_main_edges++;

            // kd-tree range query to get eligible neighbours
            std::vector<VertexDescriptorGTGraph> line;
            line.push_back(v_source);
            line.push_back(v_target);

            dist_bounds = (graph_corr[*eit].length / 2.0) + dist_tolerance_upper;

            for (auto v_line: line) {
                Vector3D p_line = {graph_corr[v_line].coords.x,
                                   graph_corr[v_line].coords.y,
                                   graph_corr[v_line].coords.z};
                kdtree_->queryRange(p_line, dist_bounds, true);
                double nr_nbors = kdtree_->getNOfFoundNeighbours();
                if (nr_nbors > 0) {
                    for (int i = 0; i < nr_nbors; ++i) {
                        unsigned int hit_idx = kdtree_->getNeighbourPositionIndex(i);
                        // check if not deleted or already main (no need to check twice)
                        if ((!corresponding_[hit_idx].delete_mark) && (!corresponding_[hit_idx].is_main)) {
                            double dist_to_edge = distance_point_to_line(corresponding_[hit_idx].coords,
                                                                           graph_corr[v_source].coords,
                                                                           graph_corr[v_target].coords);

                            // vertex is within lower distance bound: close enough to edge to be main
                            if (dist_to_edge <= dist_tolerance_lower){
                                corresponding_[hit_idx].is_main = true;
                            }
                            // vertex is within upper distance bounds: if normal & edge direction correspond, is main
                            else if (dist_to_edge <= dist_tolerance_upper){
                                vec3 edge_dir = graph_corr[v_target].coords - graph_corr[v_source].coords;
                                double angle = acos(dot(edge_dir, corresponding_[hit_idx].normal) /
                                        (length(edge_dir) * length(corresponding_[hit_idx].normal)));
                                angle = angle * 180 / M_PI; // to degrees

                                if (angle <= angle_tolerance){
                                    corresponding_[hit_idx].is_main = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    //--- constrained Delaunay triangulation

    // add all non-main and non-deleted ts vertices to new delaunay graph
    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(corresponding_);
    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
        if (!(corresponding_[*vit].is_main || corresponding_[*vit].delete_mark)){
            SGraphVertexPropGT vnew_m;
            vnew_m.coords = corresponding_[*vit].coords;
            add_vertex(vnew_m, delaunay_constrained);
        }
        // add to timestamp main as well
        // has to be intermediary, because corresponding_ will be overwritten at the end of this function
        if (corresponding_[*vit].is_main && (!corresponding_[*vit].delete_mark)){   // && degree(*vit, simplified_) > 1
            ts_main_[*vit].is_main = true;
        }
    }

    // delaunay edges
    int nr_points = num_vertices(delaunay_constrained);
    tetgenio tet_in, tet_out;
    tet_in.numberofpoints = nr_points;
    tet_in.pointlist = new double[tet_in.numberofpoints * 3];
    tet_in.numberofedges = nr_main_edges;
    tet_in.edgelist = new int[tet_in.numberofedges * 2];

    int count = 0;
    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vtc = vertices(delaunay_constrained);
    for (VertexIteratorGTGraph vit = vtc.first; vit != vtc.second; ++vit){
        tet_in.pointlist[count * 3 + 0] = delaunay_constrained[*vit].coords.x;
        tet_in.pointlist[count * 3 + 1] = delaunay_constrained[*vit].coords.y;
        tet_in.pointlist[count * 3 + 2] = delaunay_constrained[*vit].coords.z;

        ++count;
    }

    // enforce edges
    int count_edge = 0;
    for (EdgeIteratorGTGraph eit = et.first; eit != et.second; ++eit){
        if (graph_corr[*eit].is_main) {
            VertexDescriptorGTGraph v_source = source(*eit, graph_corr);
            VertexDescriptorGTGraph v_target = target(*eit, graph_corr);

            tet_in.edgelist[count_edge * 2 + 0] = indices_corr[v_source];
            tet_in.edgelist[count_edge * 2 + 1] = indices_corr[v_target];

            count_edge++;
        }
    }

    int main_ct_2 = 0;
    // delaunay triangulation
    // p = input PLC,
    // Y = keep original input lines,
    // c = convex (will delete all tetrahedra as outer if not set),
    // Q = quiet
    tetgenbehavior tetgen_args_;
    tetgen_args_.parse_commandline((char *) ("pYcQ"));
    ::tetrahedralize(&tetgen_args_, &tet_in, &tet_out);

    for (long nTet = 0; nTet < tet_out.numberoftetrahedra; nTet++) {
        long tet_first = nTet * tet_out.numberofcorners;
        for (long i = tet_first; i < tet_first + tet_out.numberofcorners; i++) {
            for (long j = i + 1; j < tet_first + tet_out.numberofcorners; j++) {
                VertexDescriptorGTGraph p_source = vertex(tet_out.tetrahedronlist[i], delaunay_constrained);
                VertexDescriptorGTGraph p_target = vertex(tet_out.tetrahedronlist[j], delaunay_constrained);

                SGraphEdgePropGT edge_new_tetra;
                edge_new_tetra.length = delaunay_constrained[p_target].coords.distance2(delaunay_constrained[p_source].coords);

                if (delaunay_constrained[p_source].is_main && delaunay_constrained[p_target].is_main){
                    auto edge_orig_corr = edge(indices_dela[p_source], indices_dela[p_target], graph_corr);
                    // if edge exists
                    if (edge_orig_corr.second){
                        // if original edge is also main
                        if (graph_corr[edge_orig_corr.first].is_main){
                            // set weight to 0 and flag as main as well
                            edge_new_tetra.is_main = true;
                            edge_new_tetra.length = 0.0;    // used as weight for MST
                            main_ct_2++;
                        }
                    }
                }

                add_edge(p_source, p_target, edge_new_tetra, delaunay_constrained);
            }
        }
    }

    //--- make new MST

    GraphGT mst_constrained = GraphGT();

    // copy vertices of delaunay into mst graph
    VertexDescriptorGTGraph rootv_curr = *(vtc.first);
    for (VertexIteratorGTGraph vit = vtc.first; vit != vtc.second; ++vit) {
        SGraphVertexPropGT vert_new;
        vert_new.coords = delaunay_constrained[*vit].coords;
        vert_new.is_main = delaunay_constrained[*vit].is_main;
        vert_new.is_lobe_node_corr = delaunay_constrained[*vit].is_lobe_node_corr;
        add_vertex(vert_new, mst_constrained);

        // find root vertex
        if (vert_new.coords.z < delaunay_constrained[rootv_curr].coords.z){
            rootv_curr = *vit;
        }
    }

    // Dijkstra's Shortest Path
    std::vector<VertexDescriptorGTGraph> dijkstra_predecessor_res(num_vertices(delaunay_constrained)); // shortest path edges
    std::vector<double> dijkstra_distance_res(num_vertices(delaunay_constrained)); // distance vertices to root in weight
    dijkstra_shortest_paths(delaunay_constrained,
                            rootv_curr,
                            weight_map(get(&SGraphEdgePropGT::length, delaunay_constrained))
                                    .distance_map(&dijkstra_distance_res[0])
                                    .predecessor_map(&dijkstra_predecessor_res[0])
    );

    // add shortest path edges into MST graph
    for (unsigned int seit = 0; seit < dijkstra_predecessor_res.size(); ++seit){
        unsigned int vert_curr = vertex(seit, mst_constrained);
        unsigned int parent_curr = dijkstra_predecessor_res.at(seit);

        if (vert_curr != parent_curr){  // root check
            SGraphEdgePropGT edge_new_mst;
            EdgeDescriptorGTGraph edge_delaunay = edge(parent_curr,vert_curr,delaunay_constrained).first;
            edge_new_mst.length = delaunay_constrained[edge_delaunay].length;
            edge_new_mst.is_main = delaunay_constrained[edge_delaunay].is_main;
            edge_new_mst.length_to_root = dijkstra_distance_res.at(seit);

            boost::add_edge(parent_curr, vert_curr, edge_new_mst, mst_constrained);
        }
        // set parent of vertex
        mst_constrained[vert_curr].parent = parent_curr;
    }

    // todo: add non-corresponding main branches to ts main
    // todo: check structural info of skeleton

    //--- find lobe connection points

    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt_mst = vertices(mst_constrained);
    for (VertexIteratorGTGraph vit = vt_mst.first; vit != vt_mst.second; ++vit){
        if (!(mst_constrained[*vit].delete_mark || *vit == rootv_curr)) {
            EdgeDescriptorGTGraph e_to_parent = edge(mst_constrained[*vit].parent, *vit, mst_constrained).first;
            // node is part of main skeleton
            if (mst_constrained[e_to_parent].is_main) {
                // todo: detect correspondence in lobe nodes
                // todo: detect unique node points (if possible use already existing method)

                // count non-main children
                std::vector<VertexDescriptorGTGraph> nonmain_children;
                for (VertexDescriptorGTGraph v_next: make_iterator_range(adjacent_vertices(*vit, mst_constrained))) {
                    if (v_next != mst_constrained[*vit].parent) {
                        EdgeDescriptorGTGraph e_next = edge(*vit, v_next, mst_constrained).first;
                        if (!mst_constrained[e_next].is_main) {
                            nonmain_children.push_back(v_next);
                        }
                    }
                }

                int min_lobe_size = 5;  // todo: parameter access

                if (!nonmain_children.empty()) {

                    // count child nodes
                    int lobe_size_curr = 0;
                    while (!nonmain_children.empty()) {
                        VertexDescriptorGTGraph child_curr = nonmain_children.back();
                        nonmain_children.pop_back();
                        lobe_size_curr++;

                        // check if min lobe size is reached
                        if (lobe_size_curr >= min_lobe_size) {
                            nonmain_children.clear();
                            mst_constrained[*vit].is_lobe_node = true;
                            break;
                        } else {
                            // process current children
                            for (VertexDescriptorGTGraph child_next: make_iterator_range(adjacent_vertices(child_curr, mst_constrained))) {
                                if (child_next != mst_constrained[child_curr].parent) {
                                    nonmain_children.push_back(child_next);
                                }
                            }
                        }
                    }
                    // if nonmain_children is empty and min lobe size was not reached, vertex will remain not a lobe node
                }

                // mark all lobe nodes additionally detected for this timestamp (not in main correspondence skeleton)
                if (mst_constrained[*vit].is_lobe_node && (!mst_constrained[*vit].is_lobe_node_corr)) {
                    mst_constrained[*vit].is_lobe_node_ts = true;
                }
            }
        }
    }

    corresponding_ = mst_constrained;
    rootv_corr_ = rootv_curr;

    return true;
}


void GSkeleton::build_ts_main(GraphGT graph_corr){
    //-- build base main

    // obtain all edges between 2 vertices flagged as main
    // (are flagged during adding all non-main edges to new constrained Delaunay)
    std::vector<EdgeDescriptorGTGraph> main_detected;
    std::pair<EdgeIteratorGTGraph, EdgeIteratorGTGraph> et = edges(ts_main_);
    for (EdgeIteratorGTGraph eit = et.first; eit != et.second; ++eit){
        // edge is connected to root
        if ((target(*eit, ts_main_) == rootv_) ||
            (source(*eit, ts_main_) == rootv_)){
            ts_main_[*eit].is_main = true;
            main_detected.push_back(*eit);
        } else if (ts_main_[target(*eit, ts_main_)].is_main && ts_main_[source(*eit, ts_main_)].is_main){
            // leaves cannot be main
            if (!(degree(target(*eit, ts_main_), ts_main_) <= 1 || degree(source(*eit, ts_main_), ts_main_) <= 1)) {
                ts_main_[*eit].is_main = true;
                main_detected.push_back(*eit);
            }
        }
    }

    // connect all edges flagged as main into single skeleton
    while (!main_detected.empty()){
        // pop current edge from found main edge stack
        EdgeDescriptorGTGraph e_curr = main_detected.back();
        main_detected.pop_back();
        // find first vertex of edge
        VertexDescriptorGTGraph v_start = source(e_curr, ts_main_);
        if (ts_main_[v_start].parent == target(e_curr, ts_main_)){
            v_start = target(e_curr, ts_main_);
        }

        // traverse back to root, set all as main
        VertexDescriptorGTGraph v_parent = ts_main_[v_start].parent;
        while (v_parent != rootv_){
            EdgeDescriptorGTGraph e_parent = edge(v_start, v_parent, ts_main_).first;
            ts_main_[e_parent].is_main = true;
            ts_main_[source(e_parent, ts_main_)].is_main = true;
            ts_main_[target(e_parent, ts_main_)].is_main = true;
            v_start = v_parent;
            v_parent = ts_main_[v_start].parent;
        }
    }

    //-- simplify
    // todo: make all ts equal-ish for better edit distance? (enforce close to merged main)

    // remove all non-main edges from graph
    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(ts_main_);
    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
        if (!(ts_main_[*vit].delete_mark || *vit == rootv_)){
            EdgeDescriptorGTGraph e_to_parent = edge(ts_main_[*vit].parent, *vit, ts_main_).first;
            if (!ts_main_[e_to_parent].is_main){
                ts_main_[*vit].clear_mark = true;
            }
        }
    }

    // remove marked (non-main) vertices
    clean_graph(ts_main_);

    // simplify entire main skeleton
    std::vector<VertexDescriptorGTGraph> start_main;
    start_main.push_back(rootv_);
    simplify_branch(ts_main_, start_main, false, 0.05, 0.25);
    clean_graph(ts_main_);

    //-- compute distances to merged main
    intermediary_skeleton_distance(graph_corr);
}


void GSkeleton::intermediary_skeleton_distance(GraphGT graph_corr){
    //-- compute minimum distance to merged main skeleton

    // make Kd-tree of midpoints of merged main edges
    int nr_points = num_edges(graph_corr);  // non-main edges have been removed
    auto kd_pts = new Vector3D[nr_points];
    std::map<unsigned int, std::pair<EdgeDescriptorGTGraph, Point_3_cgal> > midpt_map;
    // kd tree index (midpoints) <--> graph_corr edge descriptor + mid point coordinates

    unsigned int idx_curr = 0;
    std::pair<EdgeIteratorGTGraph, EdgeIteratorGTGraph> et_corr = edges(graph_corr);
    for (EdgeIteratorGTGraph eit = et_corr.first; eit != et_corr.second; ++eit){
        VertexDescriptorGTGraph v_source = source(*eit, graph_corr);
        VertexDescriptorGTGraph v_target = target(*eit, graph_corr);
        Point_3_cgal p_source = {graph_corr[v_source].coords.x,
                                 graph_corr[v_source].coords.y,
                                 graph_corr[v_source].coords.z};
        Point_3_cgal p_target = {graph_corr[v_target].coords.x,
                                 graph_corr[v_target].coords.y,
                                 graph_corr[v_target].coords.z};

        Point_3_cgal p_mid = CGAL::midpoint(p_source, p_target);
        midpt_map[idx_curr] = std::make_pair(*eit, p_mid);

        kd_pts[idx_curr].x = p_mid.x();
        kd_pts[idx_curr].y = p_mid.y();
        kd_pts[idx_curr].z = p_mid.z();

        idx_curr++;
    }

    auto kd_tree_corr = new KdTree(kd_pts, nr_points, 16);

    // find closest midpoint for each main edge of timestamp main skeleton
    std::pair<EdgeIteratorGTGraph, EdgeIteratorGTGraph> et = edges(ts_main_);
    for (EdgeIteratorGTGraph eit = et.first; eit != et.second; ++eit){
        if (ts_main_[*eit].is_main) {
            VertexDescriptorGTGraph v_source = source(*eit, ts_main_);
            VertexDescriptorGTGraph v_target = target(*eit, ts_main_);
            Point_3_cgal pe_ts_source = {ts_main_[v_source].coords.x,
                                         ts_main_[v_source].coords.y,
                                         ts_main_[v_source].coords.z};
            Point_3_cgal pe_ts_target = {ts_main_[v_target].coords.x,
                                         ts_main_[v_target].coords.y,
                                         ts_main_[v_target].coords.z};
            Point_3_cgal p_mid_ts = CGAL::midpoint(pe_ts_source, pe_ts_target);
            Vector3D p_mid_ts_query = {static_cast<float>(p_mid_ts.x()),
                                       static_cast<float>(p_mid_ts.y()),
                                       static_cast<float>(p_mid_ts.z())};

            kd_tree_corr->setNOfNeighbours(10);  // only get the closest few neighbours for speed reasons
            kd_tree_corr->queryPosition(p_mid_ts_query);
            int nr_hits = kd_tree_corr->getNOfFoundNeighbours(); // simply query one of the line points
            double dist_lines = 999;

            for (int i = 0; i < nr_hits; ++i){
                unsigned int hit_idx = kd_tree_corr->getNeighbourPositionIndex(i);  // get index of closest neighbour
                EdgeDescriptorGTGraph e_hit = midpt_map[hit_idx].first;
                Point_3_cgal p_hit = midpt_map[hit_idx].second;

                // find distance between the two lines with CGAL
                VertexDescriptorGTGraph v_hit_source = source(e_hit, graph_corr);
                VertexDescriptorGTGraph v_hit_target = target(e_hit, graph_corr);
                Point_3_cgal pe_corr_source = {graph_corr[v_hit_source].coords.x,
                                               graph_corr[v_hit_source].coords.y,
                                               graph_corr[v_hit_source].coords.z};
                Point_3_cgal pe_corr_target = {graph_corr[v_hit_target].coords.x,
                                               graph_corr[v_hit_target].coords.y,
                                               graph_corr[v_hit_target].coords.z};

                Line_3_cgal l_corr(pe_corr_source, pe_corr_target);
                Line_3_cgal l_ts(pe_ts_source, pe_ts_target);

                double dist_curr = sqrt(CGAL::squared_distance(l_ts, l_corr));
                // todo: incorporate angle similarity (weighted formula) + ?? (length?)
                if (i == 0){
                    dist_lines = dist_curr;
                } else {
                    if (dist_curr < dist_lines) {
                        dist_lines = dist_curr;
                    }
                }
            }

            ts_main_[*eit].distance = dist_lines;
            if (dist_lines > max_dist_skel_) {
                max_dist_skel_ = dist_lines;
            }
        }
    }
}


bool GSkeleton::detect_lobe_points(std::vector<Lobe*>& lobes){
    // check if corresponding has vertices
    if (num_vertices(corresponding_) <= 1){
        std::cout << "ERROR finding lobe points, corresponding skeleton does not have points" << std::endl;
        return false;
    }

    // keep track of number of nodes/node index
    int lobe_idx_curr = 0;

    // assign lobe indices
    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(corresponding_);
    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
        // filter out deleted vertices and the root
        if (!(corresponding_[*vit].delete_mark || *vit == rootv_corr_)){
            // if connector point, visit and flag all children
            if (corresponding_[*vit].is_lobe_node){
                // set index
                corresponding_[*vit].lobe_index = lobe_idx_curr;

                // make lobe
                Lobe* lobe_curr = new Lobe();
                lobe_curr->set_index(lobe_idx_curr);

                // add connector node to subgraph of lobe
                GraphGT* graph_lobe = const_cast<GraphGT *>(&(lobe_curr->get_subgraph()));
                SGraphVertexPropGT v_new;
                v_new.coords = corresponding_[*vit].coords;
                v_new.is_lobe_node = true;
                v_new.lobe_index = lobe_idx_curr;
                v_new.parent = *vit;    // set parent to index in correspondence skeleton
                VertexDescriptorGTGraph v_new_added = add_vertex(v_new, *graph_lobe);   // index always 0

                // store root node of lobe subgraph
                lobe_curr->set_connector_node(*vit);

                // detect children (all should be non_main)
                std::vector<std::pair<VertexDescriptorGTGraph, VertexDescriptorGTGraph>> children;  // skeleton <> lobe idx
                children.emplace_back(*vit, v_new_added);
                while (!children.empty()){
                    VertexDescriptorGTGraph v_curr = children.back().first;
                    VertexDescriptorGTGraph v_curr_lobe = children.back().second;
                    children.pop_back();

                    for (VertexDescriptorGTGraph v_child : make_iterator_range(adjacent_vertices(v_curr, corresponding_))){
                        if (v_child != corresponding_[v_curr].parent && !corresponding_[v_child].is_main) {
                            // store index/flag as lobe point in graph
                            corresponding_[v_child].lobe_index = lobe_idx_curr;

                            // add vertex to lobe
                            SGraphVertexPropGT v_new_child;
                            v_new_child.coords = corresponding_[v_child].coords;
                            v_new_child.lobe_index = lobe_idx_curr;
                            v_new_child.parent = v_curr_lobe;
                            VertexDescriptorGTGraph v_child_added = add_vertex(v_new_child, *graph_lobe);

                            // add edge to parent to lobe
                            EdgeDescriptorGTGraph e_to_parent = edge(v_child, corresponding_[v_child].parent, corresponding_).first;
                            SGraphEdgePropGT e_new;
                            e_new.length = corresponding_[e_to_parent].length;
                            add_edge(v_child_added, v_curr_lobe, e_new, *graph_lobe);

                            children.emplace_back(v_child, v_child_added);
                        }
                    }
                }

                // add lobe to set storing them
                lobes.push_back(lobe_curr);

                // increase/update current node index
                lobe_idx_curr++;
            }
        }
    }

    std::cout << "\t---number of lobes found: " << lobe_idx_curr << std::endl;

    // store number of lobes found
    num_lobes_ = lobe_idx_curr;

    return true;
}


bool GSkeleton::build_corresponding_merged() {
    // make kd-tree with all original points
    build_KdTree(mst_);

    // find trunk base center
    float avg_x = 0.0;
    float avg_y = 0.0;

    float dist_tolerance = 0.5; // todo: perhaps nr nbors better?
    Vector3D p_root = {corresponding_[rootv_].coords.x,
                       corresponding_[rootv_].coords.y,
                       corresponding_[rootv_].coords.z};
    kdtree_->queryRange(p_root, dist_tolerance, true);
    double nr_nbors = kdtree_->getNOfFoundNeighbours();
    if (nr_nbors > 0) {
        for (int i = 0; i < nr_nbors; ++i) {
            unsigned int hit_idx = kdtree_->getNeighbourPositionIndex(i);
            avg_x += mst_[hit_idx].coords.x;
            avg_y += mst_[hit_idx].coords.y;

        }
        avg_x = avg_x / nr_nbors;
        avg_y = avg_y / nr_nbors;
    }

    // set new root point as centroid of lowest points, at lowest height
    vec3 new_rootp = {avg_x, avg_y, mst_[rootv_].coords.z};
    corresponding_[rootv_].coords = new_rootp;

    // find main bifurcation point
    VertexDescriptorGTGraph v_bifur = find_main_bifurcation();
    corresponding_[v_bifur].is_main = true;

    // store all points on the line between rootv_ and v_bifur
    std::vector<VertexDescriptorGTGraph> trunk_verts;
    trunk_verts.push_back(v_bifur);
    VertexDescriptorGTGraph v_curr = v_bifur;
    while (v_curr != rootv_){
        trunk_verts.push_back(corresponding_[v_curr].parent);
        v_curr = corresponding_[v_curr].parent;
    }

    // remove non-main edges from trunk
    std::pair<VertexIteratorGTGraph , VertexIteratorGTGraph> vt = vertices(corresponding_);
    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
        // check all points lower than the bifur point
        if (!corresponding_[*vit].delete_mark && (corresponding_[*vit].coords.z < corresponding_[v_bifur].coords.z)){
            // check if point on trunk line, if not mark for deletion
            if (std::find(trunk_verts.begin(), trunk_verts.end(), *vit) == trunk_verts.end()){
                corresponding_[*vit].clear_mark = true;
            }
            // todo: if a branch "dips" below bifur z, this will cut all vertices from it that are too low, could result in unrealistic geometry
        }
    }

    // find/alter main skeleton for lobe nodes
    if (!find_all_lobe_nodes(corresponding_)){
        std::cout << "ERROR: could not find bifurcation points in main merged correspondence skeleton" << std::endl;
        return false;
    }

    // remove all non-main edges from graph
    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
        if (!(corresponding_[*vit].delete_mark || *vit == rootv_)){
            EdgeDescriptorGTGraph e_to_parent = edge(corresponding_[*vit].parent, *vit, corresponding_).first;
            if (!corresponding_[e_to_parent].is_main){
                corresponding_[*vit].clear_mark = true;
            }
        }
    }

    // remove marked vertices (so far only from trunk simplification)
    clean_graph(corresponding_);
    // simplify trunk between rootv_ and v_bifur
    simplify_line(corresponding_, trunk_verts);

    // simplify entire main skeleton
    // non-deletion of lobe nodes is hardcoded in simplification methods
    std::vector<VertexDescriptorGTGraph> start_main;
    start_main.push_back(rootv_);
    simplify_branch(corresponding_, start_main);
    clean_graph(corresponding_);

    // todo: detect ts points along line instead of around end points

    // todo: simplify main branches in a smarter way (use "normal" information?)?

    std::cout << "\t-- done making merged main correspondence skeleton" << std::endl;
    return true;
}


/*-------------------------------------------------------------*/
/*--------------COMPUTE STRUCTURAL INFORMATION-----------------*/
/*-------------------------------------------------------------*/

bool GSkeleton::compute_branching_levels(GraphGT& graph){
    std::vector<VertexDescriptorGTGraph> traverse_list;    // stack of nodes to still traverse

    // check if not only root
    if (boost::degree(rootv_, graph) < 1){
        return false;
    }

    // process root: set all child edges & vertices to level 0
    graph[rootv_].branch_level = 0;
    for (VertexDescriptorGTGraph child_v : make_iterator_range(adjacent_vertices(rootv_, graph))){
        if (child_v != rootv_) {
            graph[child_v].branch_level = 0;
            graph[edge(rootv_, child_v, graph).first].branch_level = 0;
            traverse_list.push_back(child_v);
        }
    }

    while (!traverse_list.empty()){
        VertexDescriptorGTGraph node_curr = traverse_list[0];
        traverse_list.erase(traverse_list.begin());

        // todo: leaf check needed?

        // straight segment
        if (degree(node_curr, graph) == 2){
            // - set degree
            // - add to list
            // - continue to next node in list
            for (VertexDescriptorGTGraph child_v : make_iterator_range(adjacent_vertices(node_curr, graph))) {
                if (child_v != graph[node_curr].parent){
                    graph[child_v].branch_level = graph[node_curr].branch_level;
                    graph[edge(node_curr, child_v, graph).first].branch_level = graph[node_curr].branch_level;
                    traverse_list.push_back(child_v);
                }
            }
        }

        // branching point
        else if (degree(node_curr, graph) > 1){
            for (VertexDescriptorGTGraph child_v : make_iterator_range(adjacent_vertices(node_curr, graph))) {
                if (child_v != graph[node_curr].parent) {
                    graph[child_v].branch_level = graph[node_curr].branch_level + 1;
                    graph[edge(node_curr, child_v, graph).first].branch_level = graph[node_curr].branch_level + 1;
                    traverse_list.push_back(child_v);

                    if ((graph[node_curr].branch_level + 1) > max_branching_level_){
                        max_branching_level_ = graph[node_curr].branch_level + 1;
                    }
                }
            }
        }
    }

    return true;
}


bool GSkeleton::compute_importance(GraphGT& graph){
    build_KdTree(graph);

    if (!compute_normals_pointcloud(graph)){
        std::cout << "ERROR computing vertex importance: could not find vertex normals with PCA" << std::endl;
        return false;
    }

    // neighbour processing per vertex
    // todo: adapting threshold (ahn2/3/4? density determination?)
    double nborhood_distance = 0.5;  // maximum distance to neighbours // todo: is this distance squared??
    double nborhood_angle = 45;   // maximum angle between normals of neighbours [degrees]

    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(graph);
    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
        // get closest neighbours
        Vector3D p_curr = {graph[*vit].coords.x, graph[*vit].coords.y, graph[*vit].coords.z};
        kdtree_->queryRange(p_curr, nborhood_distance, true);
        double nr_nbors = kdtree_->getNOfFoundNeighbours();

        if (nr_nbors == 1){ // no neighbours that are close enough were found
            graph[*vit].weight = 0;
        } else {
            int nborhood_count = 0; // nr of counted similar neighbours
            for (int l = 1; l < nr_nbors; ++l) {
                unsigned int nbor_idx = kdtree_->getNeighbourPositionIndex(l);
                // ensure only non-deleted vertices are counted as neighbours
                if (!graph[nbor_idx].delete_mark) {
                    vec3 nbor_coords = graph[nbor_idx].coords;

                    double angle = acos(dot(graph[*vit].normal, graph[nbor_idx].normal) /
                                        (length(graph[*vit].normal) * length(graph[nbor_idx].normal)));
                    angle = angle * 180 / M_PI; // to degrees

                    if (angle < nborhood_angle) {
                        nborhood_count += 1;
                    }
                }
            }
            graph[*vit].weight = nborhood_count;
            if (nborhood_count > max_weight_){
                max_weight_ = nborhood_count;
            }
        }
    }

    return true;
}


bool GSkeleton::compute_normals_pointcloud(GraphGT& graph){
    // todo: check if (points in) kd tree exist
    // todo: adapting threshold (ahn2/3/4? density determination?)
    // todo: include original point in pca?
    // todo: process points with < 5 nbors better

    // todo: rename, these are the opposite of normals really

    double threshold = 0.2; // 0.1 = 10cm

    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(graph);
    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
        // find neighbours with kd-tree
        Vector3D p_curr = {graph[*vit].coords.x, graph[*vit].coords.y, graph[*vit].coords.z};
        kdtree_->queryRange(p_curr, threshold, true);
        double nr_nbors = kdtree_->getNOfFoundNeighbours();

        // PCA
        easy3d::PrincipalAxes<3, float> pca;
        pca.begin();
        // if not enough neighbours were found, find the 5 nearest instead
        // (pca needs at least 4 points to work, 0 = same point)
        if (nr_nbors < 5) {
            kdtree_->setNOfNeighbours(5);
            kdtree_->queryPosition(p_curr);
            nr_nbors = 5;
        }
        // compute eigenvectors
        for (int l = 1; l < nr_nbors; ++l) {
            unsigned int nbor_idx = kdtree_->getNeighbourPositionIndex(l);
            vec3 nbor_coords = graph[nbor_idx].coords;

            pca.add(nbor_coords);
        }
        pca.end();

        graph[*vit].normal = pca.axis(0);
        // (can't store pca as attribute, causes heap error)
//        graph[*vit].pca = pca;   // store all axes & eigen values for future functionality
    }

    return true;
}


bool GSkeleton::find_all_lobe_nodes(GraphGT& graph){
    // check if graph exists
    if (num_edges(graph) == 0) {
        std::cerr << "ERROR: could not find bifurcation points, graph is empty" << std::endl;
        return false;
    }

    // detect lobe nodes:
    // - go over all main vertices
    // - if main end:
    //      - see if lobe is big enough
    //      - if not, walk back until it is, de-main edges along the way
    //      - only delete small/insignificant main edges, set node at bifur point if long main parent branch found
    // - if non-main connected but not main end:
    //      - see if lobe is big enough
    // - if leaf:
    //      - walk back until lobe is big enough or qualifying parent is found

    // todo: perhaps implement main tip constraints? (eg should be long enough/far away from other bifur?)
    // todo: check if parent of lobe node is also lobe node + similarity constraints

    // parameters
    double min_lobe_size = 5;   // minimum [nr points] inside an accepted lobe

    std::pair<VertexIteratorGTGraph , VertexIteratorGTGraph> vt = vertices(graph);
    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
        if (!(graph[*vit].delete_mark || *vit == rootv_ || graph[*vit].clear_mark)) {   // so far clear mark only for trunk, so safe to use
            EdgeDescriptorGTGraph e_to_parent = edge(graph[*vit].parent, *vit, graph).first;
            // node is part of main skeleton
            if (graph[e_to_parent].is_main) {
                // processing of main edge leading to leaf node not needed,
                // leaves already filtered out during main correspondence edge detection (in g_tree)

                // todo: perhaps make distance to leaf measure as structural info (like in LGrow) and use it too?

                // count main and non-main children
                std::vector<VertexDescriptorGTGraph> main_children;
                std::vector<VertexDescriptorGTGraph> nonmain_children;

                for (VertexDescriptorGTGraph v_next: make_iterator_range(adjacent_vertices(*vit, graph))) {
                    if (v_next != graph[*vit].parent) {
                        EdgeDescriptorGTGraph e_next = edge(*vit, v_next, graph).first;
                        if (graph[e_next].is_main) {
                            main_children.push_back(v_next);
                        } else {
                            nonmain_children.push_back(v_next);
                        }
                    }
                }

                // cases that can be bifur:
                // - no main children, lobe is big enough
                // - main children with also non-main children, non-main children are big enough
                // (- only main children: continue, not lobe node)

                // it will be difficult to sprout more than 1 lobe from 1 node, not done

                // node has 1+ main children
                if (!main_children.empty()){
                    // main segment with non-main children
                    if (!nonmain_children.empty()){

                        // count child nodes
                        int lobe_size_curr = 0;
                        while (!nonmain_children.empty()){
                            VertexDescriptorGTGraph child_curr = nonmain_children.back();
                            nonmain_children.pop_back();
                            lobe_size_curr++;

                            // check if min lobe size is reached
                            if (lobe_size_curr >= min_lobe_size){
                                nonmain_children.clear();
                                graph[*vit].is_lobe_node = true;
                                break;
                            } else {
                                // process current children
                                for (VertexDescriptorGTGraph child_next: make_iterator_range(adjacent_vertices(child_curr, graph))) {
                                    if (child_next != graph[child_curr].parent){
                                        nonmain_children.push_back(child_next);
                                    }
                                }
                            }
                        }
                        // if nonmain_children is empty and min lobe size was not reached, vertex will remain not a lobe node
                    }
                }
                // node has no main children
                else {

                    // node has non-main children: main leaf case
                    if (!nonmain_children.empty()){

                        // count child nodes
                        int lobe_size_curr = 0;
                        while (!nonmain_children.empty()){
                            VertexDescriptorGTGraph child_curr = nonmain_children.back();
                            nonmain_children.pop_back();
                            lobe_size_curr++;

                            // check if min lobe size is reached
                            if (lobe_size_curr >= min_lobe_size){
                                nonmain_children.clear();
                                graph[*vit].is_lobe_node = true;
                                break;
                            } else {
                                // process current children
                                for (VertexDescriptorGTGraph child_next: make_iterator_range(adjacent_vertices(child_curr, graph))) {
                                    if (child_next != graph[child_curr].parent){
                                        nonmain_children.push_back(child_next);
                                    }
                                }
                            }
                        }

                        // if not enough children, walk back until enough or other lobe node reached
                        // cases:
                        // - parent is lobe node
                        // - step to parent makes lobe big enough
                        // - parent has other main child(ren)
                        if (!graph[*vit].is_lobe_node){
                            VertexDescriptorGTGraph node_curr = *vit;
                            int child_count_curr = lobe_size_curr;
                            bool end_node_search = false;

                            while (!end_node_search){
                                // parent is already detected lobe node
                                if (graph[graph[node_curr].parent].is_lobe_node || graph[node_curr].parent == rootv_){
                                    // de-main previous edge
                                    EdgeDescriptorGTGraph edge_to_parent_curr = edge(node_curr, graph[node_curr].parent, graph).first;
                                    graph[edge_to_parent_curr].is_main = false;
                                    end_node_search = true;
                                }
                                // parent is not lobe node
                                else {

                                    // count main & non-main children of parent node
                                    int count_main_children = 0;
                                    int count_nonmain_children = 0;
                                    for (VertexDescriptorGTGraph v_child: make_iterator_range(adjacent_vertices(graph[node_curr].parent, graph))) {
                                        if (v_child != graph[graph[node_curr].parent].parent) {
                                            EdgeDescriptorGTGraph e_next = edge(graph[node_curr].parent, v_child, graph).first;
                                            if (graph[e_next].is_main) {
                                                count_main_children++;
                                            } else {
                                                count_nonmain_children++;
                                            }
                                        }
                                    }

                                    // step to parent makes lobe big enough
                                    if (child_count_curr + count_nonmain_children + 1 >= min_lobe_size){
                                        // todo: this only counts direct children of parent,
                                        //  should also count all other non-main children for lobe size

                                        // de-main branch, set parent as lobe node
                                        EdgeDescriptorGTGraph edge_to_parent_curr = edge(node_curr, graph[node_curr].parent, graph).first;
                                        graph[edge_to_parent_curr].is_main = false;
                                        graph[graph[node_curr].parent].is_lobe_node = true;
                                        end_node_search = true;
                                    }
                                    // step to parent does not make lobe big enough
                                    else {

                                        // parent has other main children
                                        if (count_main_children > 1){
                                            // de-main edge, stop searching
                                            // de-main edge to enforce all main edges lead to lobes
                                            EdgeDescriptorGTGraph edge_to_parent_curr = edge(node_curr, graph[node_curr].parent, graph).first;
                                            graph[edge_to_parent_curr].is_main = false;
                                            end_node_search = true;
                                        }
                                        // parent has no other main children
                                        else {
                                            // de-main edge, update lobe size,  take another step back
                                            EdgeDescriptorGTGraph edge_to_parent_curr = edge(node_curr, graph[node_curr].parent, graph).first;
                                            graph[edge_to_parent_curr].is_main = false;
                                            child_count_curr += count_nonmain_children + 1;
                                            node_curr = graph[node_curr].parent;

                                            // todo: update lobe size count with ALL non-main children of parent
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return true;
}


/*-------------------------------------------------------------*/
/*-------------------------UTILITY-----------------------------*/
/*-------------------------------------------------------------*/

void GSkeleton::clean_graph(GraphGT& graph){
    // Using boost::clear(vertex() instead of remove_vertex() as remove does not work for undirected graphs.
    // This means that vertices will always remain in the vertex set, but will not be connected to edges after removal.

    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(graph);
    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
        if (graph[*vit].clear_mark){
            // do not delete the root or vertices already deleted previously
            if (*vit != rootv_ && (!graph[*vit].delete_mark)){
                // if vertex is not a leaf, reroute connecting edges around it
                if (degree(*vit, graph) >= 2){
                    VertexDescriptorGTGraph parent = graph[*vit].parent;
                    std::pair<AdjacencyIteratorGTGraph, AdjacencyIteratorGTGraph> avt = adjacent_vertices(*vit, graph);
                    for (AdjacencyIteratorGTGraph ait = avt.first; ait != avt.second; ++ait){
                        // process all successive vertices
                        if (*ait != parent){
                            graph[*ait].parent = parent;

                            SGraphEdgePropGT e_new;
                            e_new.length = graph[*ait].coords.distance2(graph[parent].coords);
                            // set new edge as main if deleted edge to next was also main
                            if (graph[edge(*vit, *ait, graph).first].is_main){
                                e_new.is_main = true;
                            }
                            add_edge(parent, *ait, e_new, graph);
                        }
                    }
                }

                // reset properties
                graph[*vit].parent = *vit;
                graph[*vit].branch_level = -1;
                graph[*vit].normal = {0, 0, 0};
                graph[*vit].weight = -1;
                // todo: normals?

                boost::clear_vertex(*vit, graph);

                graph[*vit].delete_mark = true;
            }
        }
    }
    // reset importances after cleaning
    compute_branching_levels(graph);    // does not need kd-tree
    compute_importance(graph);  // has own call to kd-tree
}


void GSkeleton::simplify_branch(GraphGT& graph,
                                std::vector<VertexDescriptorGTGraph> branch,
                                bool allow_tip_removal,
                                double min_length,
                                double epsilon){
    bool end_branch = false;
    while(!end_branch){
        VertexDescriptorGTGraph v_curr = branch.back();

        // leaf case
        if ((graph[v_curr].parent != v_curr) && (degree(v_curr, graph) == 1)){
            // simplify line up until here, terminate
            simplify_line(graph, branch, allow_tip_removal, epsilon);
            end_branch = true;
        }

        // straight segment case (exclude root! second child will be missed otherwise)
        else if ((graph[v_curr].parent != v_curr) && degree(v_curr, graph) == 2){
            // get next vertex (not parent)
            VertexDescriptorGTGraph nbor;
            std::pair<AdjacencyIteratorGTGraph , AdjacencyIteratorGTGraph> avt = adjacent_vertices(v_curr, graph);
            for (AdjacencyIteratorGTGraph ait = avt.first; ait != avt.second; ++ait){
                if (*ait != graph[v_curr].parent){
                    nbor = *ait;
                }
            }
            // continue current branch with found next
            branch.push_back(nbor);
            continue;
        }

        // branching point case
        else{
            // count next edges that are not tips, gather next vertices
            int non_tip_count = 0;
            std::vector<VertexDescriptorGTGraph> nexts;
            std::pair<AdjacencyIteratorGTGraph , AdjacencyIteratorGTGraph> avt = adjacent_vertices(v_curr, graph);
            for (AdjacencyIteratorGTGraph ait = avt.first; ait != avt.second; ++ait){
                if (*ait != graph[v_curr].parent) {
                    nexts.push_back(*ait);
                    // next edge is not tip
                    if (!((degree(*ait, graph) == 1) && (graph[*ait].parent != *ait))) {
                        non_tip_count++;
                    }
                }
            }
            // process found branches
            for (VertexDescriptorGTGraph v_next : nexts){
                // next edge is tip
                if (((degree(v_next, graph) == 1) && (graph[v_next].parent != v_next))){
                    // only tips were found: continue branch to first tip
                    if (non_tip_count == 0) {
                        branch.push_back(v_next);
                        break;
                    }
                    // remove very short tips
                    EdgeDescriptorGTGraph e_next = edge(v_curr, v_next, graph).first;
                    if (graph[e_next].length < min_length && (!graph[v_next].is_lobe_node)){
                        // hardcoded non-deletion of lobe nodes
                        // no enforcement of non-deletion of tips here!
                        // todo: perhaps extra bool if this turns out to be desirable after all?
                        (graph)[v_next].clear_mark = true;
                    }
                }
                // next edge is not tip
                else{
                    // only 1 non-tip edge: continue branch
                    if (non_tip_count == 1){
                        branch.push_back(v_next);
                        break;
                    }
                    // multiple non-tip edges
                    else{
                        // terminate current branch
                        simplify_line(graph, branch, allow_tip_removal, epsilon);
                        end_branch = true;

                        // process each edge as a new branch recursively
                        std::vector<VertexDescriptorGTGraph> branch_new;
                        branch_new.push_back(v_curr);
                        branch_new.push_back(v_next);
                        simplify_branch(graph, branch_new, allow_tip_removal, min_length, epsilon);
                    }
                }
            }
        }
    }
}


bool GSkeleton::simplify_line(GraphGT& graph,
                              std::vector<VertexDescriptorGTGraph> line,
                              bool allow_tip_removal,
                              double epsilon){
    // do not simplify lines that are too short
    if (line.size() < 3){
        return false;
    }

    // find furthest vertex from spanning line
    int idx_furthest;
    double dist_furthest = -1;
    double dist_curr = 0;
    for (int i = 1; i < line.size()-1; ++i){
        VertexDescriptorGTGraph v_curr = line[i];
        dist_curr = distance_point_to_line(graph[v_curr].coords,
                                           graph[line[0]].coords,
                                           graph[line.back()].coords);

        if (dist_curr > dist_furthest){
            dist_furthest = dist_curr;
            idx_furthest = i;
        }
    }

    // further simplification possible
    if (dist_furthest > epsilon){
        // split line into two at furthest point
        std::vector<VertexDescriptorGTGraph> subline_first(line.begin(), (line.begin() + idx_furthest + 1));
        std::vector<VertexDescriptorGTGraph> subline_last((line.begin() + idx_furthest), line.end());
        // recursively simplify
        simplify_line(graph, subline_first, allow_tip_removal, epsilon);
        simplify_line(graph, subline_last, allow_tip_removal, epsilon);
    }
    // all vertices are within epsilon tolerance, no further simplification
    else{
        for (int j = 1; j < line.size(); ++j){
            // mark all non-branching points for removal
            // hardcoded non-deletion of lobe nodes and multi-child nodes
            if (degree(line[j], graph) <= 2 && (!graph[line[j]].is_lobe_node)){
                // if tips should not be deleted, do extra check
                if (!allow_tip_removal){
                    if (degree(line[j], graph) != 1){
                        (graph)[line[j]].clear_mark = true;
                    }
                } else {
                    (graph)[line[j]].clear_mark = true;
                }
            }
        }
    }

    return true;
}


double GSkeleton::distance_point_to_line(vec3 point, vec3 line_left, vec3 line_right){
//    vec3 pt_a = point;
//    vec3 pt_b = line_right;
//    vec3 pt_c = line_left;
//         a
//        /
//    c---------b

    vec3 dir_line = (line_left - line_right).normalize();
    vec3 line_ab = point - line_right;
    double dist_ab = dot(line_ab, dir_line);
    vec3 a_proj_bc = line_right + (dist_ab * dir_line);

    double distance = length(point - a_proj_bc);
    return distance;
}


VertexDescriptorGTGraph GSkeleton::find_main_bifurcation(){
    // parameters
    int max_level = 10;    // maximum branch level for branch to still be considered main
    int min_length = 5;    // minimum length (nr. edges) for branch to be considered main
    double max_height = 6; // maximum height difference with root coordinates of bifur point

    VertexDescriptorGTGraph v_bifur;
    std::vector<VertexDescriptorGTGraph> trunk_waitlist;    // next point(s) on the trunk to check if bifur point
    trunk_waitlist.push_back(rootv_);

    // contingency list to select best candidate if no perfect bifur point can be found
    std::vector<VertexDescriptorGTGraph> bifur_potentials;

    // from root, check all next nodes until good bifur point found or constraint reached
    bool bifur_found = false;
    while (!trunk_waitlist.empty()){
        // get next potential bifur vertex from stack
        VertexDescriptorGTGraph v_curr = trunk_waitlist.back();
        trunk_waitlist.pop_back();

        // viable children
        std::vector<VertexDescriptorGTGraph> child_candidates;

        // check if root, simply add viable children if so
        if (v_curr == rootv_) {
            for (VertexDescriptorGTGraph v_next: make_iterator_range(adjacent_vertices(v_curr, corresponding_))) {
                EdgeDescriptorGTGraph e_next = edge(v_curr, v_next, corresponding_).first;
                if (corresponding_[e_next].is_main && degree(v_next, corresponding_) > 1) {
                    child_candidates.push_back(v_next);
                }
            }
            // if no viable children, root is bifur point
            if (child_candidates.empty()) {
                v_bifur = rootv_;
                bifur_found = true;
                trunk_waitlist.clear();
                break;
            }
            // add viable children to candidate list
            else {
                for (VertexDescriptorGTGraph v_child : child_candidates){
                    trunk_waitlist.push_back(v_child);
                }
            }
        }
        // check viability of children
        else {
            // store viable children
            for (VertexDescriptorGTGraph v_next: make_iterator_range(adjacent_vertices(v_curr, corresponding_))) {
                if (v_next != corresponding_[v_curr].parent) {
                    // viability of child: is main, not leaf, important enough
                    EdgeDescriptorGTGraph e_next = edge(v_curr, v_next, corresponding_).first;
                    if (corresponding_[e_next].is_main && (degree(v_next, corresponding_) > 1)
                        && (corresponding_[e_next].branch_level <= max_level)) {
                        child_candidates.push_back(v_next);
                    }
                }
            }

            // if no viable children found, add point to potentials and trigger boundary condition
            if (child_candidates.empty()){
                if (trunk_waitlist.empty()){
                    bifur_potentials.push_back(v_curr);
                }
                // else: simpy do nothing = continue next while loop iteration = next candidate
            }

            // straight segment: add child to candidates, continue to next candidate (=child)
            else if (child_candidates.size() == 1){
                trunk_waitlist.push_back(child_candidates[0]);
                // do nothing else = continue to next candidate
            }

            // multiple children: potential bifur point
            else {
                std::vector<VertexDescriptorGTGraph> long_enough_nexts;    // long enough children

                for (auto child : child_candidates){
                    int count_curr = 1;
                    std::vector<VertexDescriptorGTGraph> nexts;
                    nexts.push_back(child);

                    while (count_curr < min_length){
                        // todo: check that min_length > 1?

                        std::vector<VertexDescriptorGTGraph> next_nexts;

                        // count viable children
                        for (VertexDescriptorGTGraph v_next : nexts){
                            for (VertexDescriptorGTGraph v_next_next : make_iterator_range(adjacent_vertices(v_next, corresponding_))){
                                if (v_next_next != corresponding_[v_next].parent){
                                    EdgeDescriptorGTGraph e_next = edge(v_next, v_next_next, corresponding_).first;
                                    if (corresponding_[e_next].is_main && (degree(v_next, corresponding_) > 1)
                                        && (corresponding_[e_next].branch_level <= max_level)) {
                                        next_nexts.push_back(v_next_next);
                                    }
                                }
                            }
                        }

                        // if no viable children, stop loop
                        if (next_nexts.empty()){
                            break;
                        } else{
                            count_curr++;
                            nexts = next_nexts;
                        }
                    }
                    // check if child qualifies
                    if (count_curr >= min_length){
                        long_enough_nexts.push_back(child);
                    }
                }

                // - if no children long enough, trigger boundary condition
                if (long_enough_nexts.empty()){
                    bifur_potentials.push_back(v_curr);
                }
                // - if 1 child long enough, add child to candidate list, continue
                else if (long_enough_nexts.size() == 1){
                    trunk_waitlist.push_back(long_enough_nexts[0]);
                }
                // - if 2+ children are long enough, bifur point = v_curr
                else {
                    v_bifur = v_curr;
                    bifur_found = true;
                    trunk_waitlist.clear();
                }
            }
        }
    }
    if (!bifur_found){
        if (!bifur_potentials.empty()){
            // todo: better selection criteria
            double height_curr_max = 0;
            for (VertexDescriptorGTGraph candidate : bifur_potentials){
                if (corresponding_[candidate].coords.z > height_curr_max){
                    height_curr_max = corresponding_[candidate].coords.z;
                    v_bifur = candidate;
                }
            }
            v_bifur = bifur_potentials[0];
        } else {
            v_bifur = rootv_;
        }
    }

    return v_bifur;
}


/*-------------------------------------------------------------*/
/*--------------------------EXPORT-----------------------------*/
/*-------------------------------------------------------------*/

bool GSkeleton::export_weights(const char *file_out){
    // todo: make function with graph input
    // todo: automatic model file name? file ui?

    // check if graph exists
    if (num_edges(mst_) == 0) {
        std::cerr << "ERROR: could not write xyz file, skeleton has 0 edges" << std::endl;
        return false;
    }

    std::cout << "Writing weights to .xyz: " << file_out << std::endl;

    std::ofstream storageFile;
    storageFile.open(file_out);

    // write header
    storageFile << "x y z i" << std::endl;

    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vts = vertices(mst_);
    for (VertexIteratorGTGraph iter = vts.first; iter != vts.second; ++iter) {
        if (degree(*iter, mst_) != 0 ) { // ignore isolated vertices
            vec3 v_point = mst_[*iter].coords;
            for (int i = 0; i < 3; ++i) {
                v_point[i] = ((float) std::roundf((v_point[i] + translation_[i]) * 1000)) / 1000;
                storageFile << std::setprecision(std::to_string((int) v_point[i]).length() + 3);
                storageFile << v_point[i] << " ";
                storageFile << std::setprecision(-1);
            }
            storageFile << mst_[*iter].weight;

            storageFile << std::endl;
        }
    }

    storageFile.close();
    std::cout << "file stored" <<std::endl;

    return true;
}


bool GSkeleton::export_levels(const char *file_out){
    // todo: make function with graph input
    // todo: automatic model file name? file ui?

    // check if graph exists
    if (num_edges(mst_) == 0) {
        std::cerr << "ERROR: could not write xyz file, skeleton has 0 edges" << std::endl;
        return false;
    }

    std::cout << "Writing branch levels to .xyz: " << file_out << std::endl;

    std::ofstream storageFile;
    storageFile.open(file_out);

    // write header
    storageFile << "x y z i" << std::endl;

    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vts = vertices(mst_);
    for (VertexIteratorGTGraph iter = vts.first; iter != vts.second; ++iter) {
        if (degree(*iter, mst_) != 0 ) { // ignore isolated vertices
            vec3 v_point = mst_[*iter].coords;
            for (int i = 0; i < 3; ++i) {
                v_point[i] = ((float) std::roundf((v_point[i] + translation_[i]) * 1000)) / 1000;
                storageFile << std::setprecision(std::to_string((int) v_point[i]).length() + 3);
                storageFile << v_point[i] << " ";
                storageFile << std::setprecision(-1);
            }
            storageFile << mst_[*iter].branch_level;

            storageFile << std::endl;
        }
    }

    storageFile.close();
    std::cout << "file stored" <<std::endl;

    return true;
}


bool GSkeleton::export_to_ply(const char *file_out) {
    // todo: make function with graph input
    // todo: automatic model file name? file ui?

    // check if graph exists
    if (num_edges(mst_) == 0) {
        std::cerr << "ERROR: could not write ply file, skeleton has 0 edges" << std::endl;
        return false;
    }

    std::cout << "Writing grown skelton to .ply: " << file_out << std::endl;

    // convert the boost graph to Graph (avoid modifying easy3d's GraphIO, or writing IO for boost graph)
    std::vector<easy3d::vec3> vertices_in;
    std::vector<std::tuple<int, int, int>> edges_in;
    std::map<int,int> off_map;
    int off_value = 0;

    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vts = vertices(mst_);
    for (VertexIteratorGTGraph iter = vts.first; iter != vts.second; ++iter) {
        if (boost::degree(*iter, mst_) != 0 ) { // ignore isolated vertices
            vertices_in.emplace_back(mst_[*iter].coords);
            off_map.insert({*iter, off_value});
        } else {
            off_value ++;
        }
    }

    std::pair<EdgeIteratorGTGraph, EdgeIteratorGTGraph> egs = edges(mst_);
    for (EdgeIteratorGTGraph iter = egs.first; iter != egs.second; ++iter) {
        int s_b = source(*iter, mst_);
        int t_b = target(*iter, mst_);
        int level = mst_[*iter].branch_level;

        std::tuple<int, int, int> i = { s_b - off_map[s_b], t_b - off_map[t_b], level };
        edges_in.emplace_back(i);
    }

    std::ofstream storageFile;
    storageFile.open(file_out);

    // write header
    storageFile << "ply" << std::endl;
    storageFile << "format ascii 1.0" << std::endl;
    storageFile << "element vertex " << vertices_in.size() << std::endl;
    storageFile << "property float x" << std::endl;
    storageFile << "property float y" << std::endl;
    storageFile << "property float z" << std::endl;
    storageFile << "element edge " << edges_in.size() << std::endl;
    storageFile << "property int vertex1" << std::endl;
    storageFile << "property int vertex2" << std::endl;
    storageFile << "property float level" << std::endl;
    storageFile << "end_header" << std::endl << std::endl;

    for (vec3 &vertex : vertices_in) {
        for (int i = 0; i < 3; ++i) {
            vertex[i] = ((float) std::roundf((vertex[i] + translation_[i])*1000))/1000;
            storageFile << std::setprecision(std::to_string((int) vertex[i]).length() + 3);
            storageFile << vertex[i] << " ";
            storageFile << std::setprecision(-1);
        }
        storageFile << std::endl;
    }

    storageFile << std::endl;
    for (const std::tuple<int, int, int>& edge: edges_in) {
        storageFile << std::get<0>(edge) << " " << std::get<1>(edge) << " " << std::get<2>(edge) << std::endl;
    }

    storageFile.close();
    std::cout << "skeleton file stored" <<std::endl;

    return true;
}
