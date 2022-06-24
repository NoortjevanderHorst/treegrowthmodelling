//
// Created by noort on 12/05/2022.
//

#include "interpolator.h"


Interpolator::Interpolator() : graphs_ts(3), graphs_inter_(2), paths_v_(2), paths_e_(2), interpos(2){
//    paths_v_[0].clear();
//    paths_e_[0].clear();
//    path_2_v_.clear();
//    path_2_e_.clear();
    graphs_ts.clear();
    graphs_inter_.clear();
    paths_v_.clear();
    paths_e_.clear();
    interpos.clear();
}


Interpolator::~Interpolator() {
//    if (paths_v_[0].size() > 0)
//        paths_v_[0].clear();
//    if (paths_e_[0].size() > 0)
//        paths_e_[0].clear();
//    if (path_2_v_.size() > 0)
//        path_2_v_.clear();
//    if (path_2_e_.size() > 0)
//        path_2_e_.clear();
    if (!graphs_ts.empty())
        graphs_ts.clear();
    if (!graphs_inter_.empty())
        graphs_inter_.clear();
    if (!paths_v_.empty())
        paths_v_.clear();
    if (!paths_e_.empty())
        paths_e_.clear();
    if (!interpos.empty())
        interpos.clear();
}


void Interpolator::open_path_files(std::vector<std::string> path_files) {
    /// assumes all 4 path files are given!
    /// as edges 0 - edges 1 - vertices 0 - vertices 1

    std::cout << "opening path files..." << std::endl;

    int cnt = 0;
    for (auto file : path_files){
        // edges
        if (cnt < 2) {
            std::ifstream fin(file);
            if (fin.is_open()) {
                std::string line;
                std::getline(fin, line);    // ignore header
                while (std::getline(fin, line)) {
                    std::pair<int, int> e0;
                    std::pair<int, int> e1;

                    int cursor = 0;
                    while (cursor < line.size()) {

                        // first edge
                        if (cursor == 0 && line[cursor] == ',') { // check if first edge is empty
                            e0 = std::make_pair(-1, -1);
                            cursor = 2;
                        } else if (cursor == 0) {
                            int commacnt = 0;
                            std::stringstream v00;
                            std::stringstream v01;
                            while (commacnt < 2) {
                                // check if cursor is comma
                                if (line[cursor] == ',') {
                                    commacnt++;
                                } else {
                                    if (commacnt == 1) {
                                        v01 << line[cursor];
                                    } else {
                                        v00 << line[cursor];
                                    }
                                }
                                cursor++;
                            }

                            // write vertices as int
                            int v00_i;
                            int v01_i;
                            v00 >> v00_i;
                            v01 >> v01_i;
                            e0 = std::make_pair(v00_i, v01_i);
                        }

                        // second edge
                        if (cursor != 0 && line[cursor] == ',') {   // check if second edge is empty
                            e1 = std::make_pair(-1, -1);
                            cursor += 2;
                        } else {
                            int commacnt = 0;
                            std::stringstream v10;
                            std::stringstream v11;
                            while (commacnt < 2) {
                                // check if cursor is comma
                                if (line[cursor] == ',') {
                                    commacnt++;
                                } else {
                                    if (commacnt == 1) {
                                        v11 << line[cursor];
                                    } else {
                                        v10 << line[cursor];
                                    }
                                }
                                cursor++;
                            }

                            // write vertices as int
                            int v10_i;
                            int v11_i;
                            v10 >> v10_i;
                            v11 >> v11_i;
                            e1 = std::make_pair(v10_i, v11_i);
                        }
                    }

                    paths_e_[cnt].emplace_back(e0, e1);
                }
            }
        }

        // vertices
        if (cnt >= 2) {
            std::ifstream fin(file);
            if (fin.is_open()) {
                std::string line;
                std::getline(fin, line);    // ignore header
                while (std::getline(fin, line)) {
                    int v0;
                    int v1;

                    int cursor = 0;
                    while (cursor < line.size()) {

                        // first vert
                        if (cursor == 0 && line[cursor] == ',') { // check if first vert is empty
                            v0 = -1;
                            cursor = 1;
                        } else if (cursor == 0) {
                            std::stringstream v0ss;
                            while (line[cursor] != ',') {
                                v0ss << line[cursor];
                                cursor++;
                            }

                            // write vertices as int
                            v0ss >> v0;
                        }

                        // second vert
                        if (cursor != 0 && line[cursor] == ',') {   // check if second vert is empty
                            v1 = -1;
                            cursor += 1;
                        } else {
                            int commacnt = 0;
                            std::stringstream v1ss;
                            while (commacnt < 1) {
                                // check if cursor is comma
                                if (line[cursor] == ',') {
                                    commacnt++;
                                } else {
                                    v1ss << line[cursor];
                                }
                                cursor++;
                            }

                            // write vertices as int;
                            v1ss >> v1;
                        }
                    }

                    paths_v_[cnt-2].emplace_back(std::make_pair(v0, v1));
                }
            }
        }

        cnt++;
    }

    /*std::cout << "--- egdes 0 ---" << std::endl;
    for (auto edge_step : paths_e_[0]){
        std::cout << edge_step.first.first << " " << edge_step.first.second << "-->"
            << edge_step.second.first << " " << edge_step.second.second << std::endl;
    }

    std::cout << "--- egdes 1 ---" << std::endl;
    for (auto edge_step : paths_e_[1]){
        std::cout << edge_step.first.first << " " << edge_step.first.second << "-->"
                  << edge_step.second.first << " " << edge_step.second.second << std::endl;
    }

    std::cout << "--- vertices 0 ---" << std::endl;
    for (auto vert_step : paths_v_[0]){
        std::cout << vert_step.first << "-->" << vert_step.second << std::endl;
    }

    std::cout << "--- vertices 1 ---" << std::endl;
    for (auto vert_step : paths_v_[1]){
        std::cout << vert_step.first << "-->" << vert_step.second << std::endl;
    }*/


    std::cout << "opened files." << std::endl;
}


void Interpolator::compute_correspondence(){
    std::cout << "graph ts 0 verts: " << num_vertices(graphs_ts[0]) << ", edges: " << num_edges(graphs_ts[0]) << std::endl;
    std::cout << "graph ts 1 verts: " << num_vertices(graphs_ts[1]) << ", edges: " << num_edges(graphs_ts[1]) << std::endl;
    std::cout << "graph ts 2 verts: " << num_vertices(graphs_ts[2]) << ", edges: " << num_edges(graphs_ts[2]) << std::endl;

    graphs_inter_.push_back(graphs_ts[0]);
    graphs_inter_.push_back(graphs_ts[1]);

    for (int cnt = 0; cnt < graphs_inter_.size(); ++cnt) {
        // todo: deletion adds to next graph, need local copies here of ts graphs to not break the next interpolation!!
        // ugly fix...
        if (cnt == 1){
            graphs_ts[cnt] = graphs_inter_[cnt];
        }

        /*std::cout << "\n\n###########################################################################" <<
        "\n###########################################################################" <<
        "\n###########################################################################" << std::endl;

        std::cout << "\n### graphs created" << std::endl;
        std::cout << "interpolation " << cnt << std::endl;
        std::cout << "------- vertices ---------" << std::endl;
        // vertices
        std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vp0 = vertices(graphs_inter_[cnt]);
        for (VertexIteratorGraphB vit = vp0.first; vit != vp0.second; ++vit){
            std::cout << "\t" << *vit << " (" << graphs_inter_[cnt][*vit].coords << ")" << std::endl;
        }
        std::cout << "------- edges ---------" << std::endl;
        // edges
        std::pair<EdgeIteratorGraphB, EdgeIteratorGraphB> ed = edges(graphs_inter_[cnt]);
        for (EdgeIteratorGraphB eit = ed.first; eit != ed.second; ++eit){
            VertexDescriptorGraphB vs = source(*eit, graphs_inter_[cnt]);
            VertexDescriptorGraphB vt = target(*eit, graphs_inter_[cnt]);
            std::cout << "\t" << vs << " <> " << vt << std::endl;
        }*/

        // add placeholder vertices so indices match up...
        int vert_diff = num_vertices(graphs_ts[cnt+1]) - num_vertices(graphs_ts[cnt]);
        int idx_base = num_vertices(graphs_ts[cnt]) - 1;
        int vcount = 1;
        while (vcount <= vert_diff) {
            int idx_new = idx_base + vcount;
            SGraphVertexPropB v_new;
            v_new.coords = graphs_ts[cnt+1][idx_new].coords;
            add_vertex(v_new, graphs_inter_[cnt]);
            vcount++;
        }
        // todo: algo fails if nr verts ts 0 > nr verts ts 1...

        /*std::cout << "\n### added placeholder extra verts" << std::endl;
        std::cout << "interpolation " << cnt << std::endl;
        std::cout << "------- vertices ---------" << std::endl;
        // vertices
        std::pair<VertexIteratorGraphB, VertexIteratorGraphB> v1 = vertices(graphs_inter_[cnt]);
        for (VertexIteratorGraphB vit = v1.first; vit != v1.second; ++vit){
            std::cout << "\t" << *vit << " (" << graphs_inter_[cnt][*vit].coords << ")" << std::endl;
        }*/

        std::map<VertexDescriptorGraphB, VertexDescriptorGraphB> idx_map;   // maps idx 0 <> idx1 & idx 01
        std::map<VertexDescriptorGraphB, VertexDescriptorGraphB> idx_map_reverse;   // idx1 & idx 01 <> maps idx 0
        for (auto v_path: paths_v_[cnt]) {
//            std::cout << "- vert path: " << v_path.first << " <> " << v_path.second << std::endl;
            // insertion
            if (v_path.first == -1) {
                // ts base
                SGraphVertexPropB v_new;
                v_new.coords = graphs_ts[cnt+1][v_path.second].coords;
                v_new.c_base = graphs_ts[cnt+1][v_path.second].coords;
                v_new.c_target = graphs_ts[cnt+1][v_path.second].coords;
                v_new.insert_mark = true;
                VertexDescriptorGraphB i_new = add_vertex(v_new, graphs_ts[cnt]);
                graphs_ts[cnt][i_new].i_base = i_new;
                graphs_ts[cnt][i_new].i_target = v_path.second;

                // inter
                graphs_inter_[cnt][v_path.second].i_base = i_new;
                graphs_inter_[cnt][v_path.second].i_target = v_path.second;
                graphs_inter_[cnt][v_path.second].c_base = graphs_ts[cnt+1][v_path.second].coords;
                graphs_inter_[cnt][v_path.second].c_target = graphs_ts[cnt+1][v_path.second].coords;
                graphs_inter_[cnt][v_path.second].insert_mark = true;
                // reset coords to end position (will have coords of old index otherwise)
                graphs_inter_[cnt][v_path.second].coords = graphs_ts[cnt+1][v_path.second].coords;

                idx_map[i_new] = v_path.second;
                idx_map_reverse[v_path.second] = i_new;

//                std::cout << "\tinsertion: " << i_new << " --> " << v_path.second << std::endl;
            }
            // deletion
            else if (v_path.second == -1){
                // add extra to inter
                SGraphVertexPropB v_new;
                v_new.coords = graphs_ts[cnt][v_path.first].coords;
                v_new.c_base = graphs_ts[cnt][v_path.first].coords;
                v_new.c_target = graphs_ts[cnt][v_path.first].coords;
                v_new.inter_delete_mark = true;
                v_new.i_base = v_path.first;
                VertexDescriptorGraphB i_new = add_vertex(v_new, graphs_ts[cnt+1]);
                VertexDescriptorGraphB i_new_inter = add_vertex(v_new, graphs_inter_[cnt]);

                graphs_ts[cnt+1][i_new].i_target = i_new;
                graphs_inter_[cnt][i_new_inter].i_target = i_new_inter;

                idx_map[v_path.first] = i_new;
                idx_map_reverse[i_new] = v_path.first;

//                std::cout << "\tdeletion: " << v_path.first << " <-- " << i_new << std::endl;
//                std::cout << "\t\ti new: " << i_new << ", i inter: " << i_new_inter << std::endl;
            }
            // transformation
            else {
                // ts base
                graphs_ts[cnt][v_path.first].i_base = v_path.first;
                graphs_ts[cnt][v_path.first].i_target = v_path.second;
                graphs_ts[cnt][v_path.first].c_base = graphs_ts[cnt][v_path.first].coords;
                graphs_ts[cnt][v_path.first].c_target = graphs_ts[cnt+1][v_path.second].coords;

                // inter
                graphs_inter_[cnt][v_path.second].i_base = v_path.first;
                graphs_inter_[cnt][v_path.second].i_target = v_path.second;
                graphs_inter_[cnt][v_path.second].c_base = graphs_ts[cnt][v_path.first].coords;
                graphs_inter_[cnt][v_path.second].c_target = graphs_ts[cnt+1][v_path.second].coords;
                // reset coords to end position (will have coords of old index otherwise)
                graphs_inter_[cnt][v_path.second].coords = graphs_ts[cnt][v_path.first].coords;

                idx_map[v_path.first] = v_path.second;
                idx_map_reverse[v_path.second] = v_path.first;

//                std::cout << "\ttransformation: " << v_path.first << " <> " << v_path.second << std::endl;
            }
        }

        /*std::cout << "\n### processed vert paths" << std::endl;
        std::cout << "interpolation " << cnt << std::endl;
        std::cout << "------- vertices ---------" << std::endl;
        // vertices
        std::pair<VertexIteratorGraphB, VertexIteratorGraphB> v2 = vertices(graphs_inter_[cnt]);
        for (VertexIteratorGraphB vit = v2.first; vit != v2.second; ++vit){
            std::cout << "\t" << *vit << " (" << graphs_inter_[cnt][*vit].coords << ")" << std::endl;
        }

        std::cout << "\nvertex transformations:" << std::endl;
        for (VertexIteratorGraphB vit = v2.first; vit != v2.second; ++vit){
            std::cout << "v " << *vit << ": " << graphs_inter_[cnt][*vit].i_base
            << " --> " << graphs_inter_[cnt][*vit].i_target << std::endl;
        }*/

        correspond_edges(idx_map, cnt);

        /*std::cout << "\n### processed edge paths" << std::endl;
        std::cout << "interpolation " << cnt << std::endl;
        std::cout << "------- vertices ---------" << std::endl;
        // vertices
        std::pair<VertexIteratorGraphB, VertexIteratorGraphB> v3 = vertices(graphs_inter_[cnt]);
        for (VertexIteratorGraphB vit = v3.first; vit != v3.second; ++vit){
            std::cout << "\t" << *vit << " (" << graphs_inter_[cnt][*vit].coords << ")" << std::endl;
            std::cout << "\t\t" << "source: " << graphs_inter_[cnt][*vit].i_base << " (" << graphs_inter_[cnt][*vit].c_base << ")" << std::endl;
            std::cout << "\t\t" << "target: " << graphs_inter_[cnt][*vit].i_target << " (" << graphs_inter_[cnt][*vit].c_target << ")" << std::endl;
        }

        std::cout << "------- edges ---------" << std::endl;

        std::pair<EdgeIteratorGraphB, EdgeIteratorGraphB> edd = edges(graphs_ts[cnt+1]);
        for (EdgeIteratorGraphB eit = edd.first; eit != edd.second; ++eit) {
            VertexDescriptorGraphB vs = source(*eit, graphs_ts[cnt+1]);
            VertexDescriptorGraphB vt = target(*eit, graphs_ts[cnt+1]);
            auto e_found = edge(vs, vt, graphs_inter_[cnt]);
            if (!e_found.second){
                std::cout << "\t" << vs << " <> " << vt << std::endl;
                std::cout << "\t\tcoords: " << graphs_ts[cnt+1][vs].coords << " <> " << graphs_ts[cnt+1][vt].coords << std::endl;
            }

        }

        std::cout << "\n### processed edge paths" << std::endl;
        std::cout << "interpolation " << cnt << std::endl;
        std::cout << "------- vertices ---------" << std::endl;
        // vertices
        std::pair<VertexIteratorGraphB, VertexIteratorGraphB> v3 = vertices(graphs_inter_[cnt]);
        for (VertexIteratorGraphB vit = v3.first; vit != v3.second; ++vit){
            std::cout << "\t" << *vit << " (" << graphs_inter_[cnt][*vit].coords << ")" << std::endl;
            std::cout << "\t\t" << "source: " << graphs_inter_[cnt][*vit].i_base << " (" << graphs_inter_[cnt][*vit].c_base << ")" << std::endl;
            std::cout << "\t\t" << "target: " << graphs_inter_[cnt][*vit].i_target << " (" << graphs_inter_[cnt][*vit].c_target << ")" << std::endl;
        }
        std::cout << "------- edges ---------" << std::endl;
        // edges
        std::pair<EdgeIteratorGraphB, EdgeIteratorGraphB> edd = edges(graphs_inter_[cnt]);
        for (EdgeIteratorGraphB eit = edd.first; eit != edd.second; ++eit){
            VertexDescriptorGraphB vs = source(*eit, graphs_inter_[cnt]);
            VertexDescriptorGraphB vt = target(*eit, graphs_inter_[cnt]);
            std::cout << "\t" << vs << " <> " << vt << std::endl;
//            std::cout << "\t\told: " << idx_map_reverse[vs] << " <> " << idx_map_reverse[vt] << std::endl;
//            std::cout << "\t\tcollapsed: " << (graphs_inter_[cnt][vs].coords == graphs_inter_[cnt][vt].coords) << std::endl;
//            std::cout << "\t\tcoords: " << graphs_inter_[cnt][vs].coords << " <> " << graphs_inter_[cnt][vt].coords << std::endl;

            easy3d::vec3 ps_tar = graphs_inter_[cnt][vs].c_target;
            easy3d::vec3 pt_tar = graphs_inter_[cnt][vt].c_target;
            if (ps_tar != pt_tar) {
                std::cout << "\t\tcoords target: " << ps_tar << " <> " << pt_tar << std::endl;
                auto e = edge(vs, vt, graphs_ts[cnt + 1]);
                if (e.second){
                    std::cout << "\t\texists in target" << std::endl;
                } else {
                    std::cout << "\t\tNOT in target" << std::endl;
                }
            } else {
                std::cout << "\t\tcollapsed" << std::endl;
            }
        }*/

    }

    // todo: put this in a better place
    std::cout << "interpolating..." << std::endl;
    interpolate();
    std::cout << "interpolation done" << std::endl;

}


void Interpolator::correspond_edges(std::map<VertexDescriptorGraphB, VertexDescriptorGraphB> idx_map, int ts){
    // todo: make an edge map from ts0 indices to ts1 indices, so the edge steps will work for the intermediary
    // todo: transform via edges

    // todo: edge transforms
    // 0. delete edges between old indices in g01
    // 1. draw old edges between new node indices
    // 2. make copies for add and delete, set to starting positions (use index map!)
    // 3. interpolate
    // 4. animate

//    std::cout << "\n################ edge correspondence for ts " << ts << std::endl;

    GraphB graph_ebase  = graphs_inter_[ts];
    GraphB graph_etarget = graphs_inter_[ts];

    // old edges --> correct old edges
    std::pair<EdgeIteratorGraphB, EdgeIteratorGraphB> et_b = edges(graph_ebase);
    for (EdgeIteratorGraphB eit = et_b.first; eit != et_b.second; ++eit){
        remove_edge(*eit, graph_ebase);
    }

//    std::cout << "\n- add edges to base:" << std::endl;

    std::pair <EdgeIteratorGraphB, EdgeIteratorGraphB> et_ts = edges(graphs_ts[ts]);
    for (EdgeIteratorGraphB eit = et_ts.first; eit != et_ts.second; ++eit) {
        VertexDescriptorGraphB v_source = idx_map[source(*eit, graphs_ts[ts])];
        VertexDescriptorGraphB v_target = idx_map[target(*eit, graphs_ts[ts])];
        add_edge(v_source, v_target, graph_ebase);

//        std::cout << "\tadded edge " << v_source << " <> " << v_target << " to base" << std::endl;
    }

//    std::cout << "\n- add edges to target:" << std::endl;

    // old edges --> new edges
    std::vector<std::pair<VertexDescriptorGraphB, VertexDescriptorGraphB> > twin_edges;   // edges added to target that already existed in base
    for (auto step : paths_e_[ts]){
        std::pair<int, int> e0 = step.first;
        std::pair<int, int> e1 = step.second;
//        std::cout << "- step " << e0.first << " " << e0.second << " <> " << e1.first << " " << e1.second << std::endl;

        // insertion
        if (e0.first == -1){
            add_edge(e1.first, e1.second, graph_etarget);
            // adds new vertex with default coordinates if it doesn't exist yet
//            std::cout << "\t(ins) added edge " << e1.first << " <> " << e1.second << " to target" << std::endl;
        }

        // deletion
        else if (e1.first == -1){
            remove_edge(e0.first, e0.second, graph_etarget);
//            std::cout << "\t(del) removed edge " << e0.first << " <> " << e0.second << " from target" << std::endl;
        }

        // substitution = del (+add)
        else {
            // only delete if it was not added previously
            bool twin_found = false;
            for (auto e_twin : twin_edges){
                if ((e0.first == e_twin.first && e0.second == e_twin.second) ||
                (e0.first == e_twin.second && e0.second == e_twin.first)){
                    twin_found = true;
                }
            }

            if (!twin_found){
                remove_edge(e0.first, e0.second, graph_etarget);
//                std::cout << "\t(sub) removed edge " << e0.first << " <> " << e0.second << " from target" << std::endl;
            }

            add_edge(e1.first, e1.second, graph_etarget);
//            std::cout << "\t(sub) added edge " << e1.first << " <> " << e1.second << " to target" << std::endl;

            // store twin edges that should NOT be deleted the second time they are edited
            bool e_exist = edge(e1.first, e1.second, graph_ebase).second;
            if (e_exist){
                twin_edges.emplace_back(e1.first, e1.second);
//                std::cout << "\t(sub) TWIN edge: " << e1.first << " <> " << e1.second << std::endl;
            }
        }

    }
    std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vt_01t = vertices(graph_etarget);
    for (VertexIteratorGraphB vit = vt_01t.first; vit != vt_01t.second; ++vit){
        graph_etarget[*vit].coords = graph_etarget[*vit].c_target;
    }

    // set inserted points to base positions in intermediary graph
    std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vt_b = vertices(graph_ebase);
    for (VertexIteratorGraphB vit = vt_b.first; vit != vt_b.second; ++vit){
        // set all floating vertices to the location of their first neighbour that has a non-floating neighbour
        std::vector<VertexDescriptorGraphB> v_float;
        if (degree(*vit, graph_ebase) == 0 && graphs_inter_[ts][*vit].c_base == graphs_inter_[ts][*vit].c_target){
            std::vector<VertexDescriptorGraphB> n_float;
            std::vector<VertexDescriptorGraphB> n_conn;
            for (VertexDescriptorGraphB v_next: make_iterator_range(adjacent_vertices(*vit, graph_etarget))){
                // find floating & connected neighbours
                if (degree(v_next, graph_ebase) != 0){
                    n_conn.push_back(v_next);
                } else if (graphs_inter_[ts][v_next].c_base == graphs_inter_[ts][v_next].c_target) {
                    n_float.push_back(v_next);
                }
            }

            if (!n_conn.empty()){
                VertexDescriptorGraphB parent = n_conn[0];
                graphs_inter_[ts][*vit].c_base = graphs_inter_[ts][parent].coords;
                graphs_inter_[ts][*vit].coords = graphs_inter_[ts][parent].coords;
                graph_ebase[*vit].c_base = graphs_inter_[ts][parent].coords;
                graph_ebase[*vit].coords = graphs_inter_[ts][parent].coords;
                graph_etarget[*vit].c_base = graphs_inter_[ts][parent].coords;
                // set the rest of the new branch as well
                while(!n_float.empty()){
                    VertexDescriptorGraphB v_curr = n_float.back();
                    n_float.pop_back();
                    graphs_inter_[ts][v_curr].c_base = graphs_inter_[ts][parent].coords;
                    graphs_inter_[ts][v_curr].coords = graphs_inter_[ts][parent].coords;
                    graph_ebase[v_curr].c_base = graphs_inter_[ts][parent].coords;
                    graph_ebase[v_curr].coords = graphs_inter_[ts][parent].coords;
                    graph_etarget[v_curr].c_base = graphs_inter_[ts][parent].coords;

                    for (VertexDescriptorGraphB v_next_f: make_iterator_range(adjacent_vertices(v_curr, graph_etarget))){
                        if (degree(v_next_f, graph_ebase) == 0
                        && graphs_inter_[ts][v_next_f].c_base == graphs_inter_[ts][v_next_f].c_target){
                            n_float.push_back(v_next_f);
                        }
                    }
                }
            }
        }
    }

//    std::cout << "\n- chained deletion target pos setting: " << std::endl;

    // set deleted points to correct target positions as well
    for (VertexIteratorGraphB vit = vt_b.first; vit != vt_b.second; ++vit){
        // get all leaf deleted verts
        std::vector<VertexDescriptorGraphB> v_del;
//        std::cout << "v " << *vit << ": ";
        if (graphs_inter_[ts][*vit].c_base == graphs_inter_[ts][*vit].c_target && graph_ebase[*vit].inter_delete_mark){
//            std::cout << "del" << std::endl;

            std::vector<VertexDescriptorGraphB> n_del;
            std::vector<VertexDescriptorGraphB> n_exist;
            int cnt = 0;
            for (VertexDescriptorGraphB v_next: make_iterator_range(adjacent_vertices(*vit, graph_ebase))){
                // find floating & connected neighbours
                if (!graphs_inter_[ts][v_next].inter_delete_mark){
                    n_exist.push_back(v_next);
                }
                // nbor also is to be deleted and has not been visited yet
                else if (graphs_inter_[ts][v_next].c_base == graphs_inter_[ts][v_next].c_target) {
                    n_del.push_back(v_next);
                }
                cnt++;
            }

//            std::cout << "\tnr nbor del: " << n_del.size() << ", exist: " << n_exist.size() << ", total: " << cnt << std::endl;

            if (!n_exist.empty()){
                VertexDescriptorGraphB parent = n_exist[0];
//                std::cout << "\tparent: " << parent << std::endl;

                graphs_inter_[ts][*vit].c_target = graph_etarget[parent].coords;
                graph_ebase[*vit].c_target = graph_etarget[parent].coords;
                graph_etarget[*vit].coords = graph_etarget[parent].coords;
                graph_etarget[*vit].c_target = graph_etarget[parent].coords;
                // set the rest of the new branch as well
                while(!n_del.empty()){
                    VertexDescriptorGraphB v_curr = n_del.back();
                    n_del.pop_back();
//                    std::cout << "\tdeleted nbor: " << v_curr << std::endl;

                    graphs_inter_[ts][v_curr].c_target = graph_etarget[parent].coords;
                    graph_ebase[v_curr].c_target = graph_etarget[parent].coords;
                    graph_etarget[v_curr].coords = graph_etarget[parent].coords;
                    graph_etarget[v_curr].c_target = graph_etarget[parent].coords;

                    for (VertexDescriptorGraphB v_next_f: make_iterator_range(adjacent_vertices(v_curr, graph_ebase))){
                        if (graphs_inter_[ts][v_next_f].inter_delete_mark
                            && graphs_inter_[ts][v_next_f].c_base == graphs_inter_[ts][v_next_f].c_target){
                            n_del.push_back(v_next_f);
                        }
                    }
                }
            }
        } else {
//            std::cout << "not del" << std::endl;
        }
    }

    // clear edges from intermediary graph
    std::pair<EdgeIteratorGraphB, EdgeIteratorGraphB> et_int = edges(graphs_inter_[ts]);
    for (EdgeIteratorGraphB eit = et_int.first; eit != et_int.second; ++eit){
        remove_edge(*eit, graphs_inter_[ts]);
    }

//    std::cout << "\n- add deletion doubles to inter:" << std::endl;

    // make deletion doubles
    std::pair<EdgeIteratorGraphB, EdgeIteratorGraphB> et_bn = edges(graph_ebase);
    for (EdgeIteratorGraphB eit = et_bn.first; eit != et_bn.second; ++eit){
        VertexDescriptorGraphB v_source = source(*eit, graph_ebase);
        VertexDescriptorGraphB v_target = target(*eit, graph_ebase);
        auto e = edge(v_source, v_target, graph_etarget);
        if (!e.second){
            // add double vertex
            SGraphVertexPropB v_new;
            v_new.coords = graph_ebase[v_target].coords;
            v_new.c_base = graph_ebase[v_target].coords;
            v_new.c_target = graph_etarget[v_source].coords;
            v_new.copy_mark = true;
            VertexDescriptorGraphB i_new = add_vertex(v_new, graphs_inter_[ts]);

            // add edge
            add_edge(v_source, i_new, graphs_inter_[ts]);

//            std::cout << "\tadded edge (new, del) " << v_source << " <> " << i_new << " to inter" << std::endl;
//            std::cout << "\t\told edge " << v_source << " <> " << v_target << std::endl;
        }
        // if it exists in both base and target, simply add it to the intermediary graph
        else {
            add_edge(v_source, v_target, graphs_inter_[ts]);
//            std::cout << "\tadded edge (exist) " << v_source << " <> " << v_target << " to inter" << std::endl;
        }
    }

//    std::cout << "\n- add insertion doubles to inter:" << std::endl;

    // make insertion doubles
    std::pair<EdgeIteratorGraphB, EdgeIteratorGraphB> et_t = edges(graph_etarget);
    for (EdgeIteratorGraphB eit = et_t.first; eit != et_t.second; ++eit){
        VertexDescriptorGraphB v_source = source(*eit, graph_etarget);
        VertexDescriptorGraphB v_target = target(*eit, graph_etarget);
        auto e = edge(v_source, v_target, graph_ebase);
        if (!e.second){
            // do not add between existing vertices, always add double (incorrect edges otherwise)
            SGraphVertexPropB v_new;
            v_new.coords = graph_ebase[v_source].coords;
            v_new.c_base = graph_ebase[v_source].coords;
            v_new.c_target = graph_etarget[v_target].coords;
            v_new.copy_mark = true;
            VertexDescriptorGraphB i_new = add_vertex(v_new, graphs_inter_[ts]);

            // add edge
            add_edge(v_source, i_new, graphs_inter_[ts]);
//            std::cout << "\tadded edge (new, ins) " << v_source << " <> " << i_new << " to inter" << std::endl;
//            std::cout << "\t\told edge " << v_source << " <> " << v_target << std::endl;
        }
    }

//    std::cout << "setting target positionss..." ;

    // set all positions to starting coordinates
    std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vt = vertices(graphs_inter_[ts]);
    for (VertexIteratorGraphB vit = vt.first; vit != vt.second; ++vit){
        graphs_inter_[ts][*vit].coords = graphs_inter_[ts][*vit].c_base;
    }

//    std::cout << " done." << std::endl;

}


void Interpolator::interpolate(){
    for (auto g_inter : graphs_inter_) {
        std::map<VertexDescriptorGraphB, std::vector<easy3d::vec3> > res;

        std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vt = vertices(g_inter);
        for (VertexIteratorGraphB vit = vt.first; vit != vt.second; ++vit) {
            // get positions
            std::vector<easy3d::vec3> endpos;
            easy3d::vec3 startpt = g_inter[*vit].c_base;
            easy3d::vec3 endpt = g_inter[*vit].c_target;
            easy3d::vec3 midpt = {(endpt.x + startpt.x) / 2, (endpt.y + startpt.y) / 2, (endpt.z + startpt.z) / 2};
            endpos.push_back(startpt);
            endpos.push_back(midpt);    // needed because interpolator wants 3 points
            endpos.push_back(endpt);

            // interpolate
            easy3d::SplineCurveInterpolation<easy3d::vec3> interpolator;
            interpolator.set_boundary(easy3d::SplineCurveInterpolation<easy3d::vec3>::second_deriv,
                                      0,
                                      easy3d::SplineCurveInterpolation<easy3d::Vec<3, float>>::second_deriv,
                                      0,
                                      false);

            std::vector<float> t(endpos.size(), 0.0f);
            for (std::size_t i = 0; i < endpos.size(); ++i)
                t[i] = i;
            interpolator.set_points(t, endpos);

            // get points
            std::vector<easy3d::vec3> points;
            const unsigned int resolution = ((nr_steps + 1) * 10) + 1;// 61; // nr of splits in spline (= (nr frames - 1) / 10 + 1)
            for (unsigned int i = 0; i < resolution; i += 10) {
                const easy3d::vec3 p = interpolator.eval_f(float(i) / float(resolution - 1));
//            std::cout << "\tcurve point " << i << ": " << p << std::endl;
                points.push_back(p);
            }

            // add to interpolation point list
            res[*vit] = points;
        }

        interpos.push_back(res);
    }

//    std::cout << "resulting interpos:" << std::endl;
//    for (auto p : interpos){
//        std::cout << graphs_inter_[0][p.first].c_base << " <> " << graphs_inter_[0][p.first].c_target << std::endl;
//        for (auto ip : p.second){
//            std::cout << "\t" << ip << std::endl;
//        }
//
//    }
}