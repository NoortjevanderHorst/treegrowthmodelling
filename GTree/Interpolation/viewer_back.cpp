#include "viewer_back.h"


using namespace easy3d;
//using namespace boost;

ViewerB::ViewerB() : ViewerF("Interpolation"), interp_(nullptr) {
    set_background_color(vec4(1, 1, 1, 1));

    camera_->setUpVector(vec3(0, 0, 1));
    camera_->setViewDirection(vec3(-1, 0, 0));
    camera_->showEntireScene();
}


ViewerB::~ViewerB() {
    if (interp_)
        delete interp_;
}


void ViewerB::cleanup() {
    ViewerF::cleanup();
}


std::string ViewerB::usage() const {
    return Viewer::usage() + std::string(
            "  Shift + P:       Show/Hide point cloud       \n"
            "  Shift + G:       Show/Hide skeleton          \n"
            "  Shift + B:       Show/Hide branches          \n"
            "  Shift + L:       Show/Hide leaves            \n"
    );
}


Graph* ViewerB::graph_ts(int time_index) const {
    if (models().size() < time_index){
        std::cout << "ERROR: could not access timestamp graph, index does not exist" << std::endl;
        return nullptr;
    }
    else if (!dynamic_cast<Graph*>(models()[time_index])){
        std::cout << "ERROR: could not access timestamp graph, index is not a graph" << std::endl;
        return nullptr;
    }
    else {
        return dynamic_cast<Graph*>(models()[time_index]);
    }
}


SurfaceMesh* ViewerB::branches(int time_index) const {
    int model_index = interp_->nr_timestamps + (interp_->nr_timestamps - 1) + time_index;

    if (models().size() < model_index){
        std::cout << "ERROR: could not access branch mesh, index does not exist" << std::endl;
        return nullptr;
    }
    else if (!dynamic_cast<SurfaceMesh*>(models()[model_index])){
        std::cout << "ERROR: could not access branch mesh, index is not a surface mesh" << std::endl;
        return nullptr;
    }
    else {
        return dynamic_cast<SurfaceMesh*>(models()[model_index]);
    }
}


bool ViewerB::key_press_event(int key, int modifiers)
{
    // visibility of original graphs
    if (key == GLFW_KEY_1){
        if (!graph_ts(0))
            return false;
        auto vertices = graph_ts(0)->renderer()->get_points_drawable("vertices");
        auto *edges = graph_ts(0)->renderer()->get_lines_drawable("edges");
        if (vertices)
            vertices->set_visible(!vertices->is_visible());
        if (edges)
            edges->set_visible(!edges->is_visible());
        return true;
    }
    else if (key == GLFW_KEY_2){
        if (!graph_ts(1))
            return false;
        auto vertices = graph_ts(1)->renderer()->get_points_drawable("vertices");
        auto *edges = graph_ts(1)->renderer()->get_lines_drawable("edges");
        if (vertices)
            vertices->set_visible(!vertices->is_visible());
        if (edges)
            edges->set_visible(!edges->is_visible());
        return true;
    }
    else if (key == GLFW_KEY_3){
        if (!graph_ts(2))
            return false;
        auto vertices = graph_ts(2)->renderer()->get_points_drawable("vertices");
        auto *edges = graph_ts(2)->renderer()->get_lines_drawable("edges");
        if (vertices)
            vertices->set_visible(!vertices->is_visible());
        if (edges)
            edges->set_visible(!edges->is_visible());
        return true;
    }

    // visibility of intermediary graphs
    else if (key == GLFW_KEY_4){
        if (!graph_ts(3))
            return false;
        auto vertices = graph_ts(3)->renderer()->get_points_drawable("vertices");
        auto *edges = graph_ts(3)->renderer()->get_lines_drawable("edges");
        if (vertices)
            vertices->set_visible(!vertices->is_visible());
        if (edges)
            edges->set_visible(!edges->is_visible());
        return true;
    }
    else if (key == GLFW_KEY_5){
        if (!graph_ts(4))
            return false;
        auto vertices = graph_ts(4)->renderer()->get_points_drawable("vertices");
        auto *edges = graph_ts(4)->renderer()->get_lines_drawable("edges");
        if (vertices)
            vertices->set_visible(!vertices->is_visible());
        if (edges)
            edges->set_visible(!edges->is_visible());
        return true;
    }
    else if (key == GLFW_KEY_RIGHT && modifiers == 0){
        if (!graph_ts(3))
            return false;

        // increase interpolation time (if possible)
        if (inter_idx_ < (interp_->interpos[0][0].size() -1)){
            inter_idx_++;
        }

        // make graph
        LinesDrawable* edraw = graph_ts(3)->renderer()->get_lines_drawable("edges");
        PointsDrawable* vdraw = graph_ts(3)->renderer()->get_points_drawable("vertices");
        GraphB g_inter = interp_->graphs_inter_[0];
        std::vector<easy3d::vec3> pts;

        std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vp = vertices(g_inter);
        for (VertexIteratorGraphB vit = vp.first; vit != vp.second; ++vit) {
            easy3d::vec3 v_curr = interp_->interpos[0][*vit][inter_idx_];
            pts.push_back(v_curr);
        }

        edraw->update_vertex_buffer(pts);
        vdraw->update_vertex_buffer(pts);

        // draw branches (or not) if that option was turned on
        if (show_branches_) {
            if (!branches(inter_idx_)) {
                std::cout << "ERROR: could not visualize interpolated branches, branch mesh does not exist"
                          << std::endl;
                return false;
            }

            branches(inter_idx_)->renderer()->set_visible(true);
        } else {
            if (branches(inter_idx_)){
                branches(inter_idx_)->renderer()->set_visible(false);
            }
        }

        // remove previous branch visualisation
        if (inter_idx_ > 0 && branches(inter_idx_ - 1)){
            branches(inter_idx_-1)->renderer()->set_visible(false);
        }

        return true;
    }
    else if (key == GLFW_KEY_LEFT && modifiers == 0){
        if (!graph_ts(3))
            return false;

        // decrease interpolation time (if possible)
        if (inter_idx_ > 0){
            inter_idx_ -= 1;
        }

        // make graph
        LinesDrawable* edraw = graph_ts(3)->renderer()->get_lines_drawable("edges");
        PointsDrawable* vdraw = graph_ts(3)->renderer()->get_points_drawable("vertices");
        GraphB g_inter = interp_->graphs_inter_[0];
        std::vector<easy3d::vec3> pts;

        std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vp = vertices(g_inter);
        for (VertexIteratorGraphB vit = vp.first; vit != vp.second; ++vit) {
            easy3d::vec3 v_curr = interp_->interpos[0][*vit][inter_idx_];
            pts.push_back(v_curr);
        }

        edraw->update_vertex_buffer(pts);
        vdraw->update_vertex_buffer(pts);

        // draw branches (or not) if that option was turned on
        if (show_branches_) {
            if (!branches(inter_idx_)) {
                std::cout << "ERROR: could not visualize interpolated branches, branch mesh does not exist"
                          << std::endl;
                return false;
            }

            branches(inter_idx_)->renderer()->set_visible(true);
        } else {
            if (branches(inter_idx_)){
                branches(inter_idx_)->renderer()->set_visible(false);
            }
        }

        // remove previous branch visualisation
        if (inter_idx_ < (interp_->interpos[0][0].size() -1) && branches(inter_idx_ + 1)){
            branches(inter_idx_+1)->renderer()->set_visible(false);
        }

        return true;
    }
    else if (key == GLFW_KEY_RIGHT && modifiers == GLFW_MOD_SHIFT){
        if (!graph_ts(4))
            return false;

        // increase interpolation time (if possible)
        if (inter_idx_ < (interp_->interpos[1][0].size() -1)){
            inter_idx_++;
        }

        // make graph
        LinesDrawable* edraw = graph_ts(4)->renderer()->get_lines_drawable("edges");
        PointsDrawable* vdraw = graph_ts(4)->renderer()->get_points_drawable("vertices");
        GraphB g_inter = interp_->graphs_inter_[1];
        std::vector<easy3d::vec3> pts;

        std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vp = vertices(g_inter);
        for (VertexIteratorGraphB vit = vp.first; vit != vp.second; ++vit) {
            easy3d::vec3 v_curr = interp_->interpos[1][*vit][inter_idx_];
            pts.push_back(v_curr);
        }

        edraw->update_vertex_buffer(pts);
        vdraw->update_vertex_buffer(pts);

        // draw branches (or not) if that option was turned on
        int mesh_index = interp_->interpos[1][0].size() + inter_idx_;
        if (show_branches_) {
            if (!branches(mesh_index)) {
                std::cout << "ERROR: could not visualize interpolated branches, branch mesh does not exist"
                          << std::endl;
                return false;
            }

            branches(mesh_index)->renderer()->set_visible(true);
        } else {

            if (branches(mesh_index)){
                branches(mesh_index)->renderer()->set_visible(false);
            }
        }

        // remove previous branch visualisation
        if (inter_idx_ > 0 && branches(mesh_index - 1)){
            branches(mesh_index-1)->renderer()->set_visible(false);
        }

        return true;
    }
    else if (key == GLFW_KEY_LEFT && modifiers == GLFW_MOD_SHIFT){
        if (!graph_ts(4))
            return false;

        // decrease interpolation time (if possible)
        if (inter_idx_ > 0){
            inter_idx_ -= 1;
        }

        // make graph
        LinesDrawable* edraw = graph_ts(4)->renderer()->get_lines_drawable("edges");
        PointsDrawable* vdraw = graph_ts(4)->renderer()->get_points_drawable("vertices");
        GraphB g_inter = interp_->graphs_inter_[1];
        std::vector<easy3d::vec3> pts;

        std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vp = vertices(g_inter);
        for (VertexIteratorGraphB vit = vp.first; vit != vp.second; ++vit) {
            easy3d::vec3 v_curr = interp_->interpos[1][*vit][inter_idx_];
            pts.push_back(v_curr);
        }

        edraw->update_vertex_buffer(pts);
        vdraw->update_vertex_buffer(pts);

        // draw branches (or not) if that option was turned on
        int mesh_index = interp_->interpos[1][0].size() + inter_idx_;
        if (show_branches_) {
            if (!branches(mesh_index)) {
                std::cout << "ERROR: could not visualize interpolated branches, branch mesh does not exist"
                          << std::endl;
                return false;
            }

            branches(mesh_index)->renderer()->set_visible(true);
        } else {
            if (branches(mesh_index)){
                branches(mesh_index)->renderer()->set_visible(false);
            }
        }

        // remove previous branch visualisation
        if (inter_idx_ < (interp_->interpos[1][0].size() -1) && branches(mesh_index + 1)){
            branches(mesh_index+1)->renderer()->set_visible(false);
        }

        return true;
    }

    // show intermediary branches
    else if (key == GLFW_KEY_B && modifiers == GLFW_MOD_SHIFT){
        show_branches_ = !show_branches_;
    }


    else
        return Viewer::key_press_event(key, modifiers);
}


bool ViewerB::save() const {
    SurfaceMesh* mesh; // = branches();
    if (!mesh) {
        std::cerr << "model of branches does not exist" << std::endl;
        return false;
    }

    const std::vector<std::string> filetypes = {"*.obj"};
    const std::string& file_name = dialog::save("save file", mesh->name(), filetypes, true);

    auto start = std::chrono::high_resolution_clock::now();

    if (file_name.empty())
        return false;

    if (SurfaceMeshIO::save(file_name, mesh)) {
        std::cout << "successfully saved the model of branches to file" << std::endl;

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "time taken to save branch geometry file: " << duration.count() << " ms" << std::endl;

        return true;
    }
    else {
        std::cerr << "failed saving the model of branches" << std::endl;
        return false;
    }
}


bool ViewerB::save_interpolation(){
    // interpolations exist
    if (!graph_ts(3) || !graph_ts(4)){
        std::cout << "Could not export interpolation frame, missing interpolation graph(s)." << std::endl;
        return false;
    }

    //  export only what is visible
    int ni = 0;
    Graph* gdraw;
    Graph* g0 = graph_ts(3);
    Graph* g1 = graph_ts(4);
    if (g0->renderer()->is_visible()) {
        gdraw = g0;
    } else if (g1->renderer()->is_visible()){
        gdraw = g1;
        ni = 1;
    } else {
        std::cout << "Could not export interpolation frame, no interpolation graphs visible." << std::endl;
        return false;
    }

    // write to ply
    const std::vector<std::string> filetypes = {"*.ply"};
    const std::string& initial_name = file_system::base_name(graph_ts(0)->name())
            + "_i" + std::to_string(ni) + "_" + std::to_string(inter_idx_) + ".ply";
    const std::string& file_name = dialog::save("save file", initial_name, filetypes, false);
    if (file_name.empty())
        return false;

    std::map<int, int> idx_map; // maps graph index to file index so edges get written correctly
    std::vector<easy3d::vec3> verts_file;
    std::vector<std::tuple<int, int> > edges_file;

    // todo: remove double vertices and collapsed edges?

    // get current interpolated graph vertex coordinates
    GraphB g_inter = interp_->graphs_inter_[ni];
    int i_curr = 0;
    std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vt = vertices(g_inter);
    for (VertexIteratorGraphB vit = vt.first; vit != vt.second; ++vit) {
        easy3d::vec3 vp = interp_->interpos[ni][*vit][inter_idx_];
        idx_map[*vit] = i_curr;
        verts_file.push_back(vp);
        i_curr++;
    }

    // get edges
    std::pair<EdgeIteratorGraphB, EdgeIteratorGraphB> et = edges(g_inter);
    for (EdgeIteratorGraphB eit = et.first; eit != et.second; ++eit){
        VertexDescriptorGraphB v_source = source(*eit, g_inter);
        VertexDescriptorGraphB v_target = target(*eit, g_inter);

        edges_file.emplace_back(idx_map[v_source], idx_map[v_target]);
    }

    std::ofstream storageFile;
    storageFile.open(file_name);

    // write header
    storageFile << "ply" << std::endl;
    storageFile << "format ascii 1.0" << std::endl;
    storageFile << "element vertex " << verts_file.size() << std::endl;
    storageFile << "property float x" << std::endl;
    storageFile << "property float y" << std::endl;
    storageFile << "property float z" << std::endl;
    storageFile << "element edge " << edges_file.size() << std::endl;
    storageFile << "property int vertex1" << std::endl;
    storageFile << "property int vertex2" << std::endl;
    storageFile << "end_header" << std::endl << std::endl;

    // write vertices
    for (auto &vertex : verts_file) {
        for (int i = 0; i < 3; ++i) {
            // allow for larger values being written to avoid rounding
            vertex[i] = ((float) std::roundf(vertex[i] * 1000)) / 1000;
            storageFile << std::setprecision(std::to_string((int) vertex[i]).length() + 3);
            storageFile << vertex[i] << " ";
            storageFile << std::setprecision(-1);
        }
        storageFile << std::endl;
    }

    // write edges
    storageFile << std::endl;
    for (const auto& edge: edges_file) {
        storageFile << std::get<0>(edge) << " " << std::get<1>(edge) << std::endl;
    }

    storageFile.close();
    std::cout << "Skeleton file stored for interpolation " << ni << ", frame " << inter_idx_ <<std::endl;


    return true;
}


bool ViewerB::open_correspondence() {
    if (!interp_)
        interp_ = new Interpolator();

    const std::vector<std::string> filetypes = {"*.csv"};
    std::vector<std::string> file_names = dialog::open("open files", std::string(""), filetypes, true);

    interp_->open_path_files(file_names);

    return true;
}


bool ViewerB::open_ts_graphs() {
    for (auto m : models_)
        delete m;
    models_.clear();

    if (!interp_)
        interp_ = new Interpolator();

    const std::vector<std::string> filetypes = {"*.ply"};
    std::vector<std::string> file_names = dialog::open("open files", std::string(""), filetypes, true);

    int count = 0;
    for (auto const& file_name : file_names) {
        easy3d::Model* gm = add_model(file_name, true);
        fit_screen();
        Graph* g_curr = graph_ts(count);

        // set translation for all loaded (easy3d::)graph points
        bool orig_found = false;
        std::vector<easy3d::vec3> points_trans;
        auto pts = g_curr->get_vertex_property<vec3>("v:point");
        for (auto v : g_curr->vertices()){
            vec3 p_curr = {pts[v].x, pts[v].y, pts[v].z};
            // find origin point/translation if first model
            if (count == 0 && !orig_found){
                trans_ = p_curr;
                orig_found = true;
            }
            vec3 p_trans = p_curr - trans_;
            points_trans.push_back(p_trans);
        }

        // update vertex coordinates of graph model
        auto pts_trans = g_curr->get_vertex_property<vec3>("v:point");
        for (auto v : g_curr->vertices()){
            vec3 p_curr = {pts_trans[v].x, pts_trans[v].y, pts_trans[v].z};
            vec3 p_trans = p_curr - trans_;
            pts_trans[v] = {p_trans.x, p_trans.y, p_trans.z};
        }


        // update model translation property and vertex coordinates
        Graph::ModelProperty<dvec3> tprop = g_curr->add_model_property<dvec3>("translation");
        tprop[0] = static_cast<dvec3>(trans_);

        easy3d::vec3 translation = {(float) tprop[0][0], (float) tprop[0][1], (float) tprop[0][2]};
        if (count == 0)
            std::cout << "graph translation: " << translation << std::endl;


        // make graph a boost graph
        GraphB g_boost;

        // vertices
        VertexDescriptorGraphB rootv = 0;
        auto coords = g_curr->get_vertex_property<vec3>("v:point");
        for (auto v : g_curr->vertices()) {
            // it can be assumed indices correspond between easy3d and boost (tested this)
            vec3 c_new = {coords[v].x, coords[v].y, coords[v].z};
//            std::cout << "\tcoords read: " << c_new << std::endl;
            SGraphVertexPropB v_new;
            v_new.coords = c_new;
            VertexDescriptorGraphB idx_new = add_vertex(v_new, g_boost);
            g_boost[idx_new].i_base = idx_new;

            if (g_boost[v.idx()].coords.z < g_boost[rootv].coords.z){
                rootv = v.idx();
            }
        }
        // root
        g_boost[rootv].parent = rootv;
        g_boost.rootv = rootv;

        // edges
        for (auto e : g_curr->edges()){
            Graph::Vertex vs = g_curr->vertex(e, 0);
            Graph::Vertex vt = g_curr->vertex(e, 1);
            g_boost[vs.idx()].parent = vt.idx();

            SGraphEdgePropB e_new;
            easy3d::vec3 ps = g_boost[vs.idx()].coords;
            easy3d::vec3 pt = g_boost[vt.idx()].coords;
            e_new.length = (pt - ps).length();

            add_edge(vs.idx(), vt.idx(), e_new,g_boost);
        }

        // add boost graph to interpolator base graphs
        interp_->graphs_ts.push_back(g_boost);

        count++;
    }

    std::cout << "loaded all files" << std::endl;
    std::cout << "nr models: " << models_.size() << std::endl;

    return count > 0;
}


bool ViewerB::open(){

    const std::vector<std::string> filetypes = {"*.xyz"};
    std::vector<std::string> file_names = dialog::open("open files", std::string(""), filetypes, true);

    int count = 0;
    for (auto const& file_name : file_names) {
        auto madded = add_model(file_name);
        fit_screen();
        PointCloud* cloud = dynamic_cast<PointCloud*>(madded);
        cloud->renderer()->get_points_drawable("vertices")->set_uniform_coloring(vec4(0.6, 0.6, 1, 0.1));

        // translate
        std::vector<easy3d::vec3> points_trans;
        auto pts = cloud->get_vertex_property<vec3>("v:point");
        for (auto v : cloud->vertices()){
            vec3 pt = {pts[v].x, pts[v].y, pts[v].z};
            vec3 pt_trans = pt - trans_;
            points_trans.push_back(pt_trans);
        }

        auto pdraw = cloud->renderer()->get_points_drawable("vertices");
        pdraw->update_vertex_buffer(points_trans);

        count++;
    }

    return count > 0;
}


void ViewerB::compute_correspondence(){
    interp_->compute_correspondence();

    std::cout << "correspondences computed." << std::endl;

    for (int i = 0; i < (interp_->nr_timestamps - 1); ++i){
        Graph* g_model = new Graph();
        GraphB g_inter;
//        if (i == 0){
//            g_inter = interp_->get_graph_01();
//        } else {
//            g_inter = interp_->get_graph_12();
//        }
        g_inter = interp_->graphs_inter_[i];

        std::cout << "adding interpolation " << i << std::endl;

//        std::cout << "------- vertices ---------" << std::endl;

        // vertices
        std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vp = vertices(g_inter);
        std::vector<Graph::Vertex> v_ins;
        for (VertexIteratorGraphB vit = vp.first; vit != vp.second; ++vit){
            Graph::Vertex v_curr = g_model->add_vertex(g_inter[*vit].coords);
            v_ins.push_back(v_curr);
//            std::cout << "\t" << *vit << " (" << g_inter[*vit].coords << ")" << std::endl;
            if (degree(*vit, g_inter) == 0){
//                std::cout << "\t\tfloating!" << std::endl;
            }
        }

//        std::cout << "------- edges ---------" << std::endl;

        // edges
        std::pair<EdgeIteratorGraphB, EdgeIteratorGraphB> ed = edges(g_inter);
        for (EdgeIteratorGraphB eit = ed.first; eit != ed.second; ++eit){
            VertexDescriptorGraphB vs = source(*eit, g_inter);
            VertexDescriptorGraphB vt = target(*eit, g_inter);

            g_model->add_edge(v_ins[vs], v_ins[vt]);

//            std::cout << "\t" << vs << " <> " << vt << std::endl;
        }

        add_model(g_model);
        fit_screen();
    }
}


bool ViewerB::reconstruct_geometry(){
    /// be aware: does not export point coordinates with translation so other viewers will display the mesh properly!
    /// (points will not be at their original locations, but around (0, 0, 0) instead)

    std::string dir;

    // compute trunk radii of original timestamps
    // so they can be constrained to only increase, and intermediary steps can be interpolated
    std::map<int, double> radii;


    for (int i = 0; i < (interp_->nr_timestamps - 1); ++i) {
        std::cout << "\n###started making branch geometry for interpolation " << i << std::endl;

        GraphB g_target = interp_->graphs_ts[i + 1];

        // make branches and export them for original ts AND interpolated geometry as well
        for (int i_frame = 0; i_frame < (interp_->nr_steps + 2); ++i_frame) {

            // copy to edit points on
            GraphB g_inter = interp_->graphs_inter_[i];

            // reset vertex positions of g_inter graph to reflect current frame
            std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vp = vertices(g_inter);
            for (VertexIteratorGraphB vit = vp.first; vit != vp.second; ++vit) {
                vec3 p_inter = interp_->interpos[i][*vit][i_frame];
                g_inter[*vit].coords = p_inter;
            }

            g_inter.rootv = g_target.rootv;

            // add mesh model
            SurfaceMesh *mesh_result = new SurfaceMesh;

            auto cylfit = new CylFit;

            // trunk radius
            // original ts: estimate trunk, make sure is bigger than last step
            if (i_frame == 0) {
                // get current base radius
                double r_curr;
                if (i == 0) {
                    cylfit->obtain_initial_radius(&g_inter);
                    r_curr = cylfit->get_trunk_radius();
                } else {
                    r_curr = radii[i];
                }

                // get next target radius
                auto cylfit_trunkrad = new CylFit;
                cylfit_trunkrad->obtain_initial_radius(&g_target);
                double r_target = cylfit_trunkrad->get_trunk_radius();

                // set next radius to plausible increase if smaller
                if (r_target < r_curr) {
                    r_target = 1.05 * r_curr;
                    // todo: currently just rough estimate of trunk width increase factor, could be more informed
                }

                // store radii
                radii[i + 1] = r_target;
                if (i == 0) {
                    radii[i] = r_curr;
                } else {
                    // set current radius to previously stored next radius if not first timestamp
                    cylfit->set_trunk_radius(radii[i]);
                }
            }
            // intermediary step: interpolate between current and next
            else {
                cylfit->obtain_initial_radius_inter(radii[i], radii[i + 1],i_frame, interp_->nr_steps);
            }

            cylfit->construct_kd_tree(&g_inter);
            cylfit->set_skeleton(g_inter);

            cylfit->reconstruct_branches(mesh_result);

            mesh_result->set_name(file_system::base_name(graph_ts(0)->name()) + "_branches"
                                  + "_i" + std::to_string(i) + "_" + std::to_string(inter_idx_) + ".obj");

            auto m = add_model(mesh_result);

            m->renderer()->set_visible(false);

            // for debug: toggle writing to file as well or only rendering
            bool tofile = false;

            // todo: writes intermediary timestamps twice (found as base and target)

            // write to ply
            if (tofile) {
                // only open dialogue for the directory once
                if (dir.empty()) {
                    const std::vector<std::string> filetypes = {"*.obj"};
                    const std::string &initial_name = file_system::base_name(graph_ts(0)->name())
                                                      + "_i" + std::to_string(i) + "_" + std::to_string(i_frame) +
                                                      ".obj";
                    const std::string &file_name = dialog::save("save file", initial_name, filetypes, false);

                    size_t last_slash = file_name.find_last_of("/\\");
                    dir = file_name.substr(0, last_slash + 1);

                    if (file_name.empty())
                        return false;
                    if (dir.empty())
                        return false;
                }


                std::string name_specific = file_system::base_name(graph_ts(0)->name())
                                            + "_i" + std::to_string(i) + "_" + std::to_string(i_frame);
                const std::string &file_name_curr = dir + name_specific + ".obj";

                std::ofstream out;
                out.open(file_name_curr);

                SurfaceMesh::VertexProperty<vec3> points = mesh_result->get_vertex_property<vec3>("v:point");
                for (SurfaceMesh::VertexIterator vit = mesh_result->vertices_begin();
                     vit != mesh_result->vertices_end(); ++vit) {
                    vec3 vertex = points[*vit];
                    out << "v ";
                    for (int i = 0; i < 3; ++i) {
                        vertex[i] = ((float) std::roundf((vertex[i]) * 1000)) / 1000;
                        out << std::setprecision(std::to_string((int) vertex[i]).length() + 3);
                        out << vertex[i] << " ";
                        out << std::setprecision(-1);
                    }
                    out << "\n";
                }

                for (SurfaceMesh::FaceIterator fit = mesh_result->faces_begin();
                     fit != mesh_result->faces_end(); ++fit) {
                    out << "f ";
                    SurfaceMesh::VertexAroundFaceCirculator fvit = mesh_result->vertices(*fit), fvend = fvit;
                    SurfaceMesh::HalfedgeAroundFaceCirculator fhit = mesh_result->halfedges(*fit);
                    do {
                        // write vertex index and normal index
                        out << (*fvit).idx() + 1 << " ";
                    } while (++fvit != fvend);
                    out << "\n";
                }

                out.close();

                std::cout << "\tWrote to file: " << name_specific << std::endl;
            }

            std::cout << "Branches mesh added for interpolation " << i << ", keyframe " << i_frame << std::endl << std::endl;
        }
    }


    return true;
}

