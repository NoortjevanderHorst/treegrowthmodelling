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

#include "growth_viewer.h"


using namespace easy3d;


GrowthViewer::GrowthViewer()
        : ViewerImGuiGrow("GTree"),
        gtree_(nullptr)
{
    set_background_color(vec4(1, 1, 1, 1));

    camera_->setUpVector(vec3(0, 0, 1));
    camera_->setViewDirection(vec3(-1, 0, 0));
    camera_->showEntireScene();

    shadow_ = new SoftShadow(camera());
    shadow_->set_sample_pattern(SoftShadow::SamplePattern(2));
    shadow_->set_darkness(0.3f);
    shadow_->set_softness(0.9f);

//    std::cout << usage() << std::endl;
}

GrowthViewer::~GrowthViewer() {}


/*-------------------------------------------------------------*/
/*--------------------------ACCESS-----------------------------*/
/*-------------------------------------------------------------*/

std::string GrowthViewer::usage() const {
    return Viewer::usage() + std::string(
        "  Shift + 'a':         Load and reconstruct test dataset A       \n"
            "  Shift + 'b':         Load and reconstruct test dataset B       \n"
            "  Shift + 'c':         Load and reconstruct test dataset C       \n"
            "  Shift + 'd':         Load and reconstruct test dataset D       \n"
    );
}


PointCloud* GrowthViewer::cloud_ts(int time_index) const {
    if (models().size() < time_index){
        std::cout << "ERROR: could not access timestamp point cloud, index does not exist" << std::endl;
        return nullptr;
    }
    else if (!dynamic_cast<PointCloud*>(models()[time_index])){
        std::cout << "ERROR: could not access timestamp point cloud, index is not a point cloud" << std::endl;
        return nullptr;
    }
    else {
        return dynamic_cast<PointCloud*>(models()[time_index]);
    }
}


const GraphGT* GrowthViewer::skeleton_ts(SkeletonType type, int time_index) const {
    const GraphGT* skel = nullptr;
    if (trees_.size() < time_index){
        std::cout << "ERROR: could not access timestamp skeleton, index does not exist" << std::endl;
        return skel;
    }
    else {
        switch (type) {
            case ST_DELAUNAY:
                skel = &(trees_[time_index]->get_delaunay());
                break;
            case ST_MST:
                skel = &(trees_[time_index]->get_mst());
                break;
            case ST_SIMPLIFIED:
                skel = &(trees_[time_index]->get_simplified());
                break;
            case ST_CORRESPONDING:
                skel = &(trees_[time_index]->get_corresponding());
                break;
            case ST_TS_MAIN:
                skel = &(trees_[time_index]->get_ts_main());
                break;
        }
        return skel;
    }
}


SurfaceMesh* GrowthViewer::lobe_ts(int time_index) const {
    if (trees_.empty()){
        std::cout << "ERROR: could not could not access timestamp lobe mesh, no point clouds were added" << std::endl;
        return nullptr;
    } else {
        int mesh_index = trees_.size() + time_index;
        if (models().size() <= mesh_index){
            std::cout << "ERROR: could not access timestamp lobe mesh, index " << mesh_index << " does not exist" << std::endl;
            return nullptr;
        }
        else if (!dynamic_cast<SurfaceMesh*>(models()[mesh_index])){
            std::cout << "ERROR: could not access timestamp lobe mesh, index is not a mesh" << std::endl;
            return nullptr;
        }
        else {
            return dynamic_cast<SurfaceMesh*>(models()[mesh_index]);
        }
    }
}


SurfaceMesh* GrowthViewer::branches_ts(int time_index, int type_idx) const {
    if (trees_.empty()){
        std::cout << "ERROR: could not could not access timestamp branches mesh, no point clouds were added" << std::endl;
        return nullptr;
    } else {
        int mesh_index = trees_.size() + (trees_.size() - 1);
        // corresponding
        if (type_idx == 3){
            mesh_index += 2 * time_index;
        }
        // ts main
        else if (type_idx == 4){
            mesh_index += 2 * time_index + 1;
        } else {
            std::cout << "ERROR: could not access timestamp branches mesh, type index " << type_idx << " does not exist" << std::endl;
            return nullptr;
        }

        if (models().size() <= mesh_index){
            std::cout << "ERROR: could not access timestamp branches mesh, index " << mesh_index << " does not exist" << std::endl;
            return nullptr;
        }
        else if (!dynamic_cast<SurfaceMesh*>(models()[mesh_index])){
            std::cout << "ERROR: could not access timestamp branches mesh, index is not a mesh" << std::endl;
            return nullptr;
        }
        else {
            return dynamic_cast<SurfaceMesh*>(models()[mesh_index]);
        }
    }
}


bool GrowthViewer::ts_visualisation(int ts_index, int item_index, bool show, int skeleton_type, std::vector<ImVec4> colors){
    // convert skeleton type int index to string name
    SkeletonType skel_type = static_cast<SkeletonType>(skeleton_type);

    // --- initial checks
    // check if desired timestamp point cloud exists
    if (!cloud_ts(ts_index)){
        std::cout << "ERROR: could not visualize timestamp, cloud does not exist" << std::endl; // todo: message may be redundant
        return false;
    }

    // --- set visibility/visuals of point cloud dependent drawables
    // points
    if (item_index == 0){
        cloud_ts(ts_index)->renderer()->set_visible(show);
        vec4 vcolor = {colors[0].x, colors[0].y, colors[0].z, colors[0].w};
        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->set_uniform_coloring(vcolor);
        return true;
    }

    // --- set visibility/visuals of skeleton-dependent drawables
    // check if skeleton has been reconstructed before accessing dependent drawables
    if (trees_.size() < (ts_index + 1)){
        std::cout << "Skeleton reconstruction needed in order to visualise the selected item(s). "
                     "Starting skeleton reconstruction of all loaded point clouds." << std::endl;
        reconstruct_multitemporal();
    }

    // vertex importance
    if (item_index == 1){
        update_importance_visuals_vertices(skel_type, ts_index, show, colors[0]);
        return true;
    }
    // normals
    if (item_index == 2){
        LinesDrawable* normals_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("normals");
        if (!normals_drawable)
            create_normals_drawable(ts_index);
        if (normals_drawable)
            normals_drawable->set_visible(show);
        return true;
    }
    // edges
    if (item_index == 3){
        // always create drawable again to make sure it draws the currently selected skeleton graph type
        create_skeleton_drawable(skel_type, ts_index, colors[1]);
        LinesDrawable* graph_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("graph");
        if (graph_drawable)
            graph_drawable->set_visible(show);
        return true;
    }
    // branching levels
    if (item_index == 4){
        LinesDrawable* graph_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("graph");
        if (!graph_drawable)
            create_skeleton_drawable(skel_type, ts_index, colors[1]);
        update_importance_visuals_edges(skel_type, ts_index, show, colors[1]);
        return true;
    }
    // correspondence
    if (item_index == 5){
        LinesDrawable* graph_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("graph");
        if (!graph_drawable)
            create_skeleton_drawable(skel_type, ts_index, colors[1]);
        update_correspondence_visuals(skel_type, ts_index, show, colors[1]);
        return true;
    }
    // main skeleton
    if (item_index == 6){
        LinesDrawable* main_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("main");
        if (!main_drawable)
            create_main_skeleton_drawable(skel_type, ts_index);
        if (main_drawable)
            main_drawable->set_visible(show);
        return true;
    }
    // main skeleton vertices correspondence
    if (item_index == 7){
        show_vertex_correspondence(skel_type, ts_index, show, colors[0]);
        return true;
    }
    // corresponding all-timestamp skeleton
    if (item_index == 8){
        LinesDrawable* graph_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("graph");
        if (!graph_drawable)
            create_skeleton_drawable(skel_type, ts_index, colors[1]);
        show_edge_correspondence(skel_type, ts_index, show, colors[1]);
        return true;
    }
    // bifurcation points
    if (item_index == 9){
        show_bifur(skel_type, ts_index, show, colors[0]);
        return true;
    }
    // lobe clusters
    if (item_index == 10){
        show_lobes(skel_type, ts_index, show, colors[0]);
        return true;
    }
    // lobe meshes
    if (item_index == 11){
        if (!lobe_ts(ts_index)){
            std::cout << "ERROR: could not visualize lobes, mesh does not exist" << std::endl;
            return false;
        }

        lobe_ts(ts_index)->renderer()->set_visible(show);
        return true;
    }
    // lobe skeleton (grown)
    if (item_index == 12){
        LinesDrawable* lobe_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("lobes");
        if (!lobe_drawable)
            create_lobe_skeleton_drawable(skel_type, ts_index);
        if (lobe_drawable)
            lobe_drawable->set_visible(show);
        return true;
    }
    // skeleton distance (ts & merged main edges)
    if (item_index == 13){
        LinesDrawable* graph_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("graph");
        if (!graph_drawable)
            create_skeleton_drawable(skel_type, ts_index, colors[1]);
        show_skeleton_distance(skel_type, ts_index, show, colors[1]);
    }
    // branches mesh
    if (item_index == 14){
        if (!branches_ts(ts_index, skeleton_type)){
            std::cout << "ERROR: could not visualize branches, mesh does not exist" << std::endl;
            return false;
        }

        branches_ts(ts_index, skeleton_type)->renderer()->set_visible(show);
        return true;
    }

    return false;
}


bool GrowthViewer::inter_visualisation(int item_index, bool show) {
    // check if trees exist
    if (item_index > (trees_.size() - 3)){
        std::cout << "ERROR: could not draw interpolations, the tree(s) at these indices were not reconstructed" << std::endl;
        return false;
    }

    // get ts main skeleton type
    SkeletonType skel_type = static_cast<SkeletonType>(4);
    // check if skeletons exist
    if (!skeleton_ts(skel_type, item_index)){
        std::cout << "ERROR: could not draw interpolation, index " << item_index << " does not have a ts main skeleton" << std::endl;
        return false;
    }
    if (!skeleton_ts(skel_type, item_index + 1)){
        std::cout << "ERROR: could not draw interpolation, index " << item_index + 1 << " does not have a ts main skeleton" << std::endl;
        return false;
    }

    // make/get drawable
    LinesDrawable* inter_drawable = cloud_ts(item_index)->renderer()->get_lines_drawable("inter");
    if (!inter_drawable)
        create_inter_skeleton_drawable(item_index);

    // set visibility
    if (inter_drawable)
        inter_drawable->set_visible(show);

    return true;
}


bool GrowthViewer::ts_change_colors(int ts_index, int item_index, ImVec4 color){
    // item_index 0 = vertices
    // item_index 1 = edges

    // check if desired timestamp point cloud exists
    if (!cloud_ts(ts_index)){
        std::cout << "ERROR: could not color timestamp, cloud does not exist" << std::endl; // todo: message may be redundant
        return false;
    }
    // check if item index is valid
    if (item_index < 0 || item_index > 1){
        std::cout << "ERROR: could not color timestamp, item index out of bounds (0 <> 1)" << std::endl;
        return false;
    }

    // vertices
    if (item_index == 0){
        PointsDrawable* pointcloud_drawable = cloud_ts(ts_index)->renderer()->get_points_drawable("vertices");
        pointcloud_drawable->set_uniform_coloring(vec4({color.x, color.y, color.z, 1}));
        return true;
    }

    // edges
    if (item_index == 1){
        // check if skeleton has been reconstructed before accessing dependent drawables
        if (trees_.size() < (ts_index + 1)){
            std::cout << "Skeleton reconstruction needed in order to color the selected item(s). "
                         "Please reconstruct skeleton first." << std::endl;
            return false;
        }

        LinesDrawable* graph_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("graph");
        if (!graph_drawable) {
            // MST as default, but will color the current skeleton type
            create_skeleton_drawable(ST_MST, ts_index, color);
            LinesDrawable *graph_drawable_new = cloud_ts(ts_index)->renderer()->get_lines_drawable("graph");
            // color and visibility get changed when adding, colors should be set correctly
            // and graph drawable should not be visible when created because of color input
            graph_drawable_new->set_uniform_coloring(vec4({color.x, color.y, color.z, 1.0f}));
            graph_drawable_new->set_visible(false);
        }
        if (graph_drawable)
            graph_drawable->set_uniform_coloring(vec4({color.x, color.y, color.z, 1.0f}));

        return true;
    }

    return false;
}


bool GrowthViewer::key_press_event(int key, int modifiers)
{
    // debug commands for running all operations on 4 test datasets
    if (key == GLFW_KEY_A && modifiers == GLFW_MOD_SHIFT){
        // import files
        static std::vector<std::string> filenames(3);
        filenames.resize(3);
        std::string folder = "../../resources/data/GTree_examples/A/";
        filenames[0] = folder + "26CN1_ahn2_tree_0e76be44-6026-4edd-9800-dbf03aa47e40.xyz";
        filenames[1] = folder + "26CN1_ahn3_tree_0e76be44-6026-4edd-9800-dbf03aa47e40.xyz";
        filenames[2] = folder + "26CN1_ahn4_tree_0e76be44-6026-4edd-9800-dbf03aa47e40.xyz";

        complete_multitemporal_import(filenames);

        // reconstruction
        reconstruct_multitemporal();

        // merged structure
        add_merged_cloud();

        // correspondence
        model_correspondence();

        // growth
        model_growth();

        // branches
        reconstruct_geometry();

        // interpolation
        model_interpolation();

        return true;
    }

    else if (key == GLFW_KEY_B && modifiers == GLFW_MOD_SHIFT){
        // import files
        static std::vector<std::string> filenames(3);
        filenames.resize(3);
        std::string folder = "../../resources/data/GTree_examples/B/";
        filenames[0] = folder + "26CN1_ahn2_tree_7a095ea2-f289-4b80-bd86-dbd7c00092aa.xyz";
        filenames[1] = folder + "26CN1_ahn3_tree_7a095ea2-f289-4b80-bd86-dbd7c00092aa.xyz";
        filenames[2] = folder + "26CN1_ahn4_tree_7a095ea2-f289-4b80-bd86-dbd7c00092aa.xyz";

        complete_multitemporal_import(filenames);

        // reconstruction
        reconstruct_multitemporal();

        // merged structure
        add_merged_cloud();

        // correspondence
        model_correspondence();

        // growth
        model_growth();

        // branches
        reconstruct_geometry();

        // interpolation
        model_interpolation();

        return true;
    }

    else if (key == GLFW_KEY_C && modifiers == GLFW_MOD_SHIFT){
        // import files
        static std::vector<std::string> filenames(3);
        filenames.resize(3);
        std::string folder = "../../resources/data/GTree_examples/C/";
        filenames[0] = folder + "26CN1_ahn2_tree_43c62baa-384a-45cd-a326-a8a3c274cee5.xyz";
        filenames[1] = folder + "26CN1_ahn3_tree_43c62baa-384a-45cd-a326-a8a3c274cee5.xyz";
        filenames[2] = folder + "26CN1_ahn4_tree_43c62baa-384a-45cd-a326-a8a3c274cee5.xyz";

        complete_multitemporal_import(filenames);

        // reconstruction
        reconstruct_multitemporal();

        // merged structure
        add_merged_cloud();

        // correspondence
        model_correspondence();

        // growth
        model_growth();

        // branches
        reconstruct_geometry();

        // interpolation
        model_interpolation();

        return true;
    }

    else if (key == GLFW_KEY_D && modifiers == GLFW_MOD_SHIFT){
        // import files
        static std::vector<std::string> filenames(3);
        filenames.resize(3);
        std::string folder = "../../resources/data/GTree_examples/D/";
        filenames[0] = folder + "26CN1_ahn2_tree_503089de-2979-448e-be2e-c394f35cfba5.xyz";
        filenames[1] = folder + "26CN1_ahn3_tree_503089de-2979-448e-be2e-c394f35cfba5.xyz";
        filenames[2] = folder + "26CN1_ahn4_tree_503089de-2979-448e-be2e-c394f35cfba5.xyz";

        complete_multitemporal_import(filenames);

        // reconstruction
        reconstruct_multitemporal();

        // merged structure
        add_merged_cloud();

        // correspondence
        model_correspondence();

        // growth
        model_growth();

        // branches
        reconstruct_geometry();

        // interpolation
        model_interpolation();

        return true;
    }

    else
        return Viewer::key_press_event(key, modifiers);

    return Viewer::key_press_event(key, modifiers);
}


void GrowthViewer::draw() const {
    if (!shadowing_enabled_) {
        Viewer::draw();
        return;
    }

    if (cloud_ts(0)) {
        const mat4& MVP = camera_->modelViewProjectionMatrix();
        // camera position is defined in world coordinate system.
        const vec3& wCamPos = camera_->position();
        // it can also be computed as follows:
        //const vec3& wCamPos = invMV * vec4(0, 0, 0, 1);
        const mat4& MV = camera_->modelViewMatrix();
        const vec4& wLightPos = inverse(MV) * setting::light_position;

        // check if any point cloud models are visible
        bool pc_is_visible = false;
        if (!trees_.empty()) {
            for (int i = 0; i < trees_.size(); ++i){
                if (cloud_ts(i)->renderer()->is_visible()){
                    pc_is_visible = true;
                }
            }
        }

        if (pc_is_visible) {
            ShaderProgram* program = program = ShaderManager::get_program("points_color");
            if (!program) {
                std::vector<ShaderProgram::Attribute> attributes;
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::POSITION, "vtx_position"));
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::COLOR, "vtx_color"));
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::NORMAL, "vtx_normal"));
                program = ShaderManager::create_program_from_files("points_color", attributes);
            }
            if (program) {
                program->bind();
                program->set_uniform("MVP", MVP);
                program->set_uniform("wLightPos", wLightPos);
                program->set_uniform("wCamPos", wCamPos);
                program->set_uniform("ssaoEnabled", false);
                for (auto m : models_) {
                    if (!m->renderer()->is_visible())
                        continue;
                    for (auto d : m->renderer()->points_drawables()) {
                        if (d->is_visible()) {
                            program->set_uniform("lighting", d->normal_buffer());
                            program->set_uniform("per_vertex_color", d->color_buffer());
                            program->set_uniform("default_color", vec4(0.0f, 0.0f, 0.0f, 1.0f));
                            d->draw(camera());
                        }
                    }
                }
                program->release();
            }
        }

        // check if any graph models are visible
        bool graph_is_visible = false;
        if (!trees_.empty()) {
            for (int i = 0; i < trees_.size(); ++i){
                LinesDrawable* graph_drawable = cloud_ts(i)->renderer()->get_lines_drawable("graph");
                if (graph_drawable && graph_drawable->is_visible()){
                    graph_is_visible = true;
                }
            }
        }

        if (graph_is_visible) {
            ShaderProgram* program = ShaderManager::get_program("lines_color");
            if (!program) {
                std::vector<ShaderProgram::Attribute> attributes;
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::POSITION, "vtx_position"));
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::COLOR, "vtx_color"));
                program = ShaderManager::create_program_from_files("lines_color", attributes);
            }
            if (program) {
                program->bind();
                program->set_uniform("MVP", MVP);
                program->set_uniform("per_vertex_color", false);
                program->set_uniform("default_color", vec4(0.0f, 0.0f, 0.0f, 1.0f));
                for (auto m : models_) {
                    if (!m->renderer()->is_visible())
                        continue;
                    for (auto d: m->renderer()->lines_drawables()) {
                        if (d->is_visible()) {
                            d->draw(camera());
                        }
                    }
                }
                program->release();
            }
        }
    }

    // check if any surface meshes are visible
    std::vector<TrianglesDrawable*> surfaces;
    if (!trees_.empty()) {
        for (int i = 0; i < trees_.size(); ++i) {
            // check if any lobes are visible
            if (lobe_ts(i) && lobe_ts(i)->renderer()->is_visible()){
                for (auto d : lobe_ts(i)->renderer()->triangles_drawables()){
                    surfaces.push_back(d);
                }
            }

            // check if any branches are visible
            if (branches_ts(i, 3) && branches_ts(i, 3)->renderer()->is_visible()){
                for (auto d : branches_ts(i, 3)->renderer()->triangles_drawables()){
                    surfaces.push_back(d);
                }
            }
            if (branches_ts(i, 4) && branches_ts(i, 4)->renderer()->is_visible()){
                for (auto d : branches_ts(i, 4)->renderer()->triangles_drawables()){
                    surfaces.push_back(d);
                }
            }
        }
    }
    shadow_->draw(surfaces);
}


void GrowthViewer::cleanup() {
    if (trees_.size() > 0)
        trees_.clear();

    if (shadow_)
        delete shadow_;

    if (gtree_)
        delete gtree_;

    ViewerImGuiGrow::cleanup();
}


/*-------------------------------------------------------------*/
/*-------------------------DRAWING-----------------------------*/
/*-------------------------------------------------------------*/

bool GrowthViewer::update_importance_visuals_vertices(SkeletonType type, int ts_index, bool show, ImVec4 default_color){
    // reset colors if to be not shown
    if (!show){
        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->set_uniform_coloring(
                vec4(default_color.x, default_color.y, default_color.z, default_color.w));
    } else { // if importance to be visible, color vertices
        const GraphGT* skeleton_draw = skeleton_ts(type, ts_index);

        //create the vertices vector for rendering
        std::vector<vec3> points;
        std::vector<vec3> colors;

        std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(*skeleton_draw);
        for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
            vec3 color;
            // set deleted vertex color to almost white
            if ((*skeleton_draw)[*vit].weight == -1 || (*skeleton_draw)[*vit].delete_mark){ // todo: why does weight check not work??
                color = {0.85, 0.85, 1};
            } else {
                float intensity = (*skeleton_draw)[*vit].weight / trees_[ts_index]->get_max_weight();
                // todo: this only uses latest max! changes after simplified construction...
                color = colormap(intensity);
            }
            colors.push_back(color);

            points.push_back((*skeleton_draw)[*vit].coords);
        }

        auto pcdraw = cloud_ts(ts_index)->renderer()->get_points_drawable("vertices");

        pcdraw->update_vertex_buffer(points);
        pcdraw->update_color_buffer(colors);
        pcdraw->set_property_coloring(State::VERTEX);
    }

    return true;
}


bool GrowthViewer::create_normals_drawable(int ts_index)
{
    if (!cloud_ts(ts_index))
        return false;

    const ::GraphGT* skeleton_draw = nullptr;
    skeleton_draw = &(trees_[ts_index]->get_mst());

    if (!skeleton_draw)
    {
        std::cout << "skeleton does not exist" << std::endl;
        return false;
    }

    //create the vertices vector for rendering
    std::vector<vec3> graph_points;
    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(*skeleton_draw);
    double normal_length = 0.2;

    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit)
    {
        vec3 p_curr = (*skeleton_draw)[*vit].coords;
        vec3 normal_resized = (*skeleton_draw)[*vit].normal * normal_length;
        vec3 p_norm = (*skeleton_draw)[*vit].coords + normal_resized;

        graph_points.push_back(p_curr);
        graph_points.push_back(p_norm);
    }

    //initialize the line drawable object;
    LinesDrawable* graph_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("normals");
    if (!graph_drawable) {
        graph_drawable = cloud_ts(ts_index)->renderer()->add_lines_drawable("normals");
    }
    graph_drawable->update_vertex_buffer(graph_points);
    graph_drawable->set_uniform_coloring(vec4(1.0f, 0.0f, 0.0f, 1.0f));

    return true;
}


bool GrowthViewer::create_skeleton_drawable(SkeletonType type, int skeleton_index, ImVec4 default_color)
{
    if (!cloud_ts(skeleton_index))
        return false;

    const GraphGT* skeleton_draw = skeleton_ts(type, skeleton_index);

    if (!skeleton_draw)
    {
        std::cout << "skeleton does not exist" << std::endl;
        return false;
    }

    //create the vertices vector for rendering
    std::vector<vec3> graph_points;
    std::vector<vec3> colors;
    std::pair<EdgeIteratorGTGraph , EdgeIteratorGTGraph> ep = edges(*skeleton_draw);
    VertexDescriptorGTGraph dVertex1, dVertex2;
    vec3 pVertex1, pVertex2;
    for (EdgeIteratorGTGraph eIter = ep.first; eIter != ep.second; ++eIter)
    {
        dVertex1 = source(*eIter, *skeleton_draw);
        dVertex2 = target(*eIter, *skeleton_draw);
        pVertex1 = (*skeleton_draw)[dVertex1].coords;
        pVertex2 = (*skeleton_draw)[dVertex2].coords;
        assert(!has_nan(pVertex1));
        assert(!has_nan(pVertex2));
        graph_points.push_back(pVertex1);
        graph_points.push_back(pVertex2);
    }

    //initialize the line drawable object;
    LinesDrawable* graph_drawable = cloud_ts(skeleton_index)->renderer()->get_lines_drawable("graph");
    if (!graph_drawable)
        graph_drawable = cloud_ts(skeleton_index)->renderer()->add_lines_drawable("graph");
    graph_drawable->update_vertex_buffer(graph_points);
    graph_drawable->set_uniform_coloring(vec4(default_color.x, default_color.y, default_color.z, default_color.w));

    return true;
}


bool GrowthViewer::update_importance_visuals_edges(SkeletonType type, int ts_index, bool show, ImVec4 default_color){
    LinesDrawable* graph_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("graph");

    // if importance visible, set colors back to black
    if (!show){
        graph_drawable->set_uniform_coloring(
                vec4(default_color.x, default_color.y, default_color.z, default_color.w));
    } else { // if importance not visible, color edges
        const GraphGT* skeleton_draw = skeleton_ts(type, ts_index);

        //create the vertices vector for rendering
        std::vector<vec3> colors;
        std::pair<EdgeIteratorGTGraph , EdgeIteratorGTGraph> ep = edges(*skeleton_draw);
        VertexDescriptorGTGraph dVertex1, dVertex2;
        vec3 pVertex1, pVertex2;
        for (EdgeIteratorGTGraph eIter = ep.first; eIter != ep.second; ++eIter)
        {
            dVertex1 = source(*eIter, *skeleton_draw);
            dVertex2 = target(*eIter, *skeleton_draw);
            pVertex1 = (*skeleton_draw)[dVertex1].coords;
            pVertex2 = (*skeleton_draw)[dVertex2].coords;
            assert(!has_nan(pVertex1));
            assert(!has_nan(pVertex2));

            float intensity1 = (*skeleton_draw)[dVertex1].branch_level / trees_[ts_index]->get_max_branching_level();
            float intensity2 = (*skeleton_draw)[dVertex2].branch_level / trees_[ts_index]->get_max_branching_level();
            vec3 color1 = colormap(intensity1);
            vec3 color2 = colormap(intensity2);
            colors.push_back(color1);
            colors.push_back(color2);
        }

        graph_drawable->update_color_buffer(colors);
        graph_drawable->set_property_coloring(State::VERTEX);
    }

    return true;
}


bool GrowthViewer::update_correspondence_visuals(SkeletonType type, int ts_index, bool show, ImVec4 default_color){
    LinesDrawable* graph_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("graph");

    // if importance to be not visible, set colors back to base color
    if (!show){
        graph_drawable->set_uniform_coloring(
                vec4(default_color.x, default_color.y, default_color.z, default_color.w));
    } else { // if importance yo be visible, color edges
        const GraphGT* skeleton_draw = skeleton_ts(type, ts_index);

        //create the vertices vector for rendering
        std::vector<vec3> colors;
        std::pair<EdgeIteratorGTGraph , EdgeIteratorGTGraph> ep = edges(*skeleton_draw);
        VertexDescriptorGTGraph dVertex1, dVertex2;
        vec3 pVertex1, pVertex2;
        for (EdgeIteratorGTGraph eIter = ep.first; eIter != ep.second; ++eIter)
        {
            // todo: make work for other amounts of timestamps than 3

            vec3 color = {0, 0, 0};
            if ((*skeleton_draw)[*eIter].correspondence.size() == 1){
                color = {1, 0, 0};
            }
            if ((*skeleton_draw)[*eIter].correspondence.size() == 2){
                color = {0, 1, 0};
            }
            if ((*skeleton_draw)[*eIter].correspondence.size() == 3){
                color = {0, 0, 1};
            }

            colors.push_back(color);
            colors.push_back(color);
        }

        graph_drawable->update_color_buffer(colors);
        graph_drawable->set_property_coloring(State::VERTEX);
    }

    return true;
}


bool GrowthViewer::create_main_skeleton_drawable(SkeletonType type, int ts_index){
    const GraphGT* skeleton_draw = skeleton_ts(type, ts_index);

    // create the vertices vector for rendering
    std::vector<vec3> colors;
    std::vector<vec3> graph_points;
    std::pair<EdgeIteratorGTGraph , EdgeIteratorGTGraph> ep = edges(*skeleton_draw);
    VertexDescriptorGTGraph dVertex1, dVertex2;
    vec3 pVertex1, pVertex2;
    for (EdgeIteratorGTGraph eIter = ep.first; eIter != ep.second; ++eIter)
    {
        vec3 color = {0.5, 0.5, 0.5};
        if ((*skeleton_draw)[*eIter].is_main){
            // store colors
            color = {1, 0.2, 0.2};

            colors.push_back(color);
            colors.push_back(color);

            // store selected vertices
            dVertex1 = source(*eIter, *skeleton_draw);
            dVertex2 = target(*eIter, *skeleton_draw);
            pVertex1 = (*skeleton_draw)[dVertex1].coords;
            pVertex2 = (*skeleton_draw)[dVertex2].coords;
            assert(!has_nan(pVertex1));
            assert(!has_nan(pVertex2));

            graph_points.push_back(pVertex1);
            graph_points.push_back(pVertex2);
        }
    }

    // create main skeleton drawable
    LinesDrawable* graph_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("main");
    if (!graph_drawable)
        graph_drawable = cloud_ts(ts_index)->renderer()->add_lines_drawable("main");

    graph_drawable->update_color_buffer(colors);
    graph_drawable->update_vertex_buffer(graph_points);
    graph_drawable->set_property_coloring(State::VERTEX);

    return true;
}


bool GrowthViewer::show_vertex_correspondence(SkeletonType type, int ts_index, bool show, ImVec4 default_color){
    // if importance to be not visible, set colors back to base color
    if (!show){
        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->set_uniform_coloring(
                vec4(default_color.x, default_color.y, default_color.z, default_color.w));
    } else { // if importance to be visible, color vertices
        const GraphGT* skeleton_draw = skeleton_ts(type, ts_index);

        //create the vertices vector for rendering
        std::vector<vec3> points;
        std::vector<vec3> colors;

        std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(*skeleton_draw);
        for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
            vec3 color;
            // set deleted vertex color to almost white
            if ((*skeleton_draw)[*vit].weight == -1 || (*skeleton_draw)[*vit].delete_mark){
                color = {0.85, 0.85, 1};
            } else {
                if ((*skeleton_draw)[*vit].is_main){
                    color = {1, 0, 0};
                } else {
                    color = {1, 0.7, 0.7};
                }
            }
            colors.push_back(color);

            points.push_back((*skeleton_draw)[*vit].coords);
        }

        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->update_vertex_buffer(points);
        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->update_color_buffer(colors);
        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->set_property_coloring(State::VERTEX);
    }

    return true;
}


bool GrowthViewer::show_edge_correspondence(SkeletonType type, int ts_index, bool show, ImVec4 default_color){
    LinesDrawable* graph_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("graph");

    // if importance visible, set colors back to black
    if (!show){
        graph_drawable->set_uniform_coloring(
                vec4(default_color.x, default_color.y, default_color.z, default_color.w));
    } else { // if importance not visible, color edges
        const GraphGT* skeleton_draw = skeleton_ts(type, ts_index);

        //create the vertices vector for rendering
        std::vector<vec3> colors;
        std::pair<EdgeIteratorGTGraph , EdgeIteratorGTGraph> ep = edges(*skeleton_draw);
        for (EdgeIteratorGTGraph eIter = ep.first; eIter != ep.second; ++eIter)
        {
            vec3 color = {0.5, 0.5, 0.5};
            // set main color
            if ((*skeleton_draw)[*eIter].is_main){
                color = {0.2, 0.2, 1};
            }
            colors.push_back(color);
            colors.push_back(color);
        }

        graph_drawable->update_color_buffer(colors);
        graph_drawable->set_property_coloring(State::VERTEX);
    }

    return true;
}


bool GrowthViewer::show_bifur(SkeletonType type, int ts_index, bool show, ImVec4 default_color){
    // if importance visible, set colors back to black
    if (!show){
        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->set_uniform_coloring(
                vec4(default_color.x, default_color.y, default_color.z, default_color.w));
    } else { // if importance not visible, color vertices
        const GraphGT* skeleton_draw = skeleton_ts(type, ts_index);

        //create the vertices vector for rendering
        std::vector<vec3> points;
        std::vector<vec3> colors;

        std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(*skeleton_draw);
        for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
            vec3 color;
            // set deleted vertex color to almost white
            if ((*skeleton_draw)[*vit].weight == -1 || (*skeleton_draw)[*vit].delete_mark){
                color = {0.85, 0.85, 1};
            } else {
                if ((*skeleton_draw)[*vit].is_lobe_node_ts){
                    color = {1, 0, 0};
                }
                else if ((*skeleton_draw)[*vit].is_lobe_node_corr){
                    color = {1, 0, 1};
                } else if ((*skeleton_draw)[*vit].is_main){
                    color = {0, 0, 1};  // mark main points
                }
                else {
                    color = {0.7, 0.7, 1};
                }
            }
            colors.push_back(color);

            points.push_back((*skeleton_draw)[*vit].coords);
        }

        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->update_vertex_buffer(points);
        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->update_color_buffer(colors);
        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->set_property_coloring(State::VERTEX);
    }

    return true;
}


bool GrowthViewer::show_lobes(SkeletonType type, int ts_index, bool show, ImVec4 default_color){
    // if importance visible, set colors back to black
    if (!show){
        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->set_uniform_coloring(
                vec4(default_color.x, default_color.y, default_color.z, default_color.w));
    } else { // if importance not visible, color vertices
        const GraphGT* skeleton_draw = skeleton_ts(type, ts_index);

        //create the vertices vector for rendering
        std::vector<vec3> points;
        std::vector<vec3> colors;

        std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(*skeleton_draw);
        for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit){
            vec3 color;
            // set deleted vertex color to almost white
            if ((*skeleton_draw)[*vit].weight == -1 || (*skeleton_draw)[*vit].delete_mark){
                color = {0.85, 0.85, 1};
            } else {
                // vertex belongs to a lobe
                if ((*skeleton_draw)[*vit].lobe_index != -1){
                    float intensity = float((*skeleton_draw)[*vit].lobe_index) / float(trees_[ts_index]->get_num_lobes());
                    color = colormap(intensity);

                    // todo: perhaps color connector nodes differently?
                    /*// main node lobe is connected to
                    if ((*skeleton_draw)[*vit].is_lobe_node){

                    }
                    // points inside lobe
                    else {

                    }*/
                }
                    // vertex is not part of a lobe
                else {
                    color = {0, 0, 1};
                }
            }
            colors.push_back(color);

            points.push_back((*skeleton_draw)[*vit].coords);
        }

        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->update_vertex_buffer(points);
        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->update_color_buffer(colors);
        cloud_ts(ts_index)->renderer()->get_points_drawable("vertices")->set_property_coloring(State::VERTEX);
    }


    return true;
}


bool GrowthViewer::create_lobe_skeleton_drawable(SkeletonType type, int ts_index){
    auto lobes = gtree_->get_lobes()[ts_index];

    std::vector<vec3> graph_points;

    for (auto lobe : lobes.second){
//        const GraphGT* skeleton_draw = skeleton_ts(type, ts_index);
        const GraphGT* skeleton_draw = &(lobe->get_subgraph());

        VertexDescriptorGTGraph dVertex1, dVertex2;
        vec3 pVertex1, pVertex2;
        std::pair<EdgeIteratorGTGraph , EdgeIteratorGTGraph> ep = edges(*skeleton_draw);
        for (EdgeIteratorGTGraph eIter = ep.first; eIter != ep.second; ++eIter){
            // store selected vertices
            dVertex1 = source(*eIter, *skeleton_draw);
            dVertex2 = target(*eIter, *skeleton_draw);
            pVertex1 = (*skeleton_draw)[dVertex1].coords;
            pVertex2 = (*skeleton_draw)[dVertex2].coords;
            assert(!has_nan(pVertex1));
            assert(!has_nan(pVertex2));

            graph_points.push_back(pVertex1);
            graph_points.push_back(pVertex2);
        }
    }

    // create main skeleton drawable
    LinesDrawable* graph_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("lobes");
    if (!graph_drawable)
        graph_drawable = cloud_ts(ts_index)->renderer()->add_lines_drawable("lobes");
    graph_drawable->update_vertex_buffer(graph_points);

    return true;
}


bool GrowthViewer::show_skeleton_distance(SkeletonType type, int ts_index, bool show, ImVec4 default_color){
    LinesDrawable* graph_drawable = cloud_ts(ts_index)->renderer()->get_lines_drawable("graph");

    // if importance visible, set colors back to black
    if (!show){
        graph_drawable->set_uniform_coloring(
                vec4(default_color.x, default_color.y, default_color.z, default_color.w));
    } else { // if importance not visible, color edges
        const GraphGT* skeleton_draw = skeleton_ts(type, ts_index);

        //create the vertices vector for rendering
        std::vector<vec3> colors;
        std::pair<EdgeIteratorGTGraph , EdgeIteratorGTGraph> ep = edges(*skeleton_draw);
        for (EdgeIteratorGTGraph eIter = ep.first; eIter != ep.second; ++eIter)
        {
            vec3 color = {0.9, 0.9, 0.9};
            // set main color
            if ((*skeleton_draw)[*eIter].is_main){
                double max_vis = 0.1; // trees_[ts_index]->get_max_skeleton_distance();
                float intensity = 0;
                if ((*skeleton_draw)[*eIter].distance < max_vis)
                    intensity = 1 - (*skeleton_draw)[*eIter].distance / max_vis; // inverse to get green for low
                color = colormap(intensity);
            }
            colors.push_back(color);
            colors.push_back(color);

            // todo: draw lines between the edges? so we can see if they correspond correctly?
        }

        graph_drawable->update_color_buffer(colors);
        graph_drawable->set_property_coloring(State::VERTEX);
    }

    return true;
}


vec3 GrowthViewer::colormap(float intensity){
    // todo: do rainbow instead of red-->green scale?
    float red = 0;
    float green = 0;
    float blue = 0;

    if (intensity < 0.5){
        red = 1;
        // increase green
        green = intensity * 2;
    } else if (intensity > 0.5){
        green = 1;
        // decrease red
        red = 1 - ((intensity - 0.5) * 2);
    } else {
        red = 1;
        green = 1;
    }

    float brightness = 0.9;
    red *= brightness;
    green *= brightness;
    blue *= brightness;

    return {red, green, blue};
}


bool GrowthViewer::create_inter_skeleton_drawable(int item_index){
    const GraphGT* skeleton_draw = skeleton_ts(static_cast<SkeletonType>(4), item_index);
    const GraphGT* skeleton_draw_next = skeleton_ts(static_cast<SkeletonType>(4), item_index + 1);

    //create the vertices vector for rendering
    std::vector<vec3> colors_e;
    std::vector<vec3> graph_points;
    std::vector<vec3> points_0;
    std::vector<vec3> points_1;
    std::vector<vec3> colors_v_0;
    std::vector<vec3> colors_v_1;
    std::vector<vec3> main_points_0;
    std::vector<vec3> main_points_1;

    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vt = vertices(*skeleton_draw);
    VertexDescriptorGTGraph dVertex1, dVertex2;
    vec3 pVertex1, pVertex2;
    for (VertexIteratorGTGraph vit = vt.first; vit != vt.second; ++vit) {
        // find qualifying source verts
        if ((*skeleton_draw)[*vit].is_main && !((*skeleton_draw)[*vit].delete_mark)){
            // store selected vertices
            dVertex1 = *vit;
            pVertex1 = (*skeleton_draw)[dVertex1].coords;
            assert(!has_nan(pVertex1));

            points_0.push_back(pVertex1);

            // vertex has target correspondence
            if ((*skeleton_draw)[*vit].inter_target != -1){
                // vertex colors
                colors_e.emplace_back(0, 0, 1);
                colors_e.emplace_back(0, 0, 1);

                // store corresponding vertex
                dVertex2 = (*skeleton_draw)[*vit].inter_target;
                pVertex2 = (*skeleton_draw_next)[dVertex2].coords;
                assert(!has_nan(pVertex2));

                // add points to graph, target point to drawn point cloud
                graph_points.push_back(pVertex1);
                graph_points.push_back(pVertex2);
                points_1.push_back(pVertex2);

                // add source and target colors
                colors_v_0.emplace_back(0, 1, 0);
                if ((*skeleton_draw_next)[dVertex2].corr_tip){
                    colors_v_1.emplace_back(1, 0, 1);
                }
                else if ((*skeleton_draw_next)[*vit].inter_bifur_mark_temp){
                    colors_v_1.emplace_back(0.2, 0.2, 1);
                }
                else if ((*skeleton_draw_next)[dVertex2].inter_bifur_mark) {
                    colors_v_1.emplace_back(0.7, 0.7, 1);
                }
                else {
                    colors_v_1.emplace_back(1, 0, 0);
                }
            }
            // vertex is to be deleted (no vertex correspondence)
            else {
                colors_v_0.emplace_back(0.7, 0.8, 0.7);
            }
        }
    }

    // also add to be added vertices of target to visualisation
    std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vtt = vertices(*skeleton_draw_next);
    for (VertexIteratorGTGraph vit = vtt.first; vit != vtt.second; ++vit) {
        if ((*skeleton_draw_next)[*vit].is_main && !((*skeleton_draw_next)[*vit].delete_mark)){
            if ((*skeleton_draw_next)[*vit].inter_add_mark){
                // store corresponding vertex
                pVertex2 = (*skeleton_draw_next)[*vit].coords;
                assert(!has_nan(pVertex2));
                points_1.push_back(pVertex2);

                // store color
                if ((*skeleton_draw_next)[*vit].inter_bifur_mark_temp) {
                    colors_v_1.emplace_back(0.2, 0.2, 1);
                } else if ((*skeleton_draw_next)[*vit].inter_bifur_mark){
                    colors_v_1.emplace_back(0.7, 0.7, 1);
                }

                else {
                    colors_v_1.emplace_back(0.8, 0.7, 0.7);
                }
            }
        }
    }

    // add lines of base skeleton
    std::pair<EdgeIteratorGTGraph , EdgeIteratorGTGraph> eb = edges(*skeleton_draw);
    for (EdgeIteratorGTGraph eIter = eb.first; eIter != eb.second; ++eIter)
    {
        main_points_0.push_back((*skeleton_draw)[source(*eIter, *skeleton_draw)].coords);
        main_points_0.push_back((*skeleton_draw)[target(*eIter, *skeleton_draw)].coords);
    }

    // add lines of target skeleton
    std::pair<EdgeIteratorGTGraph , EdgeIteratorGTGraph> et = edges(*skeleton_draw_next);
    for (EdgeIteratorGTGraph eIter = et.first; eIter != et.second; ++eIter)
    {
        main_points_1.push_back((*skeleton_draw_next)[source(*eIter, *skeleton_draw_next)].coords);
        main_points_1.push_back((*skeleton_draw_next)[target(*eIter, *skeleton_draw_next)].coords);
    }

    // create correspondence lines drawable
    LinesDrawable* graph_drawable = cloud_ts(item_index)->renderer()->get_lines_drawable("inter");
    if (!graph_drawable)
        graph_drawable = cloud_ts(item_index)->renderer()->add_lines_drawable("inter");

    graph_drawable->update_color_buffer(colors_e);
    graph_drawable->update_vertex_buffer(graph_points);
    graph_drawable->set_property_coloring(State::VERTEX);

    // create main skeleton drawable
    LinesDrawable* skel_drawable_base = cloud_ts(item_index)->renderer()->get_lines_drawable("graph");
    LinesDrawable* skel_drawable_target = cloud_ts(item_index + 1)->renderer()->get_lines_drawable("graph");
    if (!skel_drawable_base)
        skel_drawable_base = cloud_ts(item_index)->renderer()->add_lines_drawable("graph");
    if (!skel_drawable_target)
        skel_drawable_target = cloud_ts(item_index + 1)->renderer()->add_lines_drawable("graph");

    skel_drawable_base->update_vertex_buffer(main_points_0);
    skel_drawable_base->set_uniform_coloring(vec4(0.2, 0.7, 0.2, 1.0));

    skel_drawable_target->update_vertex_buffer(main_points_1);
    skel_drawable_target->set_uniform_coloring(vec4(1, 0.5, 0.5, 1.0));

    // draw vertices as well
    cloud_ts(item_index)->renderer()->get_points_drawable("vertices")->update_vertex_buffer(points_0);
    cloud_ts(item_index + 1)->renderer()->get_points_drawable("vertices")->update_vertex_buffer(points_1);

    cloud_ts(item_index)->renderer()->get_points_drawable("vertices")->update_color_buffer(colors_v_0);
    cloud_ts(item_index + 1)->renderer()->get_points_drawable("vertices")->update_color_buffer(colors_v_1);

    cloud_ts(item_index)->renderer()->get_points_drawable("vertices")->set_property_coloring(State::VERTEX);
    cloud_ts(item_index + 1)->renderer()->get_points_drawable("vertices")->set_property_coloring(State::VERTEX);

    return true;
}


/*-------------------------------------------------------------*/
/*----------------------------IN-------------------------------*/
/*-------------------------------------------------------------*/

bool GrowthViewer::open()
{
    for (auto m : models_)
        delete m;
    models_.clear();

    const std::vector<std::string> filetypes = {"*.xyz"};
    const std::vector<std::string>& file_names = dialog::open("open files", std::string(""), filetypes, true);

    int count = 0;
    for (auto const& file_name : file_names) {
        if (!file_name.empty()) {
            add_model(file_name);
            set_title("GTree - " + file_system::simple_name(cloud_ts(count)->name()));
            fit_screen();
            ++count;
        }
    }
    return count > 0;
}


bool GrowthViewer::open_mesh()
{
    for (auto m : models_)
        delete m;
    models_.clear();

    const std::vector<std::string> filetypes = {"*.obj"};
    const std::string& file_name = dialog::open("open file", std::string(""), filetypes);

    std::cout << "generated name" << std::endl;

    if (!file_name.empty()) {
        std::cout << "opened" << std::endl;
        set_title("GTree branches");
        add_model(file_name);
        fit_screen();
        return true;
    }
    return false;

}


std::string GrowthViewer::open_multitemporal(){
    const std::vector<std::string> filetypes = {"*.xyz"};
    std::string file_name = dialog::open("open files", std::string(""), filetypes);

    return file_name;
}


bool GrowthViewer::complete_multitemporal_import(std::vector<std::string> filenames){
    for (auto m : models_)
        delete m;
    models_.clear();

    // set to translate all imported models with first point (default is no translation)
    Translator::instance()->set_status(Translator::TRANSLATE_USE_FIRST_POINT);

    int count = 0;
    for (auto const& file_name : filenames) {
        if (!file_name.empty()) {
            add_model(file_name);
            set_title("GTree - " + file_system::simple_name(cloud_ts(count)->name()));
            fit_screen();

            PointCloud *cloud_curr = cloud_ts(count);

            PointCloud::ModelProperty<Vec<3, double> > offset_first = cloud_ts(0)->get_model_property<dvec3>("translation");
            PointCloud::ModelProperty<Vec<3, double> > offset_curr = cloud_curr->get_model_property<dvec3>("translation");

            if (count > 0) {
                Vec<3, double> offset_relative = (-offset_first[0]) - (-offset_curr[0]);

                PointCloud::VertexProperty<Vec<3, float> > points = cloud_curr->get_vertex_property<vec3>("v:point");
                std::vector<vec3> coords;
                for (auto v : cloud_curr->vertices()){
                    points[v] = points[v] + static_cast<const Vec<3, float>>(offset_relative);
                    coords.emplace_back(points[v].x, points[v].y, points[v].z);
                }
                // update drawn vertices
                cloud_curr->renderer()->get_points_drawable("vertices")->update_vertex_buffer(coords);
                // update stored translation
                PointCloud::ModelProperty<Vec<3, double> > tp = cloud_curr->model_property<dvec3>("translation");
                tp[0] = offset_first[0];
            }
            ++count;
        }
    }

    std::cout << "loaded all xyz files" << std::endl;
    std::cout << "nr models: " << models_.size() << std::endl;

    return count > 0;
}


/*-------------------------------------------------------------*/
/*---------------------------OUT-------------------------------*/
/*-------------------------------------------------------------*/

void GrowthViewer::export_skeleton() const {
    // export merged & merged main
    GSkeleton* skeleton_merged = gtree_->get_merged_skeleton();
    const GraphGT& graph_merged = skeleton_merged->get_simplified();
    const GraphGT& graph_main = skeleton_merged->get_corresponding();
    vec3 trans = skeleton_merged->get_translation();
    const std::string& initial_name_merged = file_system::base_name(cloud_ts(0)->name()) + "_skeleton_merged.ply";
    const std::string& initial_name_main = file_system::base_name(cloud_ts(0)->name()) + "_skeleton_merged_main.ply";

    export_graph(graph_merged, trans, initial_name_merged);
    export_graph(graph_main, trans, initial_name_main);
}


void GrowthViewer::export_main() const {
    // export original ts skeletonizations
    for (int tree_idx = 0; tree_idx < (trees_.size() - 1); ++tree_idx){
        const GraphGT& skeleton = trees_[tree_idx]->get_ts_main();
        vec3 trans = trees_[tree_idx]->get_translation();
        const std::string& initial_name = file_system::base_name(cloud_ts(tree_idx)->name()) + "_skeleton_tsmain.ply";

        export_graph(skeleton, trans, initial_name);
    }
}


void GrowthViewer::export_graph(const GraphGT& skeleton, vec3 translation, const std::string& initial_name) const {
    if (boost::num_edges(skeleton) == 0){
        std::cerr << "skeleton has 0 edges" << std::endl;
        return;
    }

    const std::vector<std::string> filetypes = {"*.ply"};
    const std::string& file_name = dialog::save("save file", initial_name, filetypes);
    if (file_name.empty())
        return;

    // convert the boost graph to Graph (avoid modifying easy3d's GraphIO, or writing IO for boost graph)
    std::vector<vec3> vertices;
    std::vector<std::tuple<int, int>> edges;
    std::map<int,int> off_map;
    int off_value = 0;

    auto vts = boost::vertices(skeleton);
    for (VertexIteratorGTGraph iter = vts.first; iter != vts.second; ++iter) {
        int vd = *iter;
        if (boost::degree(vd, skeleton) != 0) { // ignore isolated vertices
            vertices.emplace_back(skeleton[vd].coords);
            off_map.insert({vd, off_value});
        } else {
            off_value ++;
        }
    }

    auto egs = boost::edges(skeleton);
    for (EdgeIteratorGTGraph iter = egs.first; iter != egs.second; ++iter) {
        int s_b = boost::source(*iter, skeleton);
        int t_b = boost::target(*iter, skeleton);

        std::tuple<int, int> i = {s_b - off_map[s_b], t_b - off_map[t_b]};
        edges.emplace_back(i);
    }

    std::ofstream storageFile;
    storageFile.open(file_name);

    // write header
    storageFile << "ply" << std::endl;
    storageFile << "format ascii 1.0" << std::endl;
    storageFile << "element vertex " << vertices.size() << std::endl;
    storageFile << "property float x" << std::endl;
    storageFile << "property float y" << std::endl;
    storageFile << "property float z" << std::endl;
    storageFile << "element edge " << edges.size() << std::endl;
    storageFile << "property int vertex1" << std::endl;
    storageFile << "property int vertex2" << std::endl;
    storageFile << "end_header" << std::endl << std::endl;

    // write vertices
    for (auto &vertex : vertices) {
        for (int i = 0; i < 3; ++i) {
            // allow for larger values being written to avoid rounding
            vertex[i] = ((float) std::roundf((vertex[i] + translation[i])*1000))/1000;
            storageFile << std::setprecision(std::to_string((int) vertex[i]).length() + 3);
            storageFile << vertex[i] << " ";
            storageFile << std::setprecision(-1);
        }
        storageFile << std::endl;
    }

    // write edges
    storageFile << std::endl;
    for (const auto& edge: edges) {
        storageFile << std::get<0>(edge) << " " << std::get<1>(edge) << std::endl;
    }

    storageFile.close();


    size_t last_slash = file_name.find_last_of("/\\");
    auto name_simple = file_name.substr(last_slash + 1, file_name.size());

    std::cout << "Wrote skeleton to file: " << name_simple << std::endl;
}


void GrowthViewer::export_mesh(const SurfaceMesh* mesh, const std::string& initial_name) const {
    if (mesh->n_vertices() == 0 || mesh->n_edges() == 0 || mesh->n_faces() == 0){
        std::cout << "ERROR: could not export mesh, mesh does not contain elements.";
        return;
    }

    const std::vector<std::string> filetypes = {"*.obj"};
    const std::string& file_name = dialog::save("save file", mesh->name(), filetypes);
    if (file_name.empty())
        return;

    std::ofstream out;
    out.open(file_name);

    SurfaceMesh::VertexProperty<vec3> points = mesh->get_vertex_property<vec3>("v:point");
    for (SurfaceMesh::VertexIterator vit = mesh->vertices_begin(); vit != mesh->vertices_end(); ++vit) {
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

    for (SurfaceMesh::FaceIterator fit = mesh->faces_begin(); fit != mesh->faces_end(); ++fit) {
        out << "f ";
        SurfaceMesh::VertexAroundFaceCirculator fvit = mesh->vertices(*fit), fvend = fvit;
        SurfaceMesh::HalfedgeAroundFaceCirculator fhit = mesh->halfedges(*fit);
        do {
            // write vertex index and normal index
            out << (*fvit).idx() + 1 << " ";
        } while (++fvit != fvend);
        out << "\n";
    }

    out.close();

    size_t last_slash = file_name.find_last_of("/\\");
    auto name_simple = file_name.substr(last_slash + 1, file_name.size());

    std::cout << "Wrote mesh to file: " << name_simple << std::endl;
}


void GrowthViewer::export_lobes() const {
    for (SurfaceMesh* mesh : gtree_->get_lobe_meshes()){
        if (!mesh) {
            std::cerr << "model of lobes does not exist" << std::endl;
            return;
        }

        const std::string& initial_name = mesh->name();

        export_mesh(mesh, initial_name);
    }
}


void GrowthViewer::export_branches_corr() const {
    for (int ts_idx = 0; ts_idx < (trees_.size() - 1); ++ts_idx){
        SurfaceMesh* mesh = branches_ts(ts_idx, 3);
        if (!mesh) {
            std::cerr << "model of branches does not exist" << std::endl;
            return;
        }

        const std::string& initial_name = mesh->name();

        export_mesh(mesh,  initial_name);
    }
}


void GrowthViewer::export_branches_ts() const {
    for (int ts_idx = 0; ts_idx < (trees_.size() - 1); ++ts_idx){
        SurfaceMesh* mesh = branches_ts(ts_idx, 4);
        if (!mesh) {
            std::cerr << "model of branches does not exist" << std::endl;
            return;
        }

        const std::string& initial_name = mesh->name();

        export_mesh(mesh,  initial_name);
    }
}


void GrowthViewer::export_correspondences() const {
    //--- save mapping graph index <> file index
    std::vector<std::map<unsigned int, unsigned int> > idx_maps; // graph indices <> index as written to file, for each ts

    //--- export original ts skeletonizations
    for (int tree_idx = 0; tree_idx < (trees_.size() - 1); ++tree_idx){
        const GraphGT& skeleton = trees_[tree_idx]->get_ts_main();
        vec3 trans = trees_[tree_idx]->get_translation();
        const std::string& initial_name = file_system::base_name(cloud_ts(tree_idx)->name()) + "_skeleton_tsmaincorr.ply";

        if (boost::num_edges(skeleton) == 0){
            std::cerr << "skeleton has 0 edges" << std::endl;
            return;
        }

        const std::vector<std::string> filetypes = {"*.ply"};
        const std::string& file_name = dialog::save("save file", initial_name, filetypes);
        if (file_name.empty())
            return;

        // convert the boost graph to Graph (avoid modifying easy3d's GraphIO, or writing IO for boost graph)
        std::vector<vec4> vertices;
        std::vector<std::tuple<int, int>> edges;
        std::map<int,int> off_map;
        int off_value = 0;
        std::map<unsigned int, unsigned int> idx_map;

        auto vts = boost::vertices(skeleton);
        for (VertexIteratorGTGraph iter = vts.first; iter != vts.second; ++iter) {
            int vd = *iter;
            if (!skeleton[vd].delete_mark) { // ignore deleted vertices
                vertices.emplace_back(skeleton[vd].coords, vd);
                off_map.insert({vd, off_value});
                idx_map[vd] = vd - off_value;
            } else {
                off_value ++;
            }
        }

        auto egs = boost::edges(skeleton);
        for (EdgeIteratorGTGraph iter = egs.first; iter != egs.second; ++iter) {
            int s_b = boost::source(*iter, skeleton);
            int t_b = boost::target(*iter, skeleton);

            std::tuple<int, int> i = {s_b - off_map[s_b], t_b - off_map[t_b]};
            edges.emplace_back(i);

        }

        // store index map
        idx_maps.push_back(idx_map);

        std::ofstream storageFile;
        storageFile.open(file_name);

        // write header
        storageFile << "ply" << std::endl;
        storageFile << "format ascii 1.0" << std::endl;
        storageFile << "element vertex " << vertices.size() << std::endl;
        storageFile << "property float x" << std::endl;
        storageFile << "property float y" << std::endl;
        storageFile << "property float z" << std::endl;
        storageFile << "property float i" << std::endl;
        storageFile << "element edge " << edges.size() << std::endl;
        storageFile << "property int vertex1" << std::endl;
        storageFile << "property int vertex2" << std::endl;
        storageFile << "end_header" << std::endl << std::endl;

        // write vertices
        for (auto &vertex : vertices) {
            for (int i = 0; i < 4; ++i) {
                // allow for larger values being written to avoid rounding
                vertex[i] = ((float) std::roundf((vertex[i] + trans[i])*1000))/1000;
                storageFile << std::setprecision(std::to_string((int) vertex[i]).length() + 3);
                storageFile << vertex[i] << " ";
                storageFile << std::setprecision(-1);
            }
            storageFile << std::endl;
        }

        // write edges
        storageFile << std::endl;
        for (const auto& edge: edges) {
            storageFile << std::get<0>(edge) << " " << std::get<1>(edge) << std::endl;
        }

        storageFile.close();
        std::cout << "skeleton file stored" <<std::endl;

        std::cout << "nr verts skel " << tree_idx << ": " << boost::num_vertices(skeleton)
        << ", num verts written: " << idx_map.size()
        << ", num edges: " << boost::num_edges(skeleton) << std::endl;
    }

    //--- export correspondences to files
    for (int tree_idx = 0; tree_idx < (trees_.size() - 2); ++tree_idx){
        const GraphGT& skeleton_base = trees_[tree_idx]->get_ts_main();
        const GraphGT& skeleton_target = trees_[tree_idx + 1]->get_ts_main();

        const std::string& initial_name_v = "corr_vertices_" + std::to_string(tree_idx) + ".csv";
        const std::string& initial_name_e = "corr_edges_" + std::to_string(tree_idx) + ".csv";
        const std::vector<std::string> filetypes = {"*.csv"};
        const std::string& file_name_v = dialog::save("save file", initial_name_v, filetypes);
        const std::string& file_name_e = dialog::save("save file", initial_name_e, filetypes);
        if (file_name_v.empty() || file_name_e.empty())
            return;


        //--- write to vertices file
        std::ofstream fout_v;
        fout_v.open(file_name_v);

        // write header
        fout_v << "v0,v1" << std::endl;

        // write vertices
        std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vtb = boost::vertices(skeleton_base);
        for (VertexIteratorGTGraph vit = vtb.first; vit != vtb.second; ++vit) {
            if (!skeleton_base[*vit].delete_mark) {  // ignore deleted vertices
                // corresponding base vertices
                if (skeleton_base[*vit].inter_target != -1) {
                    fout_v << idx_maps[tree_idx][*vit] << ","
                           << idx_maps[tree_idx + 1][skeleton_base[*vit].inter_target] << std::endl;
                }
                // deleted base vertices
                else {
                    fout_v << idx_maps[tree_idx][*vit] << "," << std::endl;
                }
            }
        }

        std::pair<VertexIteratorGTGraph, VertexIteratorGTGraph> vtt = boost::vertices(skeleton_target);
        for (VertexIteratorGTGraph vit = vtt.first; vit != vtt.second; ++vit) {
            if (!skeleton_target[*vit].delete_mark) { // ignore deleted vertices

                // added target vertices
                if (skeleton_target[*vit].inter_base == -1) {
                    fout_v << "," << idx_maps[tree_idx + 1][*vit] << std::endl;
                }

//                // additional vertex inserted for collapsing base vertices
//                if (skeleton_target[*vit].inter_merged != -1){
//                    VertexDescriptorGTGraph to_coll_to = skeleton_target[*vit].inter_merged;
//                    std::cout << "vertex " << *vit << " was inserted to collapse to " << to_coll_to << std::endl;
//                    std::cout << "\tin file: " << idx_maps[tree_idx + 1][*vit]  << " to "
//                              << idx_maps[tree_idx + 1][idx_maps[tree_idx + 1][to_coll_to]] << std::endl;
//                }
            }
        }

        fout_v.close();
        std::cout << "vert corr file stored" <<std::endl;

        //--- write to edges file
        std::ofstream fout_e;
        fout_e.open(file_name_e);

        // write header
        fout_e << "e0v0,e0v1,e1v0,e1v1" << std::endl;

        // write edges
        // base deletion edges (must be first!)
        std::pair<EdgeIteratorGTGraph, EdgeIteratorGTGraph> edb = boost::edges(skeleton_base);
        for (EdgeIteratorGTGraph eit = edb.first; eit != edb.second; ++eit) {
            VertexDescriptorGTGraph vs_b = boost::source(*eit, skeleton_base);
            VertexDescriptorGTGraph vt_b = boost::target(*eit, skeleton_base);

            // mark to delete if not corresponds
            if ((skeleton_base)[*eit].inter_delete_mark){
                fout_e << idx_maps[tree_idx][vs_b] << ","
                       << idx_maps[tree_idx][vt_b] << ","
                       << ","
                       << std::endl;
            }
        }

        // base transformation edges for edges that do NOT exist in target as well
        for (EdgeIteratorGTGraph eit = edb.first; eit != edb.second; ++eit) {
            VertexDescriptorGTGraph vs_b = boost::source(*eit, skeleton_base);
            VertexDescriptorGTGraph vt_b = boost::target(*eit, skeleton_base);

            // if corresponds, mark transformation
            if (!(skeleton_base)[*eit].inter_delete_mark){
                VertexDescriptorGTGraph vs_t = skeleton_base[vs_b].inter_target;
                VertexDescriptorGTGraph vt_t = skeleton_base[vt_b].inter_target;

                fout_e << idx_maps[tree_idx][vs_b] << ","
                       << idx_maps[tree_idx][vt_b] << ","
                       << idx_maps[tree_idx + 1][vs_t] << ","
                       << idx_maps[tree_idx + 1][vt_t] << std::endl;
            }
        }

        // add target edges (to add) as well
        std::pair<EdgeIteratorGTGraph, EdgeIteratorGTGraph> edt = boost::edges(skeleton_target);
        for (EdgeIteratorGTGraph eit = edt.first; eit != edt.second; ++eit) {
            VertexDescriptorGTGraph vs_t = boost::source(*eit, skeleton_target);
            VertexDescriptorGTGraph vt_t = boost::target(*eit, skeleton_target);

            if (skeleton_target[*eit].inter_add_mark){
                fout_e << ",,"
                       << idx_maps[tree_idx + 1][vs_t] << ","
                       << idx_maps[tree_idx + 1][vt_t] << std::endl;
            }
        }

        fout_e.close();
        std::cout << "edge corr file stored" <<std::endl;
    }
}


/*-------------------------------------------------------------*/
/*-----------------------RECONSTRUCT---------------------------*/
/*-------------------------------------------------------------*/

bool GrowthViewer::reconstruct_multitemporal(){
    // todo: check all models are really point clouds?

    // todo: better check if model has already been reconstructed

    std::cout << "\nReconstructing all timestamps" << std::endl;

    for (int midx = 0; midx < models().size(); ++midx){
        if (midx >= trees_.size()){
            std::cout << "Adding model " << midx << std::endl;

            Model* m = models_[midx];
            PointCloud *cloud_curr = dynamic_cast<PointCloud *>(m);

            // remove duplicate points
            const float threshold = cloud_curr->bounding_box().diagonal_length() * 0.001f;
            std::cout << "\tremoving duplicate vertices with threshold: " << threshold << "... ";

            std::vector<PointCloud::Vertex> points_to_remove;
            const int maxBucketSize = 16;
            std::vector<vec3>& points = cloud_ts(0)->points();
            float* pointer = points[0];
            KdTree kd(reinterpret_cast<Vector3D*>(pointer), points.size(), maxBucketSize);

            std::vector<bool> keep(cloud_ts(0)->vertices_size(), true);

            double sqr_dist = threshold * threshold;
            for (std::size_t i = 0; i < points.size(); ++i) {
                if (keep[i]) {
                    const vec3 &p = points[i];
                    kd.queryRange(Vector3D(p.x, p.y, p.z), sqr_dist, true);
                    int num = kd.getNOfFoundNeighbours();
                    if (num > 1) {
                        for (int j = 1; j < num; ++j) {
                            int idx = kd.getNeighbourPositionIndex(j);
                            keep[idx] = 0;
                        }
                    }
                }
            }

            for (std::size_t i = 0; i < keep.size(); ++i) {
                if (!keep[i])
                    points_to_remove.push_back(PointCloud::Vertex(i));
            }

            for (auto v : points_to_remove)
                cloud_curr->delete_vertex(v);
            cloud_curr->collect_garbage();
            cloud_curr->renderer()->get_points_drawable("vertices")->update_vertex_buffer(cloud_curr->points());
            std::cout << cloud_curr->vertices_size() << " points remained" << std::endl;

            // initialize/clean skeleton and branches that will be drawn
            GSkeleton* skel_curr = new GSkeleton;

            SurfaceMesh* mesh = dynamic_cast<SurfaceMesh*>(m);
            if (mesh)
                mesh->clear();
            else {
                mesh = new SurfaceMesh;
            }

            auto offset = cloud_curr->get_model_property<dvec3>("translation");
            if (offset) {
                easy3d::vec3 translation = {(float) offset[0][0], (float) offset[0][1], (float) offset[0][2]};
                skel_curr->set_translation(translation);
                auto prop = mesh->model_property<dvec3>("translation");
                prop[0] = offset[0];
            }

            // reconstruct skeleton/branches
            bool status = skel_curr->reconstruct_skeleton(cloud_curr, mesh);

            if (status) {
                trees_.push_back(skel_curr);

                // if merged model was added, update the skeleton of the gtree
                if (gtree_){
                    std::cout << "storing merged skeleton" << std::endl;
                    gtree_->add_merged_skeleton(skel_curr);
                }
            }
        } else {
            std::cout << "Model " << midx << " already exists, moving on to next model";
            std::cout << " (trees_ size: " << trees_.size() << ")" << std::endl;
        }
    }

    std::cout << "Reconstruction complete." << std::endl;
    std::cout << "\tnumber of skeletons added: " << trees_.size() << std::endl;
    std::cout << "\tnumber of models, end: " << models_.size() << std::endl;

    return true;
}


bool GrowthViewer::add_merged_cloud(){
    // check if trees_ has data (reconstruction took place)
    if (trees_.empty()){
        std::cout << "ERROR: could not model growth, please reconstruct skeletons first." << std::endl;
        return false;
    }
    if (gtree_){
        delete gtree_;
    }

    // add reconstructed skeletons to growth model class (GTree)
    gtree_ = new GTree;
    gtree_->add_ts_skeletons(trees_);

    // create merged skeleton/point cloud
    gtree_->construct_merged_cloud();
    // add merged skeleton/point cloud to viewer
    PointCloud* cloud_merged = gtree_->get_merged_cloud();

    cloud_merged->set_name("merged_timestamps");
    add_model(cloud_merged);

    std::cout << "done adding merged point cloud, nr. merged timestamps: " << gtree_->get_num_timestamps() << std::endl;

    return true;
}


bool GrowthViewer::model_correspondence(){
    std::cout << "\nModelling growth" << std::endl;

    if (!gtree_->get_merged_skeleton()){
        std::cout << "GTree does not have skeleton. Starting reconstruction." << std::endl;
        reconstruct_multitemporal();
    }

    // store number of corresponding edges
    gtree_->compute_correspondence();
    // keep/complete/process corresponding edge sub-skeleton
    gtree_->compute_main_edges();
    // constrain main skeletons of timestamps with correspondence merged main skeleton
    gtree_->constrain_ts_skeletons();

    gtree_->compute_lobe_hulls();

    // check success of lobe mesh construction
    if (gtree_->get_lobe_meshes().size() < (trees_.size() - 1)){
        std::cout << "ERROR adding lobe meshes: not enough lobe meshes were constructed" << std::endl;
        return false;
    }

    // check if old lobe meshes need to be deleted
    if (models().size() > trees_.size()){
        for (int ts_count = 0; ts_count < (trees_.size() - 1); ++ts_count){
            SurfaceMesh* mesh_old = lobe_ts(ts_count);
            if (mesh_old){
                mesh_old->clear();
            }
        }
    }


    // add generated lobe meshes
    // (size - 1 to exclude merged cloud, only take original timestamps)
    for (int mesh_idx = 0; mesh_idx < (trees_.size() - 1); ++mesh_idx){
        SurfaceMesh* mesh_curr = gtree_->get_lobe_meshes()[mesh_idx];

        // set translation
        auto offset = cloud_ts(mesh_idx)->get_model_property<dvec3>("translation");
        if (offset) {
            easy3d::vec3 translation = {(float) offset[0][0], (float) offset[0][1], (float) offset[0][2]};
            auto prop = mesh_curr->model_property<dvec3>("translation");
            prop[0] = offset[0];
        }
        // set name
        mesh_curr->set_name(file_system::base_name(cloud_ts(mesh_idx)->name()) + "_lobes.obj");

        add_model(mesh_curr);
        mesh_curr->renderer()->set_visible(false);  // for checkbox GUI access, unchecked by default

        // set mesh color
        auto lobes_drawable = mesh_curr->renderer()->get_triangles_drawable("surface");
        if (lobes_drawable) {
//            lobes_drawable->set_per_vertex_color(false);
            lobes_drawable->set_uniform_coloring(vec4(0.50f, 0.83f, 0.20f, 1.0f));
        }
    }


    return true;
}


bool GrowthViewer::model_growth(){
    // region growing in lobes
    gtree_->grow_lobes();

    return true;
}


bool GrowthViewer::model_interpolation(){
    // region growing in lobes
    gtree_->interpolate();

    return true;
}


bool GrowthViewer::reconstruct_geometry(){
    // init mesh, clear previous if exists
    // check if old meshes need to be deleted
    int idx_offset = trees_.size() + (trees_.size() - 1); // 7
    int idx_max = (4 * trees_.size() - 1) + 2; // 7 + 6 + 1 = 14
    int idx_plus = models().size() - idx_offset;
    if (models().size() > idx_offset){
        int cnt = 0;
        for (int ts_count = 0; ts_count < idx_plus; ++ts_count){
            SurfaceMesh* mesh_old;
            if (cnt > (trees_.size() - 2)){
                mesh_old = branches_ts(ts_count, 4);
            } else {
                mesh_old = branches_ts(ts_count, 3);
            }

            if (mesh_old){
                mesh_old->clear();
            }
            cnt++;
        }
    }

    // timestamps
    for (int time_idx = 0; time_idx < (trees_.size() - 1); ++time_idx) {
        std::cout << "adding branches for ts " << time_idx << std::endl;

        SurfaceMesh* mesh_corr = new SurfaceMesh;
        SurfaceMesh* mesh_ts = new SurfaceMesh;

        // do cylinder fitting
        GSkeleton* skel = trees_[time_idx];
        GraphGT* graph_corr = const_cast<GraphGT *>(skeleton_ts(static_cast<SkeletonType>(3), time_idx));
        GraphGT* graph_ts = const_cast<GraphGT *>(skeleton_ts(static_cast<SkeletonType>(4), time_idx));
        skel->reconstruct_branches(cloud_ts(time_idx), mesh_corr, graph_corr, skel->get_root_corr());
        skel->reconstruct_branches(cloud_ts(time_idx), mesh_ts, graph_ts, skel->get_root());
        // todo: add lobe skeletons

        // set translation
        auto offset = cloud_ts(time_idx)->get_model_property<dvec3>("translation");
        if (offset) {
            easy3d::vec3 translation = {(float) offset[0][0], (float) offset[0][1], (float) offset[0][2]};
            auto prop_corr = mesh_corr->model_property<dvec3>("translation");
            auto prop_ts = mesh_ts->model_property<dvec3>("translation");
            prop_corr[0] = offset[0];
            prop_ts[0] = offset[0];
        }
        // set name
        mesh_corr->set_name(file_system::base_name(cloud_ts(time_idx)->name()) + "_branches_corr.obj");
        mesh_ts->set_name(file_system::base_name(cloud_ts(time_idx)->name()) + "_branches_ts.obj");

        add_model(mesh_corr);
        add_model(mesh_ts);
        mesh_corr->renderer()->set_visible(false);  // for checkbox GUI access, unchecked by default
        mesh_ts->renderer()->set_visible(false);  // for checkbox GUI access, unchecked by default

        // set mesh color
        auto branches_drawable_corr = mesh_corr->renderer()->get_triangles_drawable("surface");
        auto branches_drawable_ts = mesh_corr->renderer()->get_triangles_drawable("surface");
        if (branches_drawable_corr) {
//            branches_drawable_corr->set_per_vertex_color(false);
            branches_drawable_corr->set_uniform_coloring(vec4(0.49f, 0.38f, 0.23f, 1.0f));
        }
        if (branches_drawable_ts) {
//            branches_drawable_ts->set_per_vertex_color(false);
            branches_drawable_ts->set_uniform_coloring(vec4(0.49f, 0.38f, 0.23f, 1.0f));
        }

        std::cout << "mesh added." << std::endl;
    }

    return true;
}


bool GrowthViewer::reconstruct_all(){
    if (models().empty()){
        std::cout << "ERROR: could not reconstruct, no models exist." << std::endl;
        return false;
    }

    // reconstruction
    reconstruct_multitemporal();

    // merged structure
    add_merged_cloud();

    // correspondence
    model_correspondence();

    // growth
    model_growth();

    // branches
    reconstruct_geometry();

    // interpolation
    model_interpolation();


    return true;
}





