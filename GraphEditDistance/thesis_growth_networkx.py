import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy as sp


def open_files(directory, filenames):
    data = []
    for filename in filenames:
        filename = directory + filename
        verts = []
        edges = []
        with open(filename) as fin:
            lines = fin.readlines()
            vertices_read = False
            for line in lines:
                # find start of edges part
                if line.startswith("\n") and len(verts) > 0:
                    vertices_read = True

                # read in lines with numbers
                if line[0].isnumeric():
                    # vertex
                    if not vertices_read:
                        v_line = line.split()
                        vert = np.array(v_line).astype(float)
                        verts.append(vert)
                    # edge
                    else:
                        v_line = line.split()
                        edge = np.array(v_line).astype(int)
                        edges.append(edge)

        num_verts = len(verts)
        num_edges = len(edges)
        verts = np.array(verts)
        data_verts = pd.DataFrame(verts, columns=["x", "y", "z"])
        edges = np.array(edges)
        data_edges = pd.DataFrame(edges, columns=["v0", "v1"])
        data.append((data_verts, data_edges))

        print(f"opened file {filename.split('/')[-1]}, num vertices: {num_verts}, num edges: {num_edges}")

    return data


def make_graphs(data):
    graphs = []
    for (data_verts, data_edges) in data:
        # add edges
        gx = nx.from_pandas_edgelist(data_edges, source='v0', target='v1')
        # add node properties (coordinates)
        for nd in gx.nodes:
            gx.add_node(nd, pos=(data_verts.iloc[nd]["x"], data_verts.iloc[nd]["y"]))
            for col in data_verts.columns:
                gx.nodes[nd][col] = data_verts.iloc[nd][col]
        graphs.append(gx)

    return graphs


def draw_graph(graphs):
    options = {'node_color': 'black',
               'node_size': 5,
               'width': 1}

    # todo: not variable size

    plt.figure(0)

    ax0 = plt.subplot(221)
    # pos0 = nx.get_node_attributes(graphs[0], 'pos')
    pos0 = nx.kamada_kawai_layout(graphs[0])
    nx.draw(graphs[0], pos0, **options)
    ax0.set_aspect('equal', 'box')

    ax1 = plt.subplot(222)
    # pos1 = nx.get_node_attributes(graphs[1], 'pos')
    pos1 = nx.kamada_kawai_layout(graphs[1])
    nx.draw(graphs[1], pos1, **options)
    ax1.set_aspect('equal', 'box')

    ax2 = plt.subplot(223)
    # pos2 = nx.get_node_attributes(graphs[2], 'pos')
    pos2 = nx.kamada_kawai_layout(graphs[2])
    nx.draw(graphs[2], pos2, **options)
    ax2.set_aspect('equal', 'box')

    ax3 = plt.subplot(224)
    # pos4 = nx.get_node_attributes(graphs[4], 'pos')
    pos4 = nx.kamada_kawai_layout(graphs[4])
    nx.draw(graphs[4], pos4, **options)
    ax3.set_aspect('equal', 'box')

    plt.figure(1)

    options1 = {'node_size': 5,
                'width': 2,
                'alpha': 0.6}
    ax_f1 = plt.gca()
    nx.draw(graphs[0], pos0, **options1, edge_color='#42BFDD', node_color='#42BFDD', label='ts0')
    nx.draw(graphs[1], pos1, **options1, edge_color='#A71BD5', node_color='#A71BD5', label='ts1')
    nx.draw(graphs[2], pos2, **options1, edge_color='#D39545', node_color='#D39545', label='ts2')
    ax_f1.set_aspect('equal')
    plt.legend()

    plt.figure(2)
    ax_f2 = plt.gca()
    ax_f2.set_aspect('equal')
    nx.draw(graphs[0], pos0, **options1, edge_color='#42BFDD', node_color='#42BFDD', label='ts0')
    nx.draw(graphs[4], pos4, **options1, edge_color='#D39545', node_color='#D39545', label='merged')
    plt.legend()

    plt.figure(3)
    ax_f3 = plt.gca()
    ax_f3.set_aspect('equal')
    nx.draw(graphs[1], pos1, **options1, edge_color='#42BFDD', node_color='#42BFDD', label='ts1')
    nx.draw(graphs[4], pos4, **options1, edge_color='#D39545', node_color='#D39545', label='merged')
    plt.legend()

    plt.figure(4)
    ax_f4 = plt.gca()
    ax_f4.set_aspect('equal')
    nx.draw(graphs[2], pos2, **options1, edge_color='#42BFDD', node_color='#42BFDD', label='ts2')
    nx.draw(graphs[4], pos4, **options1, edge_color='#D39545', node_color='#D39545', label='merged')
    plt.legend()

    plt.show()


def compute_distances(graphs):
    res = []
    nr_graphs = len(graphs)
    # graph_merged = graphs[(nr_graphs - 2)]
    # graph_main = graphs[(nr_graphs - 1)]
    # graph timestamps: graphs[0,nr_graphs]
    graph_main = graphs[4]
    idxs = [0, 1, 2]
    for i in idxs:
        print("calculating distance to graph ", i)
        graph_ts = graphs[i]
        dist_curr = nx.graph_edit_distance(graph_main, graph_ts, timeout=60)
        res.append(dist_curr)
        print("done: ", dist_curr)

    print("result distances:")
    for r in res:
        print(r)


def compute_distances_path(graphs):
    res = []
    nr_graphs = len(graphs)
    # graph_merged = graphs[(nr_graphs - 2)]
    # graph_main = graphs[(nr_graphs - 1)]
    # graph timestamps: graphs[0,nr_graphs]
    idxs = [(0, 1), (1, 2)]
    roots = [(0, 121), (121, 105)]
    for i_base, i_target in idxs:
        print("graphs:", i_base, i_target)
        graph_base = graphs[i_base]
        graph_target = graphs[i_target]
        paths = nx.optimize_edit_paths(graph_base, graph_target, timeout=10, roots=roots[i_base])
        for p in paths:
            path_latest = p

        # node_path = path_latest[0]
        # edge_path = path_latest[1]
        cost = path_latest[2]
        # print("node path:", node_path)
        # print("edge path:", edge_path)
        print("cost :", cost)

        res.append(path_latest)

        # node_edits = [el for el in node_path if el[0] != el[1]]
        # edge_edits = [el for el in edge_path if el[0] != el[1]]
        # cost_edits = len(node_edits) + len(edge_edits)
        # print("node edits:",  len(node_edits))
        # print("edge edits:",  len(edge_edits))
        # print("cost edits:",  cost_edits)

    return res


def export_distance_paths(paths):
    path_base = "../GTree/resources/data/GTree_examples/"
    cnt = 0
    for path in paths:
        # read paths into dataframes
        data_verts = pd.DataFrame(path[0], columns=["v0", "v1"])

        data_e = []
        for edge_step in path[1]:
            e0 = edge_step[0]
            e1 = edge_step[1]
            if e0 is None:
                e0 = (None, None)
            if e1 is None:
                e1 = (None, None)
            e = np.array([e0, e1]).flatten()
            data_e.append(e)

        data_edges = pd.DataFrame(data_e, columns=["e0v0", "e0v1", "e1v0", "e1v1"])

        # export to csv
        data_verts.to_csv(path_base + "data_vertices_" + str(cnt) + ".csv", index=False)
        data_edges.to_csv(path_base + "data_edges_" + str(cnt) + ".csv", index=False)

        cnt += 1


def main():
    directory = "../GTree/resources/data/GTree_examples/A/skeletons/"
    filenames = ["26CN1_ahn2_tree_0e76be44-6026-4edd-9800-dbf03aa47e40_skeleton_tsmain.ply",
                 "26CN1_ahn3_tree_0e76be44-6026-4edd-9800-dbf03aa47e40_skeleton_tsmain.ply",
                 "26CN1_ahn4_tree_0e76be44-6026-4edd-9800-dbf03aa47e40_skeleton_tsmain.ply",
                 "26CN1_ahn2_tree_0e76be44-6026-4edd-9800-dbf03aa47e40_skeleton_merged.ply",
                 "26CN1_ahn2_tree_0e76be44-6026-4edd-9800-dbf03aa47e40_skeleton_merged_main.ply"]

    data = open_files(directory, filenames)

    graphs = make_graphs(data)
    draw_graph(graphs)
    paths = compute_distances_path(graphs)
    # export_distance_paths(paths)


if __name__ == "__main__":
    main()
