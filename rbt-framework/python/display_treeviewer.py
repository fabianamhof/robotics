# code partly taken from github.com/splintered-reality/py_trees

import os
import pydot
import typing
import uuid


from py_trees import behaviour
from py_trees import blackboard
from py_trees import common
from py_trees import composites
from py_trees import console
from py_trees import decorators
from py_trees import utilities

'''
function to create the dot tree from the current Behavior Tree
basically the same implementation like in py_trees.display, but takes
actual node status (RUNNING, SUCCESS, FAILURE) into consideration
RUNNING ... green
SUCCESS ... gold
FAILURE ... red
'''
def dot_tree(
        root: behaviour.Behaviour,
        visibility_level: common.VisibilityLevel=common.VisibilityLevel.DETAIL,
        collapse_decorators: bool=False,
        with_blackboard_variables: bool=False,
        with_qualified_names: bool=False):
    
    def get_node_attributes(node):
        blackbox_font_colours = {common.BlackBoxLevel.DETAIL: "dodgerblue",
                                 common.BlackBoxLevel.COMPONENT: "lawngreen",
                                 common.BlackBoxLevel.BIG_PICTURE: "white"
                                 }
        if node.status == common.Status.RUNNING:
            color = 'green'
        elif node.status == common.Status.FAILURE:
            color = 'red'
        elif node.status == common.Status.SUCCESS:
            color = 'gold'
        else:
            color = 'gray'
        if isinstance(node, composites.Selector):
            attributes = ('octagon', color, 'black')  # octagon
        elif isinstance(node, composites.Sequence):
            attributes = ('box', color, 'black')
        elif isinstance(node, composites.Parallel):
            attributes = ('parallelogram', color, 'black')
        elif isinstance(node, decorators.Decorator):
            attributes = ('ellipse', color, 'black')
        else:
            attributes = ('ellipse', color, 'black')
        try:
            if node.blackbox_level != common.BlackBoxLevel.NOT_A_BLACKBOX:
                attributes = (attributes[0], 'gray20', blackbox_font_colours[node.blackbox_level])
        except AttributeError:
            # it's a blackboard client, not a behaviour, just pass
            pass
        return attributes

    def get_node_label(node_name, behaviour):
        """
        This extracts a more detailed string (when applicable) to append to
        that which will be used for the node name.
        """
        node_label = node_name
        if behaviour.verbose_info_string():
            node_label += "\n{}".format(behaviour.verbose_info_string())
        if with_qualified_names:
            node_label += "\n({})".format(utilities.get_fully_qualified_name(behaviour))
        return node_label

    fontsize = 9
    blackboard_colour = "blue"  # "dimgray"
    graph = pydot.Dot(graph_type='digraph', ordering="out")
    graph.set_name("pastafarianism")  # consider making this unique to the tree sometime, e.g. based on the root name
    # fonts: helvetica, times-bold, arial (times-roman is the default, but this helps some viewers, like kgraphviewer)
    graph.set_graph_defaults(fontname='times-roman')  # splines='curved' is buggy on 16.04, but would be nice to have
    graph.set_node_defaults(fontname='times-roman')
    graph.set_edge_defaults(fontname='times-roman')
    (node_shape, node_colour, node_font_colour) = get_node_attributes(root)
    node_root = pydot.Node(
        root.name,
        label=get_node_label(root.name, root),
        shape=node_shape,
        style="filled",
        fillcolor=node_colour,
        fontsize=fontsize,
        fontcolor=node_font_colour,
    )
    graph.add_node(node_root)
    behaviour_id_name_map = {root.id: root.name}

    def add_children_and_edges(root, root_node, root_dot_name, visibility_level, collapse_decorators):
        if isinstance(root, decorators.Decorator) and collapse_decorators:
            return
        if visibility_level < root.blackbox_level:
            node_names = []
            for c in root.children:
                (node_shape, node_colour, node_font_colour) = get_node_attributes(c)
                node_name = c.name
                while node_name in behaviour_id_name_map.values():
                    node_name += "*"
                behaviour_id_name_map[c.id] = node_name
                # Node attributes can be found on page 5 of
                #    https://graphviz.gitlab.io/_pages/pdf/dot.1.pdf
                # Attributes that may be useful: tooltip, xlabel
                node = pydot.Node(
                    name=node_name,
                    label=get_node_label(node_name, c),
                    shape=node_shape,
                    style="filled",
                    fillcolor=node_colour,
                    fontsize=fontsize,
                    fontcolor=node_font_colour,
                )
                node_names.append(node_name)
                graph.add_node(node)
                edge = pydot.Edge(root_dot_name, node_name)
                graph.add_edge(edge)
                if c.children != []:
                    add_children_and_edges(c, node, node_name, visibility_level, collapse_decorators)

    add_children_and_edges(root, node_root, root.name, visibility_level, collapse_decorators)

    def create_blackboard_client_node(blackboard_client: blackboard.Blackboard):
        return pydot.Node(
            name=blackboard_client.name,
            label=blackboard_client.name,
            shape="ellipse",
            style="filled",
            color=blackboard_colour,
            fillcolor="gray",
            fontsize=fontsize - 2,
            fontcolor=blackboard_colour,
        )

    def add_blackboard_nodes(blackboard_id_name_map: typing.Dict[uuid.UUID, str]):
        data = blackboard.Blackboard.storage
        metadata = blackboard.Blackboard.metadata
        clients = blackboard.Blackboard.clients
        # add client (that are not behaviour) nodes
        subgraph = pydot.Subgraph(
            graph_name="Blackboard",
            id="Blackboard",
            label="Blackboard",
            rank="sink",
        )
        blackboard_keys = pydot.Node(
            "BlackboardKeys",
            label="Keys",
            shape='box'
        )
        root_dummy_edge = pydot.Edge(
            root.name,
            "BlackboardKeys",
            color="magenta",
            style="invis",
            constraint=True,
        )
        subgraph.add_node(blackboard_keys)
        graph.add_edge(root_dummy_edge)

        for unique_identifier, client in clients.items():
            if unique_identifier not in blackboard_id_name_map:
                subgraph.add_node(
                    create_blackboard_client_node(client)
                )
        # add key nodes
        for key in blackboard.Blackboard.keys():
            try:
                value = utilities.truncate(str(data[key]), 20)
                label = key + ": " + "{}".format(value)
            except KeyError:
                label = key + ": " + "-"
            blackboard_node = pydot.Node(
                key,
                label=label,
                shape='box',
                style="filled",
                color=blackboard_colour,
                fillcolor='white',
                fontsize=fontsize - 1,
                fontcolor=blackboard_colour,
                width=0, height=0, fixedsize=False,  # only big enough to fit text
            )
            subgraph.add_node(blackboard_node)
            for unique_identifier in metadata[key].read:
                try:
                    edge = pydot.Edge(
                        blackboard_node,
                        blackboard_id_name_map[unique_identifier],
                        color="green",
                        constraint=False,
                        weight=0,
                    )
                except KeyError:
                    edge = pydot.Edge(
                        blackboard_node,
                        clients[unique_identifier].__getattribute__("name"),
                        color="green",
                        constraint=False,
                        weight=0,
                    )
                graph.add_edge(edge)
            for unique_identifier in metadata[key].write:
                try:
                    edge = pydot.Edge(
                        blackboard_id_name_map[unique_identifier],
                        blackboard_node,
                        color=blackboard_colour,
                        constraint=False,
                        weight=0,
                    )
                except KeyError:
                    edge = pydot.Edge(
                        clients[unique_identifier].__getattribute__("name"),
                        blackboard_node,
                        color=blackboard_colour,
                        constraint=False,
                        weight=0,
                    )
                graph.add_edge(edge)
        graph.add_subgraph(subgraph)

    if with_blackboard_variables:
        blackboard_id_name_map = {}
        for b in root.iterate():
            for bb in b.blackboards:
                blackboard_id_name_map[bb.id()] = behaviour_id_name_map[b.id]
        add_blackboard_nodes(blackboard_id_name_map)

    return graph

'''
Writes the dot tree into a png image for visualization
uses the dot_tree function to create tree
'''
def render_dot_tree(root: behaviour.Behaviour,
                    visibility_level: common.VisibilityLevel=common.VisibilityLevel.DETAIL,
                    collapse_decorators: bool=False,
                    name: str=None,
                    target_directory: str=os.getcwd(),
                    with_blackboard_variables: bool=False,
                    with_qualified_names: bool=False):
    
    graph = dot_tree(
        root, visibility_level, collapse_decorators,
        with_blackboard_variables=with_blackboard_variables,
        with_qualified_names=with_qualified_names)
    filename_wo_extension_to_convert = root.name if name is None else name
    filename_wo_extension = utilities.get_valid_filename(filename_wo_extension_to_convert)
    filenames = {}
    extension, writer = "png", graph.write_png
    filename = filename_wo_extension + '.' + extension
    pathname = os.path.join(target_directory, filename)
    writer(pathname)
    filenames[extension] = pathname
    return filenames