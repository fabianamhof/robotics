# libraries
import json

import py_trees
import inspect

# modules
import nodes as nodes
import Actionnodes
import importlib
import node_node as gui_node

used_names = []


#####################################################################
# instantiates subtree specified in a JSON file
#
# input:
#   - jsonFile: schema of the tree
# return:
#   - BehaviourTree specified in the jsonFile
def instantiateSubtree(jsonFile, scene=None):
    # initialize root of subtree
    name = jsonFile[0]['name']
    root = schemaToNode(name, jsonFile, scene)
    if root == None:
        raise Exception("Composite type doesn't exist")

    return root


#####################################################################

#####################################################################
# creates node from given schema and calls it recursively for
# each child
#
# input:
#   - name: name of the node to create
#   - jsonFile: dictonary with all schemas
# return:
#   - Node specified in the schema
def schemaToNode(name, jsonFile, scene=None, root=False):
    # get schema and create current node
    schema = getSchema(name, jsonFile)
    if scene != None and not 'position_x' in schema:
        schema, jsonFile = setCoordinates(name, jsonFile)
    node = createCompositeNode(schema, scene, root=root)

    # get conditions
    children = schema['children']
    params = schema['params']
    preC, postC = getConditions(params, len(children))

    childNum = 0
    while childNum < len(children):
        T_fal = postConditions(postC[childNum], scene, schema)
        child = children[childNum]
        isExecutionNode = (child['kind'] == "Action" or child['kind'] == "Condition")
        T_seq = preConditionsAndActions(preC[childNum], child, isExecutionNode, scene, schema)

        if len(postC[childNum]) != 0:
            if T_seq != None:
                T_fal.add_child(T_seq) if scene == None else T_fal.addChild(T_seq, initBtNode=False)
            if T_fal != None:
                node.add_child(T_fal) if scene == None else node.addChild(T_fal, initBtNode=False)
        elif T_seq != None:
            node.add_child(T_seq) if scene == None else node.addChild(T_seq, initBtNode=False)

        if (not isExecutionNode):
            node.add_child(schemaToNode(child['name'], jsonFile, scene)) if scene == None else node.addChild(schemaToNode(child['name'], jsonFile, scene, root=False), initBtNode=False)

        childNum += 1

    return node


#####################################################################

#####################################################################

# creates a json-schema given a specific py-tree node as th root of the tree to be saved

# input:
#   - node: the node which will be converted to a json-schema with all its children
#   - json_file: the JSON-dict to put the generated schema in. //has to be empty for the root node

def node_to_schema(node: py_trees.composites.Composite, guiNode=None, json_file=[], name=None):
    schema = None
    if name == None:
        name = makeUniqueName(node.name, resetUsed=True)
    hasGraphicalNode = guiNode != None

    if (hasGraphicalNode):
        schema = {
            'name': name,
            'type': getNodeType(node),
            'position_x': guiNode.grNode.pos().x(),
            'position_y': guiNode.grNode.pos().y()
        }
    else:
        schema = {
            'name': name,
            'type': getNodeType(node)
        }
    schema_children = []
    schema_conditions = []
    root_children = node.children
    composite_children = []
    composite_children_gui = []
    for child_num in range(len(root_children)):

        added = False
        # ------------- check if child can be converted to post-condition -------------

        # can only be converted if the child is Fallback(Selector)
        gui_child = get_gui_child(guiNode, child_num)
        if isinstance(root_children[child_num], py_trees.composites.Selector):
            child_children = root_children[child_num].children

            # can only be converted if direct child has 2 children (either cond + action or sequence + action)
            if len(child_children) == 2:

                # case cond and action
                if isinstance(child_children[0], Actionnodes.conditions.condition.Condition):
                    addPreconditionsAndAction(child_children, composite_children, get_gui_child(gui_child, 0), hasGraphicalNode,
                                              schema_children, schema_conditions)
                    schema_conditions.append(
                        create_condition(child_children[0], hasGraphicalNode, get_gui_child(gui_child, 1), "Postcondition",
                                         len(schema_children) - 1))
                    added = True

                # case sequence and action
                elif isinstance(child_children[0], py_trees.composites.Sequence):

                    # check if sequence on the left only has conditions
                    all_conditions = True
                    child_child_children = child_children[0].children
                    for child_child_child in child_child_children:
                        if not isinstance(child_child_child, Actionnodes.conditions.condition.Condition):
                            all_conditions = False
                            break
                    if all_conditions:
                        addPreconditionsAndAction(child_children, composite_children, gui_child, hasGraphicalNode,
                                                  schema_children, schema_conditions)
                        for child_child_child_num in range(len(child_child_children)):
                            schema_conditions.append(
                                create_condition(child_child_children[child_child_child_num], hasGraphicalNode,
                                                 get_gui_child(gui_child, child_child_child_num),
                                                 "Postcondition", len(schema_children) - 1))
                        added = True

        # ------------- check if child can be converted to pre-condition ------------

        # can only be converted if the child is Fallback(Selector)
        elif isinstance(root_children[child_num], py_trees.composites.Sequence):
            child_children = root_children[child_num].children

            # last node of sequence children has to be action for convertion in pre-conditions
            if len(child_children) > 0 and isinstance(child_children[-1], Actionnodes.actions.action.Action):

                # check if all other nodes are postconditions
                all_conditions = True
                for child_child in child_children[:-1]:
                    if not isinstance(child_child, Actionnodes.conditions.condition.Condition):
                        all_conditions = False
                        break
                if all_conditions:
                    schema_children.append(
                        create_action(child_children[-1], hasGraphicalNode, get_gui_child(gui_child, -1)))
                    for child_child_num in range(len(child_children) - 1):
                        schema_conditions.append(
                            create_condition(child_children[child_child_num], hasGraphicalNode,
                                             get_gui_child(gui_child, child_child_num), "Precondition",
                                             len(schema_children) - 1))
                    continue

        # --------------- no conversion of child to pre- or post-condition case possible--------------

        # check if child is composite -> has to be added to schema seperately
        if isinstance(root_children[child_num], py_trees.composites.Composite) and not added:
            comp_name = makeUniqueName(root_children[child_num].name)
            schema_children.append({"name": comp_name, "kind": "Controlflow"})
            composite_children.append((root_children[child_num], comp_name))
            composite_children_gui.append(gui_child)

        # else just append it as a normal action-node
        elif not added:
            if isinstance(root_children[child_num], Actionnodes.actions.action.Action):
                schema_children.append(create_action(root_children[child_num], hasGraphicalNode, gui_child))
            else:
                schema_children.append(create_condition(root_children[child_num], hasGraphicalNode, gui_child))
    # add calculated children and conditions to schema
    schema['children'] = schema_children
    schema['params'] = schema_conditions
    json_file.append(schema)
    for index, child in enumerate(composite_children):
        node_to_schema(child[0], composite_children_gui[index], json_file, child[1])
    return json_file


def addPreconditionsAndAction(child_children, composite_children, guiNode, hasGraphicalNode, schema_children,
                              schema_conditions):
    if isinstance(child_children[1], py_trees.composites.Composite):
        all_conditions = True
        gui_child = get_gui_child(guiNode, 1)
        child_child_children = child_children[1].children
        for child_child_child in child_child_children[:-1]:
            if not isinstance(child_child_child, Actionnodes.conditions.condition.Condition):
                all_conditions = False
        if len(child_child_children) != 0 and all_conditions and isinstance(
                child_child_children[-1], Actionnodes.actions.action.Action):
            schema_children.append(
                create_action(child_child_children[-1], hasGraphicalNode, get_gui_child(gui_child[1])))
            for index, child_child_child in enumerate(child_child_children[:-1]):
                schema_conditions.append(
                    create_condition(child_child_child, hasGraphicalNode, get_gui_child(gui_child, index),
                                     "Precondition",
                                     len(schema_children) - 1))
        else:
            schema_children.append({"name": child_children[1].name, "kind": "Controlflow"})
            composite_children.append(child_children[1])
    else:
        schema_children.append(create_action(child_children[1], hasGraphicalNode, guiNode))


#####################################################################

#####################################################################
# get schema for specific node
#
# input:
#   - name: name of the node to get schema from
#   - jsonFile: file with all schemas
# return:
#   - schema specified by name
def getSchema(name, jsonFile):
    for schema in jsonFile:
        if schema['name'] == name:
            return schema

    raise Exception("Schema " + str(name) + " not found")


#####################################################################

#####################################################################    
# creates composite node from a schema
#
# input:
#   - schema: schema of a composite node in the tree
# return:
#   - node specified in the schema 
def createCompositeNode(schema, scene=None, root=False):
    # return one of the 3 possible composite nodes
    composites = {
        'Fallback': py_trees.composites.Selector(schema['name']),
        'Sequence': py_trees.composites.Sequence(schema['name']),
        'Parallel': py_trees.composites.Parallel(schema['name'])
    }

    if scene == None:
        return composites.get(schema['type'], None)
    node = gui_node.Node(scene, root=root, title=schema['name'], posX=schema['position_x'], posY=schema['position_y'],
                         kind=schema['type'])
    if not root:
        node.initBTNode(node)
    return node


#####################################################################

#####################################################################    
# gets all pre- and postconditions of all children
#
# input:
#   - params: parameters of the schema
#   - numChildren: number of child nodes
# return:
#   - pre- and postconditions from given parameters
def getConditions(params, numChildren):
    # initialize conditions for each child
    preC = [[] for i in range(numChildren)]
    postC = [[] for i in range(numChildren)]
    if len(params) == 0:
        return preC, postC

    for condition in params:
        # check if pre- or postConditions and which child
        # it belongs to
        childNum = condition['conditioned_node']

        # add conditions
        if condition['kind'] == 'Postcondition':
            postC[childNum].append(condition)
        elif condition['kind'] == 'Precondition':
            preC[childNum].append(condition)

    return preC, postC


#####################################################################

#####################################################################    
# creates postcondition nodes
#
# input:
#   - postC: all postconditions of the cihld
# return:
#   - tree of postconditions
def postConditions(postC, scene, schema: {}):
    numConditions = len(postC)
    if numConditions == 0:
        return None

    if scene != None:
        mockschema = {"name": "post_cond_fallback", "type": "Fallback", "position_x": schema['position_x'],
                      "position_y": schema['position_y'] + 200}
        fallback = createCompositeNode(mockschema, scene)
        if numConditions == 1:
            cond_node = gui_node.Node(scene, postC[0]['name'], posX=postC[0]['position_x'],
                                      posY=postC[0]['position_y'],
                                      kind=("Condition " + postC[0]['package']))
            cond_node.initBTNode(cond_node)
            fallback.addChild(cond_node, initBtNode=False)
        else:
            sequence = gui_node.Node(scene, "sequence_post_cond", posX=fallback.pos.x(), posY=fallback.pos.y() + 200,
                                     kind='Sequence')
            sequence.initBTNode(node)
            for cond in postC:
                cond_node = gui_node.Node(scene, cond['name'], posX=cond['position_x'],
                                          posY=cond['position_y'],
                                          kind=("Condition " + cond['package']))
                cond_node.initBTNode(cond_node)
                sequence.addChild(cond_node)
            fallback.addChild(sequence)
        return fallback


    else:

        fallback = py_trees.composites.Selector("Fallback")

        if numConditions == 1:
            fallback.add_child(getExecutionNodeByName(postC[0]['package'], postC[0]['kind'], postC[0]['name']))
        else:
            sequence = py_trees.composites.Sequence("Sequence")
            for cond in postC:
                sequence.add_child(getExecutionNodeByName(cond['package'], cond['kind'], cond['name']))
            fallback.add_child(sequence)

        return fallback


#####################################################################


#####################################################################    
# create precondition and action nodes
#
# input:
#   - preC: all preconditions of the child
#   - action: name of the action/child
#   - isActionNode: Boolean for seperate handling
# return:
#   - tree of precondtions and actions
def preConditionsAndActions(preC, action, isActionNode, scene=None, schema=None):
    if scene != None:
        actionNode = None
        if isActionNode:
            actionNode = gui_node.Node(scene, action['name'], posX=action['position_x'],
                                      posY=action['position_y'],
                                      kind=(getActionOrCondition(action) + " " + action['package']))
            actionNode.initBTNode(actionNode)
        if len(preC) == 0:
            return actionNode
        mockschema = {"name": "pre_cond_sequence", "type": "Sequence", "position_x": schema['position_x'],
                      "position_y": schema['position_y'] + 200}
        sequence = createCompositeNode(mockschema, scene)

        for cond in preC:
            cond_node = gui_node.Node(scene, cond['name'], posX=cond['position_x'],
                                      posY=cond['position_y'],
                                      kind=("Condition " + cond['package']))
            sequence.addChild(cond_node)

        if isActionNode:
            sequence.addChild(actionNode, initBtNode=False)

        elif len(preC) == 1:
            node = gui_node.Node(scene, preC[0]['name'], posX=preC[0]['position_x'],
                                      posY=preC[0]['position_y'],
                                      kind=(getActionOrCondition(preC[0]) + " "  + preC[0]['package']))
            node.initBTNode(node)
            return node
        return sequence

    else:
        actionNode = None
        if isActionNode:
            actionNode = getExecutionNodeByName(action['package'], action['kind'], action['name'])

        # case for no preconditions
        if len(preC) == 0:
            return actionNode

        sequence = py_trees.composites.Sequence("Sequence")

        for cond in preC:
            sequence.add_child(getExecutionNodeByName(cond['package'], cond['kind'], cond['name']))

        # case for action node with preconditions
        if isActionNode:
            sequence.add_child(actionNode)
        # case for no action node with 1 precondition (no seq)
        elif len(preC) == 1:
            return getExecutionNodeByName(preC[0]['package'], preC[0]['kind'], preC['name'])

        # all other cases
        return sequence

def getActionOrCondition(schema: {}):
    if schema['kind'] == "Action":
        return "Action"
    return "Condition"


def getExecutionNodeByName(package: str, kind: str, name: str):
    module_name = "Actionnodes." + str.lower(kind if kind == "Action" else "Condition") + "s." + package
    module = importlib.import_module(module_name)
    kind = "Condition" if kind == "Precondition" or kind == "Postcondition" else kind
    class_ = getattr(module, kind)
    instance = class_(name)
    return instance


#####################################################################

def getNodeType(node: py_trees.composites.Composite):
    if isinstance(node, py_trees.composites.Sequence):
        return 'Sequence'
    elif isinstance(node, py_trees.composites.Parallel):
        return 'Parallel'
    elif isinstance(node, py_trees.composites.Selector):
        return 'Fallback'
    elif isinstance(node, Actionnodes.actions.action.Action):
        return 'Action'
    elif isinstance(node, Actionnodes.conditions.condition.Condition):
        return 'Condition'
    raise Exception('NodeType not available' + str(node.__class__))


def retrieve_class(node):
    path = inspect.getfile(node.__class__).split('.')[0].split('/')
    i = 0
    while path[i] != 'Actionnodes':
        i = i + 1
    return '.'.join(path[i:], ) + '.' + node.__class__.__name__ + '(' + node.name + ')'


def create_action(node, hasGraphicalNode, node_gui=None):
    if hasGraphicalNode:
        return {"kind": "Action",
                "package": node.__module__.split('.')[-1],
                "name": node.name,
                "position_x": node_gui.grNode.pos().x(),
                "position_y": node_gui.grNode.pos().y()}
    else:
        return {"kind": "Action",
                "package": node.__module__.split('.')[-1],
                "name": node.name}


def create_condition(node, hasGraphicalNode, node_gui=None, conditionType="Condition", referrenced=None):
    if hasGraphicalNode:
        if referrenced != None:
            return {"kind": conditionType,
                    "package": node.__module__.split('.')[-1],
                    "name": node.name,
                    "position_x": node_gui.grNode.pos().x(),
                    "position_y": node_gui.grNode.pos().y(),
                    "conditioned_node": referrenced}
        else:
            return {"kind": conditionType,
                    "package": node.__module__.split('.')[-1],
                    "name": node.name,
                    "position_x": node_gui.grNode.pos().x(),
                    "position_y": node_gui.grNode.pos().y()}
    else:
        if referrenced != None:
            return {"kind": conditionType,
                    "package": node.__module__.split('.')[-1],
                    "name": node.name,
                    "conditioned_node": referrenced}
        else:
            return {"kind": conditionType,
                    "package": node.__module__.split('.')[-1],
                    "name": node.name}


def get_gui_child(guiNode, index):
    if guiNode == None:
        return None
    return guiNode.children[index]


def makeUniqueName(name: str, resetUsed=False):
    global used_names
    if resetUsed:
        used_names = []

    uniqueName = False
    newName = name
    i = 0
    while (not uniqueName):
        uniqueName = True
        for used_name in used_names:
            if used_name == newName:
                uniqueName = False
                newName = name + '_' + str(i)
                i += 1
                break
    used_names.append(newName)
    return newName


def setCoordinates(name, jsonFile):
    root = getSchema(name, jsonFile)
    root['position_x'] = 4000
    root['position_y'] = 40
    jsonFile = setSchema(jsonFile, root)
    return root, setAllPositionsLowerLevel([root], jsonFile)


def setSchema(jsonFile, schema):
    for i, old_schema in enumerate(jsonFile):
        if schema['name'] == old_schema['name']:
            jsonFile[i] = schema
    return jsonFile

    raise Exception("Schema " + str(name) + " not found")


def setAllPositionsLowerLevel(schemas: [], jsonFile):
    all_positons_to_set = []
    potential_children = []
    for schema in schemas:
        if 'children' in schema:
            for i, child in enumerate(schema['children']):
                all_positons_to_set.append((child, 'child', i, schema))
            for i, param in enumerate(schema['params']):
                all_positons_to_set.append((param, 'param', i, schema))
    width = len(all_positons_to_set)
    for i, entry in enumerate(all_positons_to_set):
        if entry[1] == 'child':
            if entry[0]['kind'] == "Controlflow":
                schema = getSchema(entry[0]['name'], jsonFile)
                schema['position_x'] = 4000 + ((i - width // 2) * 300)
                schema['position_y'] = entry[3]['position_y'] + 200
                jsonFile = setSchema(jsonFile, schema)
                potential_children.append(schema)
            else:
                child = entry[0]
                child['position_x'] = 4000 + ((i - width // 2) * 300)
                child['position_y'] = entry[3]['position_y'] + 200
                schema = getSchema(entry[3]['name'], jsonFile)
                schema['children'][entry[2]] = child
                jsonFile = setSchema(jsonFile, schema)
        else:
            param = entry[0]
            param['position_x'] = 4000 + ((i - width // 2) * 300)
            param['position_y'] = entry[3]['position_y'] + 200
            schema = getSchema(entry[3]['name'], jsonFile)
            schema['params'][entry[2]] = param
            jsonFile = setSchema(jsonFile, schema)
    if not len(potential_children) == 0:
        jsonFile = setAllPositionsLowerLevel(potential_children, jsonFile)
    return jsonFile
