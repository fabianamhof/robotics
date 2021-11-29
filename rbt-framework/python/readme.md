# RECONFIGURABLE BEHAVIOUR TREE FRAMEWORK

!!! preview !!! -> code could contain bugs

tested only on ubuntu 20.04 with python 3.8 (known problems with python 3.9). Best use a virtual enviroment

install needed packages with:
pip install -r requirements.txt

add the .../python folder of this project to your python path in order for this project to work. e.g. at pycharm go
to `interpreter setting` -> `gear-icon` -> `show all` -> select your enviroment -> `folder icon` -> `+ icon`
or `export PYTHONPATH=$PYTHONPATH":/path/to/repo/python"`

do the same for the gui-folder

also add the python package of your ros-installation to the python path in the same way if the handlePriorityBasic-Node
should be used (case for the example tree)
e.g. ```opt/ros/noetic/lib/python3/dist-packages```

A template for the nodes implementation is at nodes.py. If you want to implement a condition instead of a action rename
class to `Condition`

To use a different tree than the "basicNewTree.json" you have to adhust the used file in the main function for now (will
be parameterized soon).

The "guiMain" can already be tried out but it is impossible to safe/load/execute a tree yet.

## example

for the provided example (using ros so be sure all steps above are followed and ros is started) first start
treeviewer.py. afterwards start guiMain.py from within the gui-directory, load tree "basicTreeNew.json" from within the json dirctory and click on "start execution" and see the tree in the treeviewer window. last start rospy_tests_pub.py ->
the tree should change after ~5 seconds in the treeviewer-window as rospy_tests_pub.py changes the priority.

Note that this example only works if guimain is started from within the gui-folder due to some hard-coded paths

## ros-sensory-reader

if using the ros sensory reader you have to change the content in the topics.json file to the ros topics where your
sensory-data is written to. Ros has to be set up AND RUNNING in your environment. Data has to be written as a string (
probably json) by the sensory devices and handled by the nodes itself. The reader sets up a ros-subscriber to the topics
defined in topics.json file and writes them to the blackboard.

## Visualization of the tree

To visualize the tree, Qt is used. It basically reloads the rendered dot tree every 100 ms.

if on a unix-based system and getting error with "dot" not in path add your python site-packages to PATH or install
graphviz with your default package-manager i.e. ``` sudo apt install graphviz ```

After that, just execute

```
python ./treeviewer.py
```

Golden means success, green means running, red means failure

## json-format

A valid tree-file has to only contain one json array. At top-level are only controlflow nodes in the format:

```
{
    "name": String,
    "type": String,
    "children": [],
    "params": []
}
```

* name: can be choosen
* type: has to be "Sequence", "Parallel" or "Fallback"

children: two options for format:
either:

```
{
    "kind": String,
    "package":  String,
    "name": String
}
```

* kind: "Action" or "Condition"
* package: the package it is defined in
* name: can be choosen

or:

```
{
    "kind": "Controlflow",
    "name": String
}
```

* name: has to be a top-level element appearing later in the json array

parameters:

```{
    "kind": String,
    "package":  String,
    "name": String
    "conditioned_node": integer
}
```

* kind: "Precondition" or "Postcondition"
* package: the package it is defined in
* name: can be choosen
* conditioned_node: the position in the children-array which this pre-/postcondition is conditioning

### example

```
[
  {
    "name":"rbt_root",
    "type":"Fallback",
    "children": [
      {
        "kind": "Action",
        "package":  "action_1",
        "name": "action 1 name"},
      {
        "kind": "Controlflow",
        "name": "parallel_1"
      }
    ],
    "params": [
      {
        "kind": "Precondition",
        "package":  "condition_1",
        "name": "condition 1 name"
        "conditioned_bode": 1
      }
    ]
  },
  {
    "name":"parallel_1",
    "type":"Parallel",

    "children": [],
    "params": []
  }
]
```

## controls gui

double leftclick on node: create child single leftclick on node: select (+ all children)
hold single leftclick on node and move mouse: drag node + children around hold middle mouse button + move mouse: move on
screen
"delete": delete node and all children
