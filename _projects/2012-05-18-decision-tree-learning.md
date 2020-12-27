---
layout: project
title: Decision Tree Learning
description: Implementation of a decision tree learning algorithm in MATLAB
date: 2012-05-18 00:00:00
categories: [ robotics, learning ]
tags: [ matlab, decision-tree ]
filename: 2012-05-18-decision-tree-learning
img: decision-tree.jpg
carousel: [ 1.jpg ]
website: https://github.com/nlamprian/Decision-Tree-Learning
---

This is a lab assignment in the "Artificial Intelligence II" course of the ECE department at the University of Patras, for the academic year 2011 - 2012. It involves building a decision tree on a dataset about the process of providing scholarships for a certain number of attendees to the Hellenic Artificial Intelligence Summer School (HAISS) 2011.

<img src="https://github.com/nlamprian/Decision-Tree-Learning/wiki/assets/tree-entropy.jpg" alt="tree entropy" />

The provided dataset is a spreadsheet with the following attributes:

<table class="border-table">
  <tr class="centered">
    <td style="width:18%">Academic Level</td>
    <td style="width:25.5%">Has Attended AI Course</td>
    <td style="width:15.5%">Year of Study</td>
    <td style="width:15%">Department</td>
    <td style="width:12%">University</td>
    <td style="width:14%;background:#ddd;">Scholarship</td>
  </tr>
</table>

The entries have values like:

<table class="border-table">
  <tr class="centered">
    <td style="width:18%">Undergraduate</td>
    <td style="width:25.5%">Yes</td>
    <td style="width:15.5%">4th</td>
    <td style="width:15%">ECE</td>
    <td style="width:12%">UPatras</td>
    <td style="width=14%;background:#ddd;">No</td>
  </tr>
</table>

The first thing I had to do was to encode the data so they can be handled by the algorithm. The scheme was to assign a unique zero based index to each of the different categories within each attribute:

<table class="border-table">
  <tr class="centered">
    <td style="width:18%">1</td>
    <td style="width:25.5%">1</td>
    <td style="width:15.5%">4</td>
    <td style="width:15%">3</td>
    <td style="width:12%">5</td>
    <td style="width=14%;background:#ddd;">0</td>
  </tr>
</table>

In the implementation, `decision_tree_learning` is the main function of the algorithm. It takes the following parameters: `examples` - remaining training set on the current node, `depth` - depth of the current node, `attributes` - remaining attributes (labels) on the current node, `tree_path` - path from the root to the current node, `parent_examples` - remaining training set on the parent node of the current node.

{% highlight matlab %}
decision_tree_learning(examples, depth, attributes, tree_path, parent_examples)
{% endhighlight %}

The encoded dataset appears on the `HAISS2011.txt` file. It is loaded automatically, and the resulting decision tree is displayed dynamically on the terminal. The algorithm creates the nodes on the tree in a `depth first` traversal. When a new node is created, we get back information (information gains of all remaining attributes) about the decision to peek the current attribute. For every leaf node that's created, the path to that node and the outcome is displayed on the console.

{% highlight text %}
% Sample output (node 5 on the figure)

Depth 2:
=============
Information Gains:
Attribute 2: 0.650022
Attribute 4: 0.650022
Attribute 5: 0.190875
Attribute with the highest I.G.: 2

Path:
==========
Attribute: 3 1 2
Value    : 2 2 0
Output   : 0

Path:
==========
Attribute: 3 1 2
Value    : 2 2 2
Output   : 1
{% endhighlight %}

The source code is available on [GitHub](https://github.com/nlamprian/Decision-Tree-Learning).
