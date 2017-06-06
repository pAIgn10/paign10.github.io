---
layout: project
title: Voronoi Diagrams
description: Implementation of a maximum area coverage algorithm utilizing Voronoi diagrams in MATLAB
date: 2012-09-24 00:00:00
categories: [ robotics, control ]
tags: [ matlab, mobile-robot, control, voronoi ]
filename: 2012-09-24-voronoi-diagrams
img: voronoi.jpg
carousel: [ 1.jpg, 2.jpg ]
website: https://github.com/nlamprian/Voronoi-Diagrams
---

This is a project in the "Robotic Systems" course of the ECE department at the University of Patras, for the academic year 2011 - 2012. It involves controlling 4 mobile robots such that they cover the maximum area possible inside a polygon.

<p><div class="row">
<div class="col-lg-8 col-md-8 col-sm-9">
<br>
We are given a convex polygon with the following coordinates:
<br>
<br>
{% highlight matlab %}
xa = [0 2.125 2.9325 2.975 2.9325 2.295 0.85 0.17];
ya = [0 0 1.5 1.6 1.7 2.1 2.3 1.2];
{% endhighlight %}
</div>
<div class="col-lg-4 col-md-4 col-sm-3">
<img src="http://i76.photobucket.com/albums/j16/paign10/voronoi-polygon_zps07b37a07.jpg" alt="cut" />
</div>
</div></p>

<p><div class="row">
<div class="col-md-7">
<br>
Inside this polygon, there are 4 mobile robots with the following initial positions and orientations:
</div>
<br>
<div class="col-md-5">
{% highlight matlab %}
xr = [0.25 0.5 0.75 1.0];
yr = [0.25 0.5 0.75 1.0];
th = [0 pi/6 pi/3 pi/2];
{% endhighlight %}
</div>
</div></p>

The task is to simulate the trajectories of the robots as they coordinate in order to cover the maximum area of the polygon. Each robot is responsible for circular areas around it with the following radii:

{% highlight matlab %}
R = [0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0];
{% endhighlight %}

r-limited Voronoi cells are used for the estimation of the coverage area of each robot. There are two distinct modes of operation for the robots. In one mode, the robots hasve a skid-steer model, so they are free to move in any direction, effectively ignoring their orientation. In the second one, they have a car-like model (with $$\Delta\phi_{max}=15^o$$), constraining this way their controllability.

The r-limited Voronoi cell are estimated as follows:

<p><div class="row">
<div class="col-xs-12 col-sm-5 col-sm-push-7">
<img src="http://i76.photobucket.com/albums/j16/paign10/voronoi-control_zps566eab3e.jpg" alt="voronoi-control" />
</div>
<div class="col-xs-12 col-sm-7 col-sm-pull-5">
<ol><li>
We find the \(2\) potential points (red circles in the figure) where a Voronoi cell intersects a neighboring one. We calculate the vectors from these points to the center of the neighboring cell, and then we add these vectors to get a new one. This last vector will be perpendicular to the line connecting the intersecting points and will be pointing away from the center of the first cell. We extend this vector and add it to the intersecting points to get a new set of points, further away from the neighboring cell. Considering these \(4\) points, we have created a rectangular area that encloses the intersection of the two cells.
</li></ol>
</div>
</div></p>

<ol start="2">
<li>We remove from the first cell the intersection between the cell and the computed rectangle, and we end up with a polygon that does not intersect the neighboring cell.</li>
<li>We repeat the above steps for all neighboring cells.</li>
<li>Finally, the r-limited Voronoi cell will be the intersection of the resulting polygon from the earlier steps with the convex hull of interest.</li>
</ol>

For the control input, we calculate the centroid of the r-limited Voronoi cell of each robot, and then we take a step towards these centroids while considering the modes of operation mentioned before.

<iframe width="560" height="315" src="https://www.youtube.com/embed/VVCh-MWaWYg?showinfo=0&amp;theme=light" frameborder="0" allowfullscreen></iframe>

The source code is available on [GitHub](https://github.com/nlamprian/Voronoi-Diagrams).
