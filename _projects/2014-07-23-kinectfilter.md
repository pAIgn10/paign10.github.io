---
layout: project
title: KinectFilter
description: Series of example applications putting together libfreenect, OpenCV, OpenGL and OpenCL
date: 2014-07-23 00:00:00
categories: [ robotics, processing ]
tags: [ c++, heterogeneous, opencl, opengl, kinect ]
filename: 2014-07-23-kinectfilter
img: kinectfilter.jpeg
carousel: [ 1.png ]
website: https://github.com/nlamprian/KinectFilter
---

KinectFilter offers a series of **example applications** that take the **Kinect data stream**, **process** it on the **GPU**, and **display** the results on a **window**. I sketched these applications and then further expanded them as I was learning about `OpenCL` and familiarizing myself with `libfreenect`.

The [libfreenect](https://github.com/OpenKinect/libfreenect/) **library** is used for interfacing with **Kinect**. The **processing** of the data is done on the **GPU** with `OpenCL`. `OpenGL` handles the display of the data. One main advantage of **OpenCL** is that it can share data directly with **OpenGL**. So, after processing, there is no need to return the results back to the host. OpenGL takes control of the buffers on the GPU and goes from there. The use of `textures` and `vertex buffers` is demonstrated. There are examples to showcase how to setup `OpenCL-OpenGL` `interoperability` using the **OpenCL C++ API**.

Currently, on **version 2.0**, there are 3 examples  in which a **Laplacian of Gaussian (LoG) filter** is applied on the **RGB stream**, and another one in which a **3D point cloud** is built while performing **RGB normalization**.

The **source code** is available on [GitHub](https://github.com/nlamprian/KinectFilter).
