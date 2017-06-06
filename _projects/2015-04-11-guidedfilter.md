---
layout: project
title: GuidedFilter
description: Implementation of the Guided Image Filtering algorithm in OpenCL
date: 2015-04-11 00:00:00
categories: [ robotics, image-processing ]
tags: [ c++, library, filtering, smoothing, opencl, images, point_clouds ]
filename: 2015-04-11-guidedfilter
img: gf.png
carousel: [ 1.png, 2.png, 3.png ]
website: https://github.com/nlamprian/GuidedFilter
---

GuidedFilter is an implementation of the [Guided Filter](http://research.microsoft.com/en-us/um/people/kahe/eccv10/) algorithm in OpenCL. The Guided Filter is an image filter with many applications. It has a non-approximate algorithm which is `O(1)` in the filter window size. It has excellent performance characteristics which makes it a great alternative to the popular `Bilateral Filter`.

Applications of the Guided Filter include **Detail Enhancement and HDR Compression**, **Flash/No-Flash Denoising**, **Matting/Guided Feathering**, **Haze Removal**, and **Joint Upsampling**. But the most basic and important application of the filter, and the one that interests me, is the **Edge-Preserving Smoothing**. The Guided Filter gives great results and doesn't suffer from gradient reversal artifacts that can be introduced by the Bilateral Filter.

The Guided Filter can operate on **RGB** images, as well as **Depth** images. The GuidedFilter repository contains two applications that let one examine the effects of the filter interactively, on both these cases. The following videos demonstrate the applications.

**Guided Image Filtering on Kinect RGB stream with OpenCL:**

<iframe width="560" height="315" src="https://www.youtube.com/embed/cFQu10OsztI?showinfo=0&amp;theme=light" frameborder="0" allowfullscreen></iframe>

**Guided Image Filtering on Kinect RGB and Depth streams with OpenCL:**

<iframe width="560" height="315" src="https://www.youtube.com/embed/PTLU1SiHCEY?showinfo=0&amp;theme=light" frameborder="0" allowfullscreen></iframe>

It's very easy to apply the **Guided Filter** in your own applications. A consistent **API** has been adopted across all operations supported by the project. You configure and initialize an object associated with an algorithm, write your data on a device buffer, execute the kernel(s), and get back the results. Below is a dummy example to showcase the workflow. For more details, take a look at the examples in the repository.

{% highlight cpp linenos %}
#include <CLUtils.hpp>
#include <GuidedFilter/algorithms.hpp>

using namespace clutils;
using namespace cl_algo;

int main(int argc, char **argv) {
  // Setup the OpenCL environment
  CLEnv env(kernel_files);  // Initializes a basic environment
  env.addQueue(0, 0);  // Adds a second queue

  // Configure kernel execution parameters
  CLEnvInfo<2> info(0, 0, 0, { 0, 1 }, 0);
  const GF::GuidedFilterConfig C = GF::GuidedFilterConfig::I_EQ_P;
  GF::GuidedFilter<C> gf(env, info);
  gf.init(width, height, radius, eps);

  // Copy data to device
  gf.write(GF::GuidedFilter<C>::Memory::D_IN, data);

  // Execute kernels
  gf.run();

  // Copy results to host
  cl_float *results = (cl_float *) gf.read();

  return 0;
}
{% endhighlight %}

You can find the complete **documentation** at [guided-filter.nlamprian.me](http://guided-filter.nlamprian.me/). The **source code** is available on [GitHub](https://github.com/nlamprian/GuidedFilter).
