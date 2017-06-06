---
layout: project
title: RandomBallCover
description: Implementation of the Random Ball Cover nearest neighbor data structure in OpenCL
date: 2015-05-14 00:00:00
categories: [ robotics, search ]
tags: [ c++, opencl, nearest-neighbor, point-cloud ]
filename: 2015-05-14-randomballcover
img: rbc.png
carousel: [ 1.png, 2.png ]
website: https://github.com/nlamprian/RandomBallCover
---

RandomBallCover is an implementation of a **Nearest Neighbor (NN)** data structure in OpenCL. **Random Ball Cover (RBC)** takes up where **Brute Force (BF)** leaves off. BF search on the GPU outperforms state-of-the-art NN search algorithms on the CPU. With RBC, it's possible to get even more performance gains than BF. RBC is a two-tier BF search that explores a heavily pruned search space. The data structure was proposed by Lawrence Cayton, and it's described in depth in two of his [papers](http://www.lcayton.com/papers.html).

Currently, a variation of the proposed algorithms is implemented that does **approximate NN search**. For the `RBC construction`, the data structure of the **exact search algorithm** is built where each database point is assigned to its nearest representative. The `RBC search` is done according to the **one-shot search algorithm**. The main application of the project is the handling of 6D (3D geometric and 3D photometric information) **point clouds**. The algorithm is able to perform on an input of `|X|=|Q|=16384` and `|R|=256` with the following results:
* RBC construction in 344 microseconds.
* RBC search in 714 microseconds.
* Mean error 1.05\*.

\* Mean distance between the queries and the computed NNs, relative to the mean distance between the queries and the true NNs.

It’s quite easy to use the Random Ball Cover data structure in your own applications. You only need to create the structure, and then you can search for the NNs of your queries. You can configure the classes to use different kernels, and customize them to your own system. Below is a dummy example to showcase the workflow. For more details, take a look at the examples in the repository.

{% highlight cpp linenos %}
#include <CLUtils.hpp>
#include <RBC/algorithms.hpp>

using namespace clutils;
using namespace cl_algo;

int main(int argc, char **argv) {
  // Setup OpenCL environment
  CLEnv env(kernel_files);

  // Configure kernel execution parameters for RBC construction
  CLEnvInfo<1> info(0, 0, 0, { 0 }, 0);
  const RBC::KernelTypeC K1 = RBC::KernelTypeC::SHARED_NONE;
  const RBC::RBCPermuteConfig P1 = RBC::RBCPermuteConfig::GENERIC;
  RBC::RBCConstruct<K1,P1> rbcC(env, info);
  rbcC.init(nx, nr, d);

  // Copy data to device
  rbcC.write(RBC::RBCConstruct<K1,P1>::Memory::D_IN_X, dataX);
  rbcC.write(RBC::RBCConstruct<K1,P1>::Memory::D_IN_R, dataR);

  rbcC.run();  // Construct

  // Configure kernel execution parameters for RBC search
  const RBC::KernelTypeC K2 = RBC::KernelTypeC::SHARED_NONE;
  const RBC::RBCPermuteConfig P2 = RBC::RBCPermuteConfig::GENERIC;
  const RBC::KernelTypeS S2 = RBC::KernelTypeS::GENERIC;
  RBC::RBCSearch<K2,P2,S2> rbcS(env, info);

  // Couple interfaces
  rbcS.get(RBC::RBCSearch<K2,P2,S2>::Memory::D_IN_R) = 
      rbcC.get (RBC::RBCConstruct<K1,P1>::Memory::D_IN_R);
  rbcS.get(RBC::RBCSearch<K2,P2,S2>::Memory::D_IN_X_P) = 
      rbcC.get (RBC::RBCConstruct<K1,P1>::Memory::D_OUT_X_P);
  rbcS.init(nq, nr, nx, d);

  // Copy data to device
  rbcS.write(RBC::RBCSearch<K2,P2,S2>::Memory::D_IN_Q), dataQ);

  rbcS.run();  // Search

  // Copy results to host
  cl_float *Qp = (cl_float *) rbcS.read(
      RBC::RBCSearch<K2,P2,S2>::Memory::H_OUT_Q_P, CL_FALSE);
  cl_float *NN = (cl_float *) rbcS.read(
      RBC::RBCSearch<K2,P2,S2>::Memory::H_OUT_NN);

  return 0;
}
{% endhighlight %}

You can find the complete **documentation** at [random-ball-cover.nlamprian.me](http://random-ball-cover.nlamprian.me/). The **source code** is available on [GitHub](https://github.com/nlamprian/RandomBallCover).
