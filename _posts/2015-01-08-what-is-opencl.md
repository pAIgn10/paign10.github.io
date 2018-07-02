---
layout: post
title: What is OpenCL?
subtitle: From computer architectures to parallel programming
date: 2015-01-08 00:00:00
author: Nick Lamprianidis
header-img: 2015-01-08-what-is-opencl/logo.png
categories: [ blog, software, parallel, opencl ]
tags: [ parallel, heterogeneous, opencl, overview ]
---

In the '60s, Moore's law predicted that every two years, transistor density in an integrated circuit would double. Having smaller transistor size would allow us to increase the operating frequency and thus double the performance. But increasing frequency also increases power consumption. So at some point, this process would hit a limit where a slight increase in frequency would cause excessive power consumption and meet heat dissipation constraints. CPU designers had to find other ways to increase performance. They kept frequency at constant levels and developed a number of `design patterns` that increased the operations performed in a single CPU on a given clock cycle. Here is a short talk on these ideas.

<iframe width="560" height="315" src="https://www.youtube.com/embed/ET9wxEuqp_Y?showinfo=0&amp;theme=light" frameborder="0" allowfullscreen></iframe>

Those design patterns extract the `parallelism` found in an `execution sequence` and try to fill the different units of the processor with work. And if instructions aren't arriving fast enough, `cache` memories are added to bring the data closer. But it's not always possible to cover the entire CPU with work. So, if one program is not enough to flood the CPU, then why not run two or more programs together. Let's call them `threads`. If one thread leaves available units, the other threads will use them. Another way to occupy multiple units is to offer instructions that will operate on a bunch of data independently. This is called `SIMD` (Single Instruction Multiple Data) or `Vector Processing`. But ultimately, the natural development to performance increase, with all those available transistors, is adding more `CPU cores` on a single chip. Now threads can run in parallel without interfering with each other.

Why did I mention all these? So that I can answer the following question: What's the difference between a `CPU` and a `GPU`? For a long time, I myself thought that GPUs involved a whole other industry, independent of that of CPUs. But this is not the case. GPUs constitute just an architecture under one common `architectural` `design` `space`. Choosing some of the design patterns mentioned above and implementing them in different degrees, defines an architecture.

<p><div class="row">
    <div class="col-lg-8 col-md-8 col-sm-9">
        CPUs aim for low <code>latency</code> of a single task. They work to complete the execution of <code>one thread</code> in the shortest possible time. For that, they dedicate a significant portion of the die area to hardware and cache memories that make this happen. The CPU is great for executing algorithms with many data associations, like sorting. The control hardware in the CPU will make sure to service those dependencies in the most efficient way.
    </div>
    <div class="col-lg-4 col-md-4 col-sm-3">
        <img src="http://images.anandtech.com/reviews/cpu/intel/SNBE/Core_I7_LGA_2011_Die.jpg" alt="anandtech core i7" >
    </div>
</div></p>

<p><div class="row">
    <div class="col-lg-4 col-md-4 col-sm-3">
        <img src="http://images.anandtech.com/doci/8526/GeForce_GTX_980_Block_Diagram_FINAL.png" alt="anandtech fermi" >
    </div>
    <div class="col-lg-8 col-md-8 col-sm-9">
        On the other hand, GPUs are <code>throughput</code> processors. They work to minimize the execution time of <code>a number of threads</code>. GPUs try to get more jobs done per unit time. For that, they include a large number of simpler processing elements. A single processing element might work slower than that of a CPU, but as a whole, a GPU manages to actually get more things done in a given time. But that is not even close to the whole story.
    </div>
</div>
<div class="row">
    <div class="col-lg-12 col-md-12 col-sm-12">
        What gives GPUs the great advantage, and the chance to achieve staggering execution times, is their inherent ability to address <code>parallelism</code>.
    </div>
</div></p>

<p><div class="row">
    <div class="col-lg-8 col-md-8 col-sm-9">
        Think for a second about how many needs an average person has nowadays. Would that person be able to live by today's standards if they had to handle everything by themselves? Probably not. What allows us to maintain a quality of life is our presence within communities.
    </div>
    <div class="col-lg-4 col-md-4 col-sm-3">
        <img src="http://aprillancebees.com/wp-content/uploads/2013/08/brood.jpg" alt="bees aprillancebees.com" >
    </div>
</div>
<div class="row">
    <div class="col-lg-12 col-md-12 col-sm-12">
        That is, having groups of people work independently in order to achieve a set of common goals. In the same way, designing an algorithm for a problem that is decomposed into subproblems that execute in parallel and communicate results as necessary, might actually reach the goal in fewer steps. And there is a lot of parallelism in the world. Having raw <code>compute power</code> and clever <code>algorithm designs</code> can potentially deliver astonishing outcomes.
    </div>
</div></p>

To make matters more concrete, let's consider a CPU and a GPU architecture. An Intel `Core i7` Extreme Edition processor has $$8$$ cores, and each core can have $$2$$ threads in-flight to make more efficient use of the available resources. This results in `16 active threads` ready to be executed. An AMD `Radeon R9 270X` has $$20$$ cores. Each of these cores has $$4$$ SIMD units inside. Each SIMD unit has $$16$$ processing elements and operates on $$64$$ wide vectors. Also, each SIMD unit can handle up to $$10$$ vector threads. This amounts to $$\ 20\ cores $$ $$ *\ 4 \frac{SIMD\ units}{core} $$ $$ *\ 10 \frac{vector\ threads}{SIMD\ unit} $$ $$ *\ 64 \frac{elements}{vector\ thread} $$ $$ = 51200\ elements $$. So, an R9 270X GPU can potentially have up to `51200 active items` (in analogy to CPU threads) ready to be processed. Pay close attention to the number of available processing elements $$( 20\ cores $$ $$ *\ 4 \frac{SIMD\ units}{core} $$ $$ *\ 16 \frac{processing\ elements}{SIMD\ unit} $$ $$ = 1280\ p.e. )$$ and the effort being made $$( 10 \frac{vector\ threads}{SIMD\ unit} )$$ so that the entire GPU has always work to do.

<div class="row">
    <div class="col-lg-3 col-md-3 col-sm-3">
        <img src="http://images.anandtech.com/doci/8360/OpenCLLogo_678x452.png" alt="opencl anandtech.com" >
    </div>
    <div class="col-lg-6 col-md-6 col-sm-6">
        Finally, we get to the point where we ask the question: What is OpenCL™? <code>OpenCL</code> is a framework that allows the development of general purpose applications on heterogeneous systems.        
    </div>
    <div class="col-lg-3 col-md-3 col-sm-3">
        <img src="http://dn.odroid.com/homebackup/201407071058089142.jpg" alt="xu3 hardkernel.com" >
    </div>
</div>
A `heterogeneous system` is one that comprises different architectures, like a PC with a CPU and a GPU. It defines an abstract hardware model that you, as a developer, target when you write code. Then, if a vendor is interested in supporting the execution of OpenCL applications in their hardware, they take that abstract model and they map it to their own devices. So, OpenCL applications are `portable`! They can run on a PC or `embedded device`, unlike `CUDA` applications that they only execute on Nvidia GPUs. But sadly, the **performance** of OpenCL applications is not yet portable. This depends on the available resources, the underlining **memory system**, etc.

In any OpenCL application, there is a `host` (the CPU) that coordinates execution, and one or more `devices` (possibly GPUs) that execute the OpenCL code. OpenCL functions are called `kernels` and they are written in OpenCL C. `OpenCL C` is an extension over the C99 language.

An OpenCL kernel looks like the following. It's a standard C function with a few more `keywords` specific to OpenCL, and a number of `functions` that help the individual kernel instances to identify themselves and synchronize.

{% highlight cpp %}
__kernel
void vecAdd(__global int *A, __global int *B, __global int *C) {
  size_t idx = get_global_id(0);
  C[idx] = A[idx] + B[idx];
}
{% endhighlight %}

Let's say you want to do a vector addition. You write the kernel as if you have a single element to process, and then you **dispatch** that kernel for execution while specifying the number of **kernel instances** to launch. You do that and you watch a few million elements get processed **in less than a millisecond**.

OpenCL aims to accelerate your applications. It brings a range of architectures under the same environment. It gives you the ability to assign parts of your application to different processors, appropriate for certain kinds of computations so that you can benefit the most from each one of them.

You don't have to know anything about **parallel programming** to get started with OpenCL. You just need to find the enthusiasm and motivation necessary, because it's going to be a long ride. The place to start is one: <a href="http://www.amazon.co.uk/Heterogeneous-Computing-OpenCL-Revised-1-2/dp/0124058949/ref=sr_1_1?ie=UTF8&amp;qid=1420656418&amp;sr=8-1&amp;keywords=heterogeneous+computing+with+opencl">Heterogeneous Computing with OpenCL</a>. It will introduce you to OpenCL, the different architectures available today (like APUs), and you'll learn how to program in parallel. It has no prerequisites, and it will manage to answer all of your questions in due course. There is one section though in which it is seriously lacking: `buffers`. For this, you'll need to check the [AMD APP OpenCL Programming Guide](http://amd-dev.wpengine.netdna-cdn.com/wordpress/media/2013/07/AMD_Accelerated_Parallel_Processing_OpenCL_Programming_Guide-rev-2.7.pdf) (assuming you have an AMD GPU) on the subject of **OpenCL Data Transfer Optimization**. The next valuable resource is the [OpenCL Programming Guide](http://www.amazon.co.uk/OpenCL-Programming-OpenGL-Aaftab-Munshi/dp/0321749642/ref=sr_1_sc_2?ie=UTF8&amp;qid=1420657066&amp;sr=8-2-spell&amp;keywords=opencl+progamimng) book. It's an extensive API reference with lots of basic examples. But it's a bit old. It targets the OpenCL 1.1 specification. When everything else fails (or not), you turn to the [online documentation](https://www.khronos.org/registry/cl/sdk/1.2/docs/man/xhtml/) at khronos.org. And lastly, you'll probably want to print, laminate, and have by your side the reference cards for the OpenCL [API](https://www.khronos.org/files/opencl-1-2-quick-reference-card.pdf) and [C++ Wrapper](https://www.khronos.org/files/OpenCLPP12-reference-card.pdf).

The future is parallel. Whatever the framework you prefer, don't wait! Explore. Stay relevant.
