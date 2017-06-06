---
layout: project
title: LiFuelGauge
description: Arduino library for interfacing with the MAXIM MAX17043 / MAX17044 Li+ fuel gauge
date: 2014-01-10 00:00:00
categories: [ software, embedded ]
tags: [ c++, lipo, battery ]
filename: 2014-01-10-lifuelgauge
img: lifuelgauge.png
carousel: [ 1.png, 2.png ]
website: https://github.com/nlamprian/LiFuelGauge
---

LiFuelGauge is an **Arduino library** that offers an API for interfacing with the **MAXIM MAX17043/MAX17044 Li+ fuel gauges**.

MAXIM MAX17043/MAX17044 Li+ fuel gauges are devices that determine the **state of a single cell LiPo battery**. They offer measurements of the battery's **voltage** and determine the battery's **state of charge** (SoC) relative to its maximum capacity. They are also able to output an **alert** signal when a **critical SoC** threshold is reached. They are useful devices for situations like when an important operation is about to begin on a computing device, and so it's imperative to confirm that the energy requirements are met.

LiFuelGauge abstracts away the process of communicating with a **MAX17043 / MAX17044 fuel gauge**. It simplifies the setup and management of the module by offering a convenient interface for controlling its various parameters.

There is a **tutorial** on LiFuelGauge on [codebender's blog](http://blog.codebender.cc/2014/02/13/lifuelgauge/). The **source code** is available on [GitHub](https://github.com/nlamprian/LiFuelGauge).
