---
layout: project
title: SharpIRLib
description: Arduino library for interfacing with the Sharp IR range finders
date: 2013-11-14 00:00:00
categories: [ software, embedded ]
tags: [ c++, range-finder, infrared, model-fitting ]
filename: 2013-11-14-sharpirlib
img: sharp-ir.png
carousel: [ 1.jpg, 2.jpg ]
website: https://github.com/nlamprian/SharpIRLib
---

SharpIRLib is a project around the **Sharp IR range finders**. It consists of **two parts**. The first one allows to **fit a model** on the sensors' response characteristics. The second one is an **Arduino library** that offers an API for interfacing with the sensors.

**Infrared range finders**  are `proximity` sensors. They can be used in either **analog** mode to measure **distance**, or **digital** mode to detects **obstacles** in front of mobile robots. You can find some technical details on the sensors, [here](https://acroname.com/articles/sharp-infrared-ranger-comparison).

The `Curve Fitting` takes some measurements from a sensor, and then processes these data in either **MATLAB** or **Python** to extract a function mapping the voltage readings to some unit of length (**$$cm$$** or **$$in$$**). The `library` tries to abstract away the setup and operation of the sensor. It offers features such as **distance reading**, **averaging**, **change of reference point** and **obstacle detection**.

There is a **tutorial** on SharpIRLib on [codebender's blog](http://blog.codebender.cc/2014/01/23/sharpirlib/). The **source code** is available on [GitHub](https://github.com/nlamprian/SharpIRLib).
