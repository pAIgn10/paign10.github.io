---
layout: project
title: ArduinoPixel
description: Android app that communicates with an Arduino web server to control a NeoPixel LED strip
date: 2014-03-24 00:00:00
categories: [ software, android ]
tags: [ arduino, android, iot, distributed ]
filename: 2014-03-24-arduinopixel
img: arduinopixel.png
carousel: [ 2.jpg, 1.jpg ]
website: https://play.google.com/store/apps/details?id=ln.paign10.arduinopixel
---

This project is a server-client system that aims to control a NeoPixel LED strip. The server part comprises an `Arduino` sketch that implements a `Web Server` and offers an `API` for controlling the [NeoPixel LED Strip](http://www.adafruit.com/products/1138). The client is an `Android` app, named `ArduinoPixel`, that connects to the server and makes API calls to control the **color** and the **on/off state** of the **LED strip**.

<iframe width="560" height="315" src="https://www.youtube.com/embed/AuqOQ0Pe_c0?showinfo=0&amp;theme=light" frameborder="0" allowfullscreen></iframe>

The client communicates with the server through the following **HTTP requests**:
* `GET` request to `/`. The server responds with a **Hello from Arduino Server** message.
* `GET` request to `/strip/status/`. The server responds with **ON** or **OFF** for the on/off state of the strip.
* `GET` request to `/strip/color/`. The server responds with a **JSON representation** of the strip's **color**, e.g. `{"r":92,"g":34,"b":127}`.
* `PUT` request to `/strip/status/on/`. The server **turns** the strip **on**.
* `PUT` request to `/strip/status/off/`. The server **turns** the strip **off**.
* `PUT` request to `/strip/color/`. The server **changes** the strip's **color**. The **data** are delivered as a **JSON object**, e.g. `{"r":48,"g":254,"b":176}`.

I've also written a **script** on **Linux** that **automates** the turning on and off of the strip with the start and shutdown of the computer during the **night**. It will be available soon.

You can find the **Arduino sketch** on [codebender](https://codebender.cc/sketch:31742) and the **Android app** on [Google Play](https://play.google.com/store/apps/details?id=ln.paign10.arduinopixel). The **source code** is available on [GitHub](https://github.com/nlamprian/ArduinoPixel).

**[humblehacker](https://codebender.cc/sketch:62851)** at codebender has forked the **Arduino sketch** and adapted it for `Arduino Yun`!
