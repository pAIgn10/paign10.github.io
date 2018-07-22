---
layout: post
title: Wacom Pen Tablets in Linux
subtitle: Configure your Wacom pen tablet
date: 2018-07-21 00:00:00
author: Nick Lamprianidis
header-img: 2018-07-21-wacom-pen-tablets-in-linux/logo.png
categories: [ blog, hardware, tablets ]
tags: [ wacom, review, setup ]
---

As a geek, I have bought many gadgets through out the years. Some were useful, other were useless. But by far, my most valuable possession is my Wacom tablet.

I hate mice. I find them limiting and awkward, and any ergonomic ones are often questionable and, especially in my case, not applicable, as I am left-handed. I've been disappointed in how I interact with my computer for a long time. I've also given trackballs a try, but things didn't get much better. After a while I always get tired. There is no point in trying to find that special mouse that can do the trick for you, as there's been forever a tool that's natural, perfectly fitted for the human hand, and offers the optimal control at your fingertips. This tool is the pen... and for the digital age, the pen tablet.

![intuos3-devianart]({{ "/assets/img/blog" | absolute_url }}/2018-07-21-wacom-pen-tablets-in-linux/intuos3.png)

With a pen tablet, you have a pen with which you interact and a pad which tracks the position of the pen. You have the total freedom in how you choose to hold and use the pen. You can interact with it for hours and never notice any fatigue. Of course pen tablets are not meant to replace mice, as they are not merely pointing devices, but since I wouldn't call myself a graphic designer, I won't touch the subject.

The number one brand in pen tablets is Wacom. I bought my first pen tablet in 2007 and it's a Wacom Intuos3 6x8. So I've been using this pen tablet for 11 years now, I'm completely satisfied and I've never had a problem with it. It hasn't even shown any signs of wear. I didn't even had to change the nib (tip of the pen)... as far as I can remember.

However great Wacom tablets are, sadly, Wacom doesn't have Linux support. Fortunately, Linux has communities, and one of them develops the [Linux Wacom](https://linuxwacom.github.io/) project. The project is actively maintained and offers excellent support. Honestly, my experience with the pen in Linux has been way more pleasant than what the official Wacom driver offers in Windows. The project includes a kernel driver, an X driver, and a metadata library. The kernel driver takes the hardware-specific events and translates them into standard input events. The X driver takes the aforementioned kernel events and translates into XInput events that are consumed by applications. The metadata library, libwacom, provides information about the connected Wacom devices. The kernel driver is already integrated into the Linux kernel. If you have an old pen tablet, you just plug it in and you are ready to go. If you have a newer model, you might need to update the Linux kernel and the components just mentioned.

Now let's see how we can configure a device. We can change many of the parameters, like the screen area that the pad maps to, whether the pad maps to the screen (absolute mode) or the pen is used as a mouse (relative mode), the orientation of the pad, the threshold value for registering a click (pens are pressure sensitive), etc. These can be set statically with a configuration file, or dynamically with a command line tool, `xsetwacom`, that the X driver offers. Some basic settings are also accessible through your desktop control panel. I didn't have to change much. I only set the relative mode for the pen, and tweaked its acceleration and sensitivity. So for the rest of this post, I'll only describe how I configured the buttons on the pen and the pad.

I mentioned earlier that the X driver produces X events. We can see these events with `xev`. Open a terminal, type xev, move the cursor on the Event Tester window, and start moving the pen and pressing the buttons. You'll see the events being printed out on the terminal with various relevant data.

```
MotionNotify event, serial 37, synthetic NO, window 0x4200001,
    root 0x28b, subw 0x0, time 12687069, (134,126), root:(1005,601),
    state 0x10, is_hint 0, same_screen YES

ButtonPress event, serial 37, synthetic NO, window 0x4200001,
    root 0x28b, subw 0x0, time 12688353, (134,126), root:(1005,601),
    state 0x10, button 12, same_screen YES

ButtonRelease event, serial 37, synthetic NO, window 0x4200001,
    root 0x28b, subw 0x0, time 12688498, (134,126), root:(1005,601),
    state 0x10, button 12, same_screen YES
```

The only interesting parameter for us about the button events is the `button`. This process is an easy way we can identify which button corresponds to which number. We need to write down this number as we'll use it in the next step to map a button to our own key combination. At the end of this step we end up with something like this.

```
Wacom Intuos3 6x8 pad
 ___________________________________________________________________________
|                                                                           |
|    ___ ___ ___     _____________________________________     ___ ___ ___  |
|   |   |   |   |   |                                     |   |   |   |   | |
|   |   | 1 | 4 |   |          0 - Button 3               |   | 6 | 8 |   | |
|   | 0 |___| + |   |          1 - Button 1               |   | + |___| A | |
|   |   |   | | |   |          2 - Button 2               |   | | |   |   | |
|   |   | 2 | + |   |          3 - Button 8               |   | + | 9 |   | |
|   |___|___| 5 |   |          4 - StripLeftUp            |   | 7 |___|___| |
|   |   3   |   |   |          5 - StripLeftDown          |   |   |   B   | |
|   |_______|___|   |          6 - StripRightUp           |   |___|_______| |
|                   |          7 - StripRightDown         |                 |
|                   |          8 - Button 9               |                 |
|                   |          9 - Button 10              |                 |
|                   |          A - Button 11              |                 |
|                   |          B - Button 12              |                 |
|                   |                                     |                 |
|                   |_____________________________________|                 |
|                                                                           |
|                                    WACOM                                  |
|                                                                           |
|                                                                           |
|___________________________________________________________________________|

Wacom Intuos3 6x8 stylus                 ________  ___
 __-------------------------------------/    3   \/ 2 \--------\__
|``                                                             _ > 1
 ``------------------------------------------------------------/
```

Now we can use the xsetwacom utility to set our desired key combinations. We can choose all the key combinations we want and put them in a script so we can enforce them any time we need, or have the script autostart with each Linux session.

The format in which we specify the key combinations is specific to the utility. Typing a name corresponds to a key press. Preceding the name with a plus sign means holding down a key, and preceding the name with a minus sign means releasing the key. Below you can see an example configuration.

<div  class="tex2jax_ignore">
{% gist nlamprian/fbd0623d0509b22af2469e9c283dcca3 intuos3-6x8-config.sh %}
</div>

We see how with each command we specify the device to which we are refering, the button on that device, and then the key combination it should map to. Once we execute the script, we can check the current configuration of the device by runninng:

{% highlight bash %}
$ xsetwacom -s get "Wacom Intuos3 6x8 Pad pad" all
Property 'Wacom Tablet Area' does not exist on device.
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "Button" "1" "key +Control_L +x -x "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "Button" "2" "key +Control_L +c -c "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "Button" "3" "key +Control_L +z -z "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "Button" "8" "key +Control_L +v -v "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "Button" "9" "key +Control_L +w -w "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "Button" "10" "key +Control_L +Tab -Tab "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "Button" "11" "key +Alt_L +F4 -F4 "
xs
If all is well, you should see a window with a ball flying around. Now you are done. Enjoy this beast of a machine and do your wonders.etwacom set "Wacom Intuos3 6x8 Pad pad" "Button" "12" "key +Alt_L +Tab -Tab "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "ToolDebugLevel" "0"
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "TabletDebugLevel" "0"
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "Suppress" "2"
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "RawSample" "4"
Property 'Wacom Pressurecurve' does not exist on device.
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "Mode" "Absolute"
Property 'Wacom Hover Click' does not exist on device.
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "Touch" "off"
Property 'Wacom Hardware Touch Switch' does not exist on device.
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "Gesture" "off"
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "ZoomDistance" "0"
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "ScrollDistance" "0"
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "TapTime" "250"
Property 'Wacom Proximity Threshold' does not exist on device.
Property 'Wacom Rotation' does not exist on device.
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "RelWheelUp" "1" "button +5 "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "RelWheelDown" "2" "button +4 "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "AbsWheelUp" "3" "button +4 "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "AbsWheelDown" "4" "button +5 "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "AbsWheel2Up" "5" "button +4 "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "AbsWheel2Down" "6" "button +5 "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "StripLeftUp" "1" "button +4 "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "StripLeftDown" "2" "button +5 "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "StripRightUp" "3" "button +4 "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "StripRightDown" "4" "button +5 "
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "Threshold" "0"
xsetwacom set "Wacom Intuos3 6x8 Pad pad" "BindToSerial" "0"
Property 'Wacom Pressure Recalibration' does not exist on device.

{% endhighlight %}

This was just a short introduction. There is plenty of documentation on [GitHub](https://github.com/linuxwacom) to find out everything you need about configuring your pen tablet.

On another topic, I just got a Wacom Intuos BT S as a more compact and wireless device that I can carry with me. Support for it only came recently. I don't know how well it works in Ubuntu 18.04, but in Ubuntu 16.04 it requires a lot of changes and I haven't tried all them yet. As it is in Ubuntu 16.04, it only works through the USB interface. If you update the Linux kernel to 4.17 or above, it will be seen also through Bluetooth, but it won't work properly. On the hardware side, compared with the Intuos3, as it has a slimmer frame around the active pad area, it's a bit uncomfortable that the hand has to sit on the table level. The pen is thinner which feels nice, but it's less grippy, so I often end up making an effort to keep it at the proper tilt angle. Also the pad doesn't have a scroll bar which is a big minus. Let's see if it will live up to the expectations of its predecessor.

That's all for now. See you in the next post.
