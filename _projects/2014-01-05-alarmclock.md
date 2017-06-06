---
layout: project
title: AlarmClock
description: Arduino sketch that makes use of an LCD Keypad Shield and a DS1307 RTC to build an alarm clock
date: 2014-01-05 00:00:00
categories: [ software, embedded ]
tags: [ c++, alarm, clock, fsm ]
filename: 2014-01-05-alarmclock
img: alarmclock.png
carousel: [ 1.png, 2.png, 3.png ]
website: https://github.com/nlamprian/AlarmClock
---

AlarmClock is an **Arduino sketch** that implements an **alarm clock**.

AlarmClock comprises an **LCD Keypad Shield** for the IO operations, a **DS1307 RTC** for the timekeeping, and a **buzzer** for the alarm.

The **control** is managed by a **finite state machine**. The `FSM` greatly helps to organize the **operation** of, and **transitions** between, the different states of the clock and the various menus.

There is a **tutorial** on AlarmClock on [codebender's blog](http://blog.codebender.cc/2014/01/16/alarmclock/). The **source code** is available on [GitHub](https://github.com/nlamprian/AlarmClock).
