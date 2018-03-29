---
layout: post
title: ROS Bag Recorder
subtitle: Record bags from your nodes
date: 2018-03-29 00:00:00
author: Nick Lamprianidis
header-img: 2018-03-29-ros-bag-recorder/logo.png
categories: [ blog, software, ros ]
tags: [ linux, ros, bag ]
---

Let's say you want to perform a test, record the events and data that present themselves, and later play them back, in order for you to take a closer look at what happened. In ROS, you can store messages, that get published to topics, to a [bag](http://wiki.ros.org/Bags) file. You don't need to know much about bags to get started, as there is a command line tool, [rosbag](http://wiki.ros.org/rosbag), to help you create and playback bag files.

Now, what if you wanted to have greater control over what gets stored and when? No worries, as bags have an API that allows you to create a file programmatically from any of your nodes. Consider the following example, taken from one of the [wiki](http://wiki.ros.org/rosbag/Code%20API#cpp_api) pages.

{% highlight cpp %}
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

rosbag::Bag bag;
bag.open("test.bag", rosbag::bagmode::Write);

std_msgs::String str;
str.data = std::string("foo");

std_msgs::Int32 i;
i.data = 42;

bag.write("chatter", ros::Time::now(), str);
bag.write("numbers", ros::Time::now(), i);

bag.close();
{% endhighlight %}

A file is opened, two messages of different types are written to the file, then the file is closed, and a bag now exists in the current working directory of the node. But given this API, a few questions arise. What if you wanted to record messages from many topics? You would have to add write calls to many different places, which is not ideal. What if you didn't know what are the topics you want to listen to at compile time? You would need to create a generic subscriber. What if you wanted to potentially save multiple bag files, but decide whether you actually do, based on an event? It would be nice if you could avoid opening all these files and writing to disk. 

Fortunately, there is the `rosbag::Recorder` class, which offers some of this functionality, and this means that you don't have to implement everything yourself. But the recorder has been designed to work with the `record` function of the rosbag tool. Even though there is a great architecture inside, it is not exposed in a way that would allow many different uses. So, in this post, I will describe how you can set the recorder up and how it works, and you can then take it and modify it as needed to fit your case.

It's pretty easy to start a recorder. You just have to initialize some options, give them to the recorder, and ask it to run. The recorder is meant to run until the node terminates. There is no stopping it.

{% highlight cpp %}
#include <rosbag/recorder.h>

rosbag::RecorderOptions options;
rosbag::Recorder recorder(options);
recorder.run();
{% endhighlight %}

The options are defined in the `rosbag::RecorderOptions` struct. There is no documentation on these parameters, apart from a short mention to some of them, [here](https://github.com/ros/ros_comm/blob/kinetic-devel/tools/rosbag/src/record.cpp#L50). So, we'll go through them one by one. But before we move forward, I'll have to describe two terms relating to bags, chunk and snapshot.

* Messages are written to files in [chunks](http://wiki.ros.org/Bags/Format/2.0). When a message is written to a `rosbag:::Bag`, it is stored in a buffer internally, and when the size of the buffer exceeds a threshold, this chunk of messages is moved to the file.
* "Triggering a snapshot" means taking all the messages received so far from a buffer and moving them to a bag file. The `rosbag::Bag` is created at the moment of triggering. This means all the messages stay in memory until the last moment, and if you let the recorder run for a long time, it might fill your RAM. So, keep that in mind when using snapshots.<br><br>

{% highlight cpp %}
struct RecorderOptions {
  bool            trigger;
  bool            record_all;
  bool            regex;
  bool            do_exclude;
  bool            quiet;
  bool            append_date;
  bool            snapshot;
  bool            verbose;
  CompressionType compression;
  std::string     prefix;
  std::string     name;
  boost::regex    exclude_regex;
  uint32_t        buffer_size;
  uint32_t        chunk_size;
  uint32_t        limit;
  bool            split;
  uint64_t        max_size;
  uint32_t        max_splits;
  ros::Duration   max_duration;
  std::string     node;
  unsigned long long min_space;
  std::string min_space_str;
  std::vector<std::string> topics;
};
{% endhighlight %}

We'll start with those parameters that are not used. These are `trigger`, `quiet`, and `name`. Considering the state of the recorder right now, I'm guessing that there was the intention to do more, but that never happened. If it works, don't mess with it. Right?

The rest are described as follows:

* `record_all`: The recorder subscribes to all topics. For that, it queries the master regularly. So, it subscribes to new topics as they appear on the network.
* `node`: The recorder subscribes only to the topics that this node subscribes to.
* `topics`: If `record_all` is false and `node` is empty, the recorder subscribes to the topics specified in this list.
* `regex`: If true, `topics` is expected to be a list of regular expressions. The recorder subscribes to the topics that match these regular expressions.
* `exclude_regex`: Any topic that matches this regular expression is excluded.
* `do_exclude`: Enables `exclude_regex`.
* `limit`: Limit in the number of messages to record per topic. When the limit is exceeded, the recorder unsubscribes from the topic.
* `verbose`: Outputs a log when a message is received.
* `prefix`: The start of the name of the bag file. If you want to store the bag file in a directory other than the current working one, you need to set here an absolute path, e.g. /tmp/mybag.
* `append_date`: Appends the date after the `prefix`, e.g. mybag_2018-03-29-19-42-38.
* `compression`: Compression type of the bag file. Either bz2 or lz4.
* `snapshot`: If false, when the recorder starts, a bag file is created. Then, when a message is received, the message is put in a buffer, a thread is notified, and the thread immediately writes the message to the bag. If snapshot is true, there is no bag and the message stays in the buffer. In this case, the recorder has a subscriber to `snapshot_trigger`. When a message is received to snapshot_trigger, a new buffer is created, and the contents of the old buffer are stored in a bag file.
* `buffer_size`: The size (in bytes) of the buffer that received messages are stored in. If exceeded, old messages start getting dropped.
* `chunk_size`: The size (in bytes) of the bag chunks.
* `split`: If true, messages can be split in multiple bag files.
* `max_splits`: Maximum number of splits.
* `max_size`: Maximum size of a bag. If exceeded, the recorder terminates, or a new split is created if `split` is true.
* `max_duration`: Maximum duration of a bag. If exceeded, the recorder terminates, or a new split is created if `split` is true.
* `min_space`: If the minimum free space (in bytes) on  disk is below this threshold, the recorder terminates.
* `min_space_str`: Just a string representation of `min_space`, used when writing logs.

That's it. You bother only with the options you want to modify, give them to the recorder, and let it run. Now, what are some potentials issues or restrictions if you want to use the recorder? I'll just mention a few:

* If you want to take snapshots, you are restrict to have only one recorder under a given namespace. Otherwise, when you want to trigger a specific recorder, you need to send a message to `snapshot_trigger` which will be received by all recorders under that namespace.
* If you want to take snapshots, and trigger them programmatically, you can call `doTrigger`. But there are two issues with this method. Even though it's a method of the recorder, it still triggers the snapshot by publishing a message (see above). Also, after publishing the message, it blocks (and that's not the only problem. Try it yourself and see. Hint: Two threads call ros::spin).
* When `max_duration` gets exceeded, the recorder is done. It would be nice if there was another parameter that worked like `limit`. That is, a rolling window is implemented, such that when time is exceeded, old messages start getting dropped.
* Once the recorder starts, you have no way to stop it other than to terminate the node.

By now, you should have an idea of why I messed around with the recorder in the first place. I wanted to have something that maintains the last few seconds of communication, and if an error happens, it saves that to a file. Thankfully, with all that's already in the recorder, it's not that hard to implement the rest.

So, the options are explained, the potential presented, and you can take that and do your own thing. See you in the next post.