---
layout: post
title: Gigabyte Aero 15X
subtitle: Ubuntu 16.04 and CUDA 9.2 installation
date: 2018-07-07 00:00:00
author: Nick Lamprianidis
header-img: 2018-07-01-gigabyte-aero-15x/logo.png
categories: [ blog, hardware, laptop ]
tags: [ laptop, guide ]
---

Whenever you buy a new computer, you want to set it up, get it up and running as fast as possible, and start messing around with it. But this doesn't always happen, especially when you go with the cutting edge.

That is also the case with the Gigabyte Aero 15X. I received the laptop, unboxed it, turned it on, took a backup of the Windows 10 installation (with the provided software), and then it was time to wipe the drive clean and install Linux.

I plugged in a live Ubuntu 16.04 USB stick, went on with the installation, the installation finished and finally I logged in. The first thing to do? Install the graphics driver. And that is where the problems begin. Nothing with or around the graphics card would work correctly. I spent a good couple of weeks, with no luck, looking into it. I eventually came across a [reddit post](https://www.reddit.com/r/linux_gaming/comments/8gz8wy/installing_ubuntu_mate_1804_on_the_gigabyte_aero/) where a guy hinted an issue with the kernel version. And that was it. A few trials and errors later and the laptop was ready to fly. Below, I'll give you step by step instructions for setting up Ubuntu, and for installing the graphics driver and CUDA.

Download [Ubuntu 16.04](https://www.ubuntu.com/download/alternative-downloads) (or your favorite Ubuntu flavor) and write the image to a USB stick with [Etcher](https://etcher.io/).

Restart the computer, press F2 to enter to BIOS, go to the "Security" tab, and hit "Detele all Secure Boot variables". Plug the USB stick in and press F12 while the computer boots to set the USB stick as the first bootable device. Next, when you see the Ubuntu selection menu, move to the "Install Ubuntu" option and press E. A script will appear on the screen. Go to the line before the last one that begins with "linux ...", move to the end of the line, type "nouveau.modeset=0", and press F10 to boot.

```
linux /boot/vmlinuz-4.13.0-41-generic root=UUID=36286167-4eba-4a1e-a202-155c6baafa01 ro quiet splash $vt_handoff nouveau.modeset=0
```

Disabling nouveau is something that you'll have to do every time you boot Ubuntu until you install the Nvidia driver.

Now go ahead and install Ubuntu. At some point, you will be asked whether you want to disable Secure Boot. Agree to it and set a password. You will type this password next time you boot Ubuntu. Wait for the installation to complete, restart the computer and enter the password (you will be prompted). From now on when you start the computer, you'll see a message "Booting in insecure mode". Restart the computer, enter to BIOS, go to the "Security" tab, and hit "Reset Secure Boot variables".

Restart and hold the "Shift" key, so the grub menu appears. Press E and do the steps for nouveau as described above, and then you can log in Ubuntu as usual. Do any updates you have to do, and then you need to update the Linux kernel. At the time I did this, the installed version was 4.13.0-41, and I upgraded it to 4.15.0-15.

{% highlight bash %}
~ sudo apt install linux-image-extra-4.15.0-15-generic
~ sudo apt install linux-headers-4.15.0-15 linux-headers-4.15.0-15-generic
{% endhighlight %}

Now you can install the latest Nvidia driver.

{% highlight bash %}
~ sudo add-apt-repository ppa:graphics-drivers
~ sudo apt update
~ sudo apt install nvidia-396
{% endhighlight %}

To verify that the installation was successful, you should see the following output by running "nvidia-smi".

```
Sat Jul  7 15:13:37 2018       
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 396.24.02              Driver Version: 396.24.02                 |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  GeForce GTX 107...  Off  | 00000000:01:00.0 Off |                  N/A |
| N/A   49C    P8     8W /  N/A |    411MiB /  8119MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
                                                                               
+-----------------------------------------------------------------------------+
| Processes:                                                       GPU Memory |
|  GPU       PID   Type   Process name                             Usage      |
|=============================================================================|
|    0      1397      G   /usr/lib/xorg/Xorg                           214MiB |
|    0      2620      G   ...-token=4F0D8FD9540EF9E8ED262B8E5637035E   193MiB |
+-----------------------------------------------------------------------------+
```

That's it. Now the computer is ready. If interested, keep reading also to install CUDA.

Go to the [Nvidia CUDA Toolkit](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1604&target_type=runfilelocal) page, download and install the runfiles. The Nvidia driver is already installed, so when prompted, select no. You may also choose to install the samples.

{% highlight bash %}
~ sudo sh cuda_9.2.88_396.26_linux.run
~ sudo sh cuda_9.2.88.1_linux.run
{% endhighlight %}

Update your path variables so you can see the CUDA compiler and libraries.

{% highlight bash %}
~ echo -e "export PATH=$PATH:/usr/local/cuda-9.2/bin" >> ~/.bashrc
~ echo -e "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-9.2/lib64" >> ~/.bashrc
~ source ~/.bashrc
~ nvcc --version
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2018 NVIDIA Corporation
Built on Wed_Apr_11_23:16:29_CDT_2018
Cuda compilation tools, release 9.2, V9.2.88
{% endhighlight %}

To verify that CUDA works correctly, you can run one of the provided samples.

{% highlight bash %}
~ cd ~/NVIDIA_CUDA-9.2_Samples/5_Simulations/smokeParticles
~ make
~ ./smokeParticles
{% endhighlight %}

If all is well, you should see a window with a ball flying around. Now you are done. Enjoy this beast of a machine and do your wonders.