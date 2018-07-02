---
layout: post
title: Customize your PS1
subtitle: Towards a functional command prompt
date: 2015-02-02 00:00:00
author: Nick Lamprianidis
header-img: 2015-02-02-customize-your-ps1/logo.png
categories: [ blog, software, bash ]
tags: [ linux, bash, prompt, git ]
---

Yesterday I came across a [webcast](https://www.youtube.com/watch?v=U8GBXvdmHT4) for `git`. The host of this webinar set out to present git. He started with some background information and later he got to the point of setting up git. He opened his terminal, created a new repository, and cd'ed into the repo's directory. Then something beautiful happened. The name of his branch showed up in the `prompt statement`. It was such an obvious thing to do. Working on multiple branches and moving around, having to check every time if any files need to be committed or stashed can become tedious. But it doesn't have to be that way. The prompt statement can help us in this process. So, I spent the night looking into it.

<div><pre><code><span style='font-weight:bold;color:#dfcf2f'>my_repo </span><span style='font-weight:bold;color:#34d2d2'>(master*) </span><span style='font-weight:bold;color:#ad7fa8'>src/awesome_lib </span><span style='color:#33cc77'>_</span></code></pre></div>

Above is what came out of it. Firstly the name of the repository we are in appears. Then, inside the parentheses is the name of the branch we are currently checking out. That name may be followed by an asterisk. The presence of that `*` indicates that there are uncommitted/untracked files. `git status -s` is used for determining this, so you know what to expect. After that, we have the relative path we are in within the repository. And lastly, it's the prompt.

<div><pre><code><span style='font-weight:bold;color:#ad7fa8'>username@hostname</span><span style='font-weight:bold;color:#cfbf1f'>:</span><span style='font-weight:bold;color:#34d2d2'>~/working/directory </span><span style='font-weight:bold;color:#33cc77'>$ </span><span style='color:#33cc77'>_</span></code></pre></div>

And of course whenever we are outside of a git repository, the prompt "resets" and provides general information again.

This type of customization can extend to whatever other **distinguishable** directory categories you might have. You do that by adding another _elif_ condition within the _if_ statement of `set_ps1`.

<div  class="tex2jax_ignore">
{% gist nlamprian/f17caa014177742728a2 .bashrc %}
</div>

The secret behind this mod is the `PROMPT_COMMAND`. The contents of **PROMPT_COMMAND** are executed just before `PS1` is displayed. So, we include `set_ps1` within PROMPT_COMMAND in order for _set_ps1_ to be called the right time and set _PS1_.

Open your `~/.bashrc` in your favorite editor. Comment out any lines that try to set/modify **PS1**. Copy the gist above anywhere within **.bashrc**. Open a new terminal and you are done.

You can learn more about the prompt statement at [the geek stuff](http://www.thegeekstuff.com/2008/09/bash-shell-ps1-10-examples-to-make-your-linux-prompt-like-angelina-jolie/). On the other hand, if you don't wanna know anything about PS1, you can use this online [PS1 generator](http://bashrcgenerator.com/).

For more advanced features, take a look at [Oh My Zsh](https://github.com/robbyrussell/oh-my-zsh).

