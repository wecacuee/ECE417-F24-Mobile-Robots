---
layout: post
title: ECE417  2022  Lab 8, Computing homography
date: 2022-04-04
usemathjax: true
---

# Summary of steps
{:.no_toc}

* Seed list to be replaced by TOC
{:toc}

# Removing perspective


## Get a picture with perspective effect

Click a picture of the lab so that windows or cupboards
have a perspective effect. Perspective effect is when parallel lines in the 3D world are not parallel, instead converging in the image.

![](code/data/removing-perspective-distortion.png)

``` shellsession
mkdir -p ~/04-04/code/data
```

Somehow (by emailing it to yourself?) get the image to the linux and save the image as `~/04-04/code/data/removing-perspective-distortion.png`.
