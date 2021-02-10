---
layout: post
title: paper222
description: >
  Visual Computing is a module I took this semester at NUS ECE, hosted by Professor Robby Tan. It covers some of the most classic CV algorithms.
image: /assets/img/img-ee5731/bg.jpg
sitemap: false
---

These years, rapid progressions in Deep Learning have brought enormous improvements to the vision-based applications. DL methods, mainly based on CNN frameworks, are constructed usually by training instead of the domain-specific designing and programming in the traditional CV algorithms. They are easy to build, often achieve better accuracy, and the trained models can be very lightweight. So why do we still need to learn about traditional CV methods?

Unlike the black box models in machine learning, traditional CV algorithms are more explainable and tunable. They are also more general since the design of the feature extraction process doesn't rely on any specific image dataset. Therefore, algorithms like SIFT are often used for image stitching and 3D mesh reconstruction, which don't require any class knowledge. In fact, DL can sometimes overkill, while many traditioal CV techniques can also be simplified and implemented on devices like microcontrollers.

The EE5731 Visual Computing module provides us with knowledge on topics including:

* Viola Jones face detection algorithm
  * Haar-like features
  * Adaboost
  * Integral image
* HOG (Histogram of Oriented Gradients) features
* SIFT (Scale Invariant Feature Transform) algorithm
  * Scale space extrema - DoG (Difference of Gaussian)
  * Keypoint localization
    * Contrast threshold
    * Edge threshold
  * Orientation assignment
  * SIFT descriptor
  * Homography
  * RANSAC
* Camera parameters
  * Forward propagation
  * Backward propagation
* Depth from stereo
* Markov random field
* Depth from video
* Optical flow
* Structure decomposition
