<!-- class: invert -->

# Optical Flow Processing Stack

### And use of MPEG motion vectors

By Aurimas Blažulionis

---

# Table of contents

- Background
- OFPS
- Detection
- Tracking

---

# Camera tracking

* Position in 3D space.
* Rotation in 3D space.
* Changes over time.

---

# The case for tracking

* CGI
* AR
* Robotics

---

# State-of-the-art

### MaskFlowNet+VOLDOR

---

# MaskFlowNet

* Very high accuracy.
* Very slow.

![contain](resources/video/maskflownet.webp)

---

# VOLDOR

* High efficiency.
* High accuracy.

![contain](resources/demo/voldor.png)

---

# The problem

### Finding features (flow) is expensive.

---

# MPEG

* Video compression standard.
* Hardware accelerated.
* **Motion gets encoded.**

---

# I-Frames

* Essentially JPEG.

---

# P-Frames

* Predicted frame.
* Changes since previous frame.
* **Motion.**

---

# B-Frames

* Bi-directional predicted frame.
* Not useful for realtime operation.

---

# Extracting the motion

* FFMPEG/LibAV.
* Custom decoder.

---

# Resulting performance

* MaskFlowNet CPU: 2147ms
* MaskFlowNet GPU: 299ms
* OpenCV: 45.679ms
* LibAV: 13.36ms
* (160x faster than MaskFlowNet)

---

# What's the potential?

* Detecting motion?
* Tracking camera motion?
* SLAM?

---

# Plan

* Integrated processing stack.
* Accompanying app.

---

# OFPS

* Abstract interfaces.
* Modular.
* Extensibile.
* Dynamic plugin loader.

---

# OFPS architecture

![contain](resources/demo/architecture.png)

---

# Motion detection

---

![bg contain](resources/demo/detection-1.jpg)

---

![bg contain](resources/demo/detection-2.jpg)

---

![bg contain](resources/demo/detection-3.jpg)

---

![bg contain](resources/demo/detection-4.jpg)

---

# Live demo

---

# Predicting with VOLDOR

* Does not work.

---

# MPEG vs. MaskFlowNet

---

![bg contain](resources/video/maskflownet_compare.webp)

---

# Other approaches

* Blender's libmv.
* OpenCV's 5-point algorithm.
* Almeida's estimator.

---

# Conclusions

* Tracking camera pose with 6DoF is too complex.
* Constraining to just rotational tracking makes it feasible.

---

# Conclusions

### Rotational tracking avg. error

\# | Almeida | Almeida RANSAC | libmv | OpenCV
-----|:----:|:-----:|:-----:|:----:
MPEG | 0.024 | 0.016 | 0.096 | 0.19
Farneback | 0.015 | 0.015 | - | -

---

# Conclusions

### Rotational tracking perf.

\# | Almeida | Almeida RANSAC | libmv | OpenCV
-----|:----:|:-----:|:-----:|:----:
MPEG | 2.4ms | 12.9ms | 45.6ms | 118.2ms
Farneback | 45.1ms | 53.24ms | 20+s | 20+s

---

# Live Demo

---
