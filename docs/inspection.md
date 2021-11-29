<!-- class: invert -->

# Low-power camera motion tracking

### Powered by encoded motion vectors

By Aurimas Blažulionis

---

# Usecases

* CGI
* AR
* Robotics

---

# The why

* Conventional methods are computationally expensive.
* Video encoders perform similar computations to optical flow.
* Possible offload with hardware accelerators.

---

![bg contain](resources/video/unmoshed.webp)

---

![bg contain](resources/video/moshed.webp)

---

# Project

* Rust-based
* Motion vector extraction library
* Processing library
* Utilities/App

---

# Challenges

* Interpreting the data is hard.

---

![bg contain](resources/video/moshed.webp)

---

# Challenges

- Interpreting the data.

- Achieving high performance.

* Scene reconstruction.

---

# Progress

* Literature review

---

![bg contain](resources/video/robust_estimation.png)

---

![bg contain](resources/video/robust_estimation_impr.png)

---

![bg contain](resources/video/arbitrary_motion_estimation.png)

---

# Progress

- Literature review

* Motion vector extraction

* Hardware

---
