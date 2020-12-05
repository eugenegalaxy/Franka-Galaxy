<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->

<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/eugenegalaxy/Franka-Galaxy">
    <img src="doc/logo_m.jpg" alt="Logo" width="600" height="424">
  </a>

  <h3 align="center">Galaxy Franka Driver</h3>

  <p align="center">
    Your best chance at controlling Franka Emika Panda beasts.
    <br />
  </p>
</p>

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

Hey everyone, I am Eugene `Galaxy` Galaktionovs. This project is a collection of functions to control Franka Emika robots via code. The purpose of this repository is to collect common Franka functions under one source and use it as a driver submodule in other projects. The project consists of two main points:
- C++ 'driver' containing common robot commands, such as move, set parameters, read states, etc.
- Python Web API containing methods to send requests to Franka Desk web browser API. For example, you can open and lock brakes on Franka robot.

<!-- GETTING STARTED -->
## Getting Started

This repository is made in ROS infrastructure, even though not much of ROS is used at the moment (but will be in the close future).
To use this repo, one must clone it to *catkin_workspace/src/* directory, and *catkin_make* it. But *OPS!*, it won't compile because one is missing dependencies. Read below about them.

### Prerequisites
- Linux with patched Kernel for Preemtiveness. See *doc/alternative_real-time_kernel_INSTALLATION_process* for instructions.
- ROS Melodic (maybe earlier versions too, don't know)
- cmake at least3.10
- Following packages are all dependences of submodule 'frankx' and must be installed on your system:
  1. Eigen v3.3.7
  2. Libfranka v0.7.1
  3. Pybind11 v2.6.0
  4. Catch2 v2.9 (only for testing)
See `doc/install_frankx.txt` for some instructions (NEEDS UPDATE)

<!-- USAGE EXAMPLES -->
## Usage

TBA

_For more examples, please refer to the `doc` directory._



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request



<!-- LICENSE -->
## License

Distributed under **TBA**. See `LICENSE` for more information.



<!-- CONTACT -->
## Contact
Jevgenijs Galaktionovs - [Really A Robot](www.reallyarobot.com) - jga@reallyarobot.com
