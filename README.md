## About kinverse

Kinverse is the cross-platform lightweight library for solving inverse kinematics written in [C++17](https://en.wikipedia.org/wiki/C%2B%2B17).

## Building from source

Right now kinverse doesn't have any binaries for installation. The only way to use it is build from source using [Visual Studio 2019](https://visualstudio.microsoft.com/vs/) solution. In order to do that you need to install these third party libraries:

- [VTK 8.1.0](https://github.com/Kitware/VTK/releases/tag/v8.1.0) for visualization
- [google-test 1.10.0](https://github.com/google/googletest/releases/tag/release-1.10.0) for testing

In the near futute it is planned to ship project to [CMake](https://cmake.org/) build automation tool.

## License

The kinverse library is open-sourced software licensed under the [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause)

## Near future plans

- ship the project to [CMake](https://cmake.org/) build automation tool;
- get the [CI server](https://en.wikipedia.org/wiki/Continuous_integration) up and running;
- make a binary installer;
- create a website with tutorials, contribution rules, etc. We already have a domain name [kinverse.org](https://kinverse.org/);
- create a place for getting in touch with community.
