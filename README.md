![EdynLogo](https://user-images.githubusercontent.com/762769/88462008-1149e000-ce6e-11ea-9ffd-218d935c0be7.png)

_Edyn_ (pronounced like "eh-dyin'") stands for _Entity Dynamics_ and it is a real-time physics engine organized as an ECS (Entity-Component System) using the amazing [EnTT](https://github.com/skypjack/entt) library. The main goals of this library is to be multi-threaded and to support networked and distributed physics simulation of large dynamic worlds.

It is still in an early stage of development and is not yet ready for use. Feel free to explore and contribute meanwhile.

Examples are located in a separate repository: [Edyn Testbed](https://github.com/xissburg/edyn-testbed)

## Build Instructions

### Requirements

A compiler with C++17 support is required, along with `CMake` version 3.12.4 or above.

### Steps

In the _Edyn_ directory:

```
$ mkdir build
$ cd build
$ conan install ../conanfile.txt
$ cmake ..
$ make
```
