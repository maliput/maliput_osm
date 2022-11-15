[![GCC](https://github.com/maliput/maliput_osm/actions/workflows/build.yml/badge.svg)](https://github.com/maliput/maliput_osm/actions/workflows/build.yml)

# maliput_osm

## Description

`maliput_osm` package is a [Maliput](https://github.com/maliput/maliput) backend implementation.
Its underlying format specification is based on [OSM-lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_maps) format.
Allowing the users to load a road network out of a OSM file description.

**Note**: For full information about Maliput please visit [Maliput Documentation](https://maliput.readthedocs.io/en/latest/index.html).

### Resources

`maliput_osm` provides several map resources at [maliput_osm/resources/osm](resources/osm).
These resources are:
 - OSM files for describing different road networks using [OSM-lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_maps) format specification.
 - YAML files for describing `maliput`'s road network information of type: Range Value Rules, Discrete Value Rules, Traffic Lights, Phase Rings, Intersections, etc.

Resources are installed natively, so the users are able to use them for their own interest.
In order to get the installation path check the environment variable: `MALIPUT_OSM_RESOURCE_ROOT`.

## API Documentation

Refer to [Maliput osm's Online API Documentation](https://maliput.readthedocs.io/en/latest/html/deps/maliput_osm/html/index.html).

## Examples

[Getting Started](https://maliput.readthedocs.io/en/latest/getting_started.html) page is a good place for starting to see Maliput's capabilities and how to use a Maliput backend for getting a road network.

 - [maliput_integration](https://github.com/maliput/maliput_integration): Concentrates applications created for maliput. See [maliput_integration's tutorials](https://maliput.readthedocs.io/en/latest/html/deps/maliput_integration/html/integration_tutorials.html). These applications allow to select `maliput_osm` as the backend.

## Installation

### Supported platforms

Ubuntu Focal Fossa 20.04 LTS, ROS2 Foxy.

### Binary Installation on Ubuntu

See [Installation Docs](https://maliput.readthedocs.io/en/latest/installation.html#binary-installation-on-ubuntu).

### Source Installation on Ubuntu

#### Prerequisites

```
sudo apt install python3-rosdep python3-colcon-common-extensions
```

#### Build

1. Create colcon workspace if you don't have one yet.
    ```sh
    mkdir colcon_ws/src -p
    ```

2. Clone this repository in the `src` folder
    ```sh
    cd colcon_ws/src
    git clone https://github.com/maliput/maliput_osm.git
    ```

3. Install package dependencies via `rosdep`
    ```
    export ROS_DISTRO=foxy
    ```
    ```sh
    rosdep update
    rosdep install -i -y --rosdistro $ROS_DISTRO --from-paths src
    ```

4. Build the package
    ```sh
    colcon build --packages-up-to maliput_osm
    ```

    **Note**: To build documentation a `-BUILD_DOCS` cmake flag is required:
    ```sh
    colcon build --packages-select maliput_osm --cmake-args " -DBUILD_DOCS=On"
    ```
    More info at [Building Documentation](https://maliput.readthedocs.io/en/latest/developer_guidelines.html#building-the-documentation).

For further info refer to [Source Installation on Ubuntu](https://maliput.readthedocs.io/en/latest/installation.html#source-installation-on-ubuntu)


### For development

It is recommended to follow the guidelines for setting up a development workspace as described [here](https://maliput.readthedocs.io/en/latest/developer_setup.html).

## Contributing

Please see [CONTRIBUTING](https://maliput.readthedocs.io/en/latest/contributing.html) page.

## License

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://github.com/maliput/maliput_osm/blob/main/LICENSE)
