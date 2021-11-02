# TrafficLightTask

ROS packages for traffic light detection and analysis.

## Building

For building traffic light processing packages from this repo you need:

* Create ROS workspace:
    
    ```
    mkdir traffic_light_workspace && cd traffic_light_workspace
    mkdir src && cd src
    ```

* Clone this repository to *src* directory:
    
    ```
    git clone https://github.com/smk-robotics/TrafficLightTask.git
    ```

* Build all packages using *catkin_make*:
    
    ```
    cd ..
    catkin_make -DCMAKE_BUILD_TYPE=Release
    ```

## Running

To launch full traffic light processing pipeline you need to run:

```
source devel/setup.sh
roslaunch traffic_light_processing full_traffic_light_processing.launch
```

If you want to launch traffic light fetcher ROS node standalone run:

```
source devel/setup.sh
roslaunch traffic_light_fetcher traffic_light_fetcher.launch 
```

If you want to launch traffic light analysis ROS node standalone run:

```
source devel/setup.sh
roslaunch traffic_light_analysis traffic_light_analysis.launch
```

## License

GNU General Public License v3.0.

## Maintainers

* Kirill Smirnov <smk.robotics@gmail.com>