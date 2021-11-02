# TrafficLightTask

ROS packages for very simple traffic light processing.

## Details

Repository contains 3 packages:

* **traffic_light_fetcher** - ROS package for simple traffic light fetching. 
Contains library with simple traffic light detection based on Hough transform 
and traffic light fetcher node that uses it. 

* **traffic_light_analysis** - ROS package for analyzing the traffic light 
fetching node results.

* **traffic_light_processing** - ROS meta-package for launching full traffic light
processing pipeline including traffic light fetcher and traffic light analysis.

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
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF
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

## Testing

If you want to run tests, you need to build packages with 
`CATKIN_ENABLE_TESTING=ON` (it is ON by default in ROS) and run `ctest`: 

```
catkin_make
cd build
ctest --verbose
```

## License

GNU General Public License v3.0.

## Maintainers

* Kirill Smirnov <smk.robotics@gmail.com>