# Custom Gazebo Plugins
- Just used the Gazebo-Classic API to create my own custom "model plugins" for fun (I learnt most of my cpp while doing this)
- Feel free to go through but as of 2025 the Classic Version Plugins are depreciated
- The custom plugin here is quite extensive and can act as a template for many other applications you can think off
- There is a diff_bot plugin(ros2_control one added here by mistake) in the main branch and a custom mecanum bot plugin in the [mecanum_bot_gazebo_plugin branch](https://github.com/Labeeb1234/custom_gazebo_plugins/tree/mecanum_bot_gazebo_plugin)

- To use the plugin, download/clone the plugin to your ros2 workspace and the commands below
~~~ bash
## (build)
colcon build
## source the workspace
source install/setup.bash
## export the build plugin path to the GAZEBO_PLUGIN_PATH
export GAZEBO_PLUGIN_PATH=$HOME/gazebo_plugin_tutorial/build:$GAZEBO_PLUGIN_PATH
## rest is adding the plugin params in your URDF file just like any other Gazebo Plugin 
~~~

  > **Note** Will update a custom plugin version for Ignition-Gazebo or Gazebo Sim soon......
