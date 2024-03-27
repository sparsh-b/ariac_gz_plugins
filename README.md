# To spawn the camera in an empty world
- Export the required environment variable to let Gazebo know where to find the model resources
  - `export GZ_SIM_RESOURCE_PATH=<path-to-this-repo>/models`
  - This Env variable can be used to point to world files & other resources as well.
- Spawn the world with model file in it
  - `gz sim -v 4 worlds/camera_world.sdf`

# To view the camera's feed:
- After gazebo spawns, Click the `...` symbol on top-right & search for `Image Display` plugin & add (by clicking) it.