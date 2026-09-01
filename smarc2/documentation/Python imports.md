# Python imports
Welcome (back) to the "but it worked so well with ROS1!" camp. Here you will find some hair-pulling avoidance tips for python imports. These are functional simplifications that'll get you going without reading language and ros specs.

**What?** If you import like `from myfile import mything` with ROS2, this file won't be able to import when you `colcon build` and launch the node. If you import like `from .myfile import mything` (notice the dot) then it will run fine with a launch, but now you can not run the file with just `python3 file.py`.

**Why?!** It worked in ROS1 because of a hack that catkin allowed. Colcon does not allow the same hack, thus it doesn't work the same way with ROS2. 

This is consistent with the general theme of bringing C++ and Python closer together within the context of ROS2. Like with C++, you now need to build and use launch files to test python stuff.

## Dirty workaround
It wouldn't be python without a funny workaround:
```python
try:
    from myfile import mything # for terminal
except:
    from .myfile import mything # for ROS2
```

## Slightly less dirty
If some imports are ONLY used for running from the terminal, you can also put them under the main: `if __name__ == "__main__":`, these imports will only be ran when you run the file by itself and not when ROS runs it.

## How about folders?
If you want something like this:
```
colcon_ws
|- yourpackage
|  |- yourpackage
|  |- __init__.py
|  |- main.py
|  |- module_folder
|  |  |- __init__.py
|  |  |- module1.py
|  |  |- module2.py
```

And you want to import like `from module_folder.module1 import func` in `main.py` then you need to tell colcon that `module_folder` is in fact a module folder. 

1) Notice the `__init__.py` in `module_folder`.
2) Add that folder to your `setup.py` like so: `package = [yourpackage, yourpackage/module_folder]` (this line should be there if you used `ros2 pkg` to create your package)
3) Import as `from .module_folder.module1 import thing` (period needed for ROS2, shouldn't be there for non-ros2)




