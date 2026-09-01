# Porting a package
[These notes](./media/SAM%20Humble%20Port.png) provide a journal-like graphic of Ozer's experience porting some packages to ROS2.
Here, they will be expanded and itemized for ease of reading.







## Step 1: At least skim [the official migration guide](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html)
The [official guides](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html) are already written in an almost step-by-step fashion, but it can be daunting to go through _all_ of it in one go without guidance.
Having a look at it will allow you to quickly go back and forth as you progress.
I have tried all the automatic tools mentioned here, but none worked for me.



## Step 2: Package artifacts
I have found it to be easiest to create a new package from scratch and then move over files and their insides one by one.
Check the [new package tutorial](./Making%20a%20new%20package.md).
This will give you a valid ROS2 package that will compile.

### Package.xml
There is little change to this file from ROS1, and all the changes are already done for you if you followed the [new package tutorial](./Making%20a%20new%20package.md).
You only need to move over the dependencies you had in ROS1 over here.

### CMakeLists.txt
If you have this, this is where most of the changes are.
Check [the ported package](../examples/sam_basic_controllers/CMakeLists.txt) to see how both C++ and Python stuff is defined now.

#### Setup.py
It should be the case that you don't have a CMakeLists.txt file for your pure-python package. You should instead have `setup.py`.
This is a simpler file to modify, with basically only a list of files that you want installed and where they should go.
If you compare [the example here](../gui/smarc_nodered/setup.py) to your auto-generated one, you will see that you simply need to tell setuptools to copy things around.

**Launch, config etc:** Any launch files, config files and such should be listed under `data_files=...`. 

**Executables:** These need to go under `entry_points=...`. These are the things that will be runnable using `ros2 run` and from launch files. Notice that the executable names do not need to match the file name, function name, or aynthing really, it is just an identifier that you are giving to the fully-qualified path of a specific function.


## Step 3: Launch files
There are quite a few changes to be made here.
In my experience, the following replacements make the bulk of the edits you need to make:
- `$(arg ...)` --> `$(var ...)`
- `<node ... ns= ...` --> `<node ... namespace= ...`
- `rosparam` --> `param`
- `$(find ...)` --> `$(find-pkg-share ...)` (There are other `find-` commands availble now, but the most commonly used one is the package share, where launch files, executables and such are placed)
- `<group ns=...` --> `<group .../> <push-ros-namespace .../>` (The `ns` argument is separated from the `group` tag into its own tag `push-ros-namespace` which works in the same way.)

You can use [my notes](./media/SAM%20Humble%20Port.png) to see what I did to fix many other common errors you might face, rather than googling.

**Python:** Another change is how Python nodes are handled. You can not just use the filename to launch a node, you must first define the executable in `setup.py` and use the name you gave the executable there in the launch file, just like you would do for C++ nodes.

### Params
This is also the time to think about if all these parameters you are passing (particularly the topics, see [Topics.msg](../messages/README.md#topicsmsg)) are needed. 
ROS2 has removed the notion of a "global parameter server", so all params are now local to a node.
This might require changes to your code.

If you have a long list of parameters, it might be worthwhile to put them into a yaml file instead.
That way you can pass different sets of parameters easily.
Yaml parameter files also allow you to use ONE file for many nodes' parameters. Check [the official docs](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1/Migrating-Parameters.html).



## Step 4: Actual code
**This is the perfect time to do some clean-up!**

**Recommended design pattern:** Please check [the model-view-controller design](./media/ROS2%20Node%20Design.png) and implement it in your new and shiny ROS2 package, such that we reduce the likelihood of ending up in the same spaghetti we were in ROS1 ;)

**Example:** [This minimal ros2 example](../examples/ros2_python_examples/ros2_python_examples/) should provide you the general idea of how and why. It is heavily commented, with multiple different examples of doing the same thing for you to pick and choose.

**Python imports:** Python3 under ROS2 needs some amount of extra care when importing things. Read [this guide here](./Python%20imports.md) to see how/why/what.

The official [python guide](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1/Migrating-Python-Packages.html) and [cpp guide](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1/Migrating-CPP-Packages.html) are all very clear as well.



## Step 5: Documentation
**A readme is a must!**

Check [this opinion piece](./Writing%20a%20nice%20README.md) about how to write a README that will be read.

At the very least, your readme should contain:
- Topics you use
- Services you provide, with example usage
- Parameters that matter
- External stuff needed to compile/run




## Step 6: Tests
Since you probably had to change a pile of stuff, and even (**hopefully**) changed the architecture a little, this is the perfect opportunity to write some tests for your stuff!

Again, [this minimal ros2 example](../examples/ros2_python_examples/test) is relevant. 
[test_testing.py](../examples/ros2_python_examples/test/test_testing.py) shows the absolute minimum examples of two tests, one that fails and one that passes all the time.


> TODO(Ozer) integration tests
