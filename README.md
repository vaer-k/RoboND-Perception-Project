## Project: Perception Pick & Place

---

## Rubric critera
### Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
For this section of the project, I began by downsampling voxels in the image to reduce the resolution of the image in order to facilitate a more tractable computational problem. This works by choosing a lower resolution space and using neighboring points to grid vertices to estimate a value for each voxel. Next, I applied a passthrough filter to extract only the region of the image where the table surface and objects were located for further analysis. 

At this point, I chose to diverge slightly from the basic instructions for the project to apply noise filtering to the image. Dust, poor lighting conditions, and camera lens flaws, among other things, can provide sources of error in the resultant image. We can improve the quality of the image by removing outliers. We do this by filtering out points that is more distant from its neighbors than the global average. 

After this initial filtering, I applied RANSAC plane segmentation to separate the objects on the table from the table itself. RANSAC filtering works by choosing a model of the target object, in this case a plane, and determining the minimal subset of points needed to represent it, in this case three points in cartesian coordinates. Next, a random sample of points are selected from the scene are selected and each point in the scene is tested to validate whether the point fits the model. This procedure is performed iteratively and finally the model parameters with the most consensus among points in the scene is selected. All of the points that coincide with the model are then considered to belong to the model, which in our case represents the table that the objects are sitting on. 

The following image is an example the resulting selection after all of the processing steps so far:
![ransac](http://imgur.com/XDBKibd.png)

And by selecting all of the points in the scene that are outliers to the final RANSAC-fitted model, we end up with the objects themselves, since there are all that is left in the scene after preprocessing. This will turn our image from this:

![raw](http://imgur.com/B019BL6.png)

into this:

![table-removed](http://imgur.com/NR1qTCC.png)

Right now, all of the objects have been selected from the scene, but they have yet to be distinguished from each other. We need to perform clustering to separate the pieces of the image to determine the set of points that represent each object.

### Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.
In order to distinguish each object from each other object, we'll need to perform some clustering. If we knew how many objects to expect in the scene we could use K-means clustering, but because this isn't known in advance for our application, we'll need to use a method that can detect a variable number of clusters automatically. The algorithm of choice here is called DBSCAN, and it works by exploring each point in the data and measuring its euclidean distance to its nearest neighbors. By tuning a couple thresholds, one to delineate the maximum distance required to be considered a neighboring point, and another threshold to delineate those core, defining points in a cluster, we are finally left with labeled points distinguishing member points of each object from those of another object. 

![dbscan](http://imgur.com/EOAzSHE.png)

### Complete Exercise 3 Steps. Features extracted and SVM trained. Object recognition implemented.
Next, we want to train a support vector machine model to learn the color and shape representation of each object. The colors are collect directly from the image, and the surface normals are computed from the depth sensor recordings to describe the shapes in the scene. The surface normal computations were provided by the given assignment materials.

The pattern of colors and surface normals of each object can be bundled into feaetures as histograms that serve as a kind of fingerprint for each object's appearance. We train the SVM model by providing example images of each object. After binning the values of each point in the image into histograms and concatenating the histogram values together, we feed the resultant feature vector into the training algorithm. The SVM learns boundaries separating various object classes from each other in feature space, and after training can be used to classify unseen images. 

The following confusion matrices describe the results of the SVM I trained for this dataset:
![confusion-matrix](http://imgur.com/CQK4qiT.png)
![normalized-confusion-matrix](http://imgur.com/FVkhzpG.png)

The clean diagonals across the table indicate successful classifications. 

### For all three tabletop setups (test*.world), perform object recognition, then read in respective pick list (pick_list_*.yaml). Next construct the messages that would comprise a valid PickPlace request output them to .yaml format.
I applied this processing pipeline to each of three test datasets, and was able to successfully identify the objects in each scene. Below are images from each test set with clearly labeled objects generated by this pipeline. The only missed objet is the "glue" in test world 3, which is partially occluded.  

![world1-labels](http://imgur.com/sQDSFN2.png)
![world2-labels](http://imgur.com/9GGvqhv.png)
![world3-labels](http://imgur.com/XPLd86p.png)

When I first ran this pipeline, I did it without the noise filtering, and furthermore I did not convert RGB values to HSV. Using HSV values to represent color values is more reliable because it is less sensitive to changes in lighting conditions, especially poor lighting conditions. Without these processes, the SVM model was trained with only an accuracy of less than 60% on the test set. Implementing the noise filtering and HSV conversion drastically improved the test set accuracy, resulting in a final score of 80.8%. This was accurate enough to successfully classify the objects in the given test world images, but was not enough to be successful in the challenge sets.

If I were to improve on this model, I would begin by refining the tuning of the SVM model. There are several parameters, including the choice of kernel and the level of regularization exacted, that could be better tuned to provide better results. It's also possible that another classifier algorithm entirely would do better to learn the image representations, so that is something I would like to explore as well. Finally, I could improve the results most easily and probably quite signficantly by providing many more training examples to the classifier. I used 50 different perspective images of each object to train representations, but using many more would likely improve the results signficantly.

---
# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  See the example `output.yaml` for details on what the output should look like.  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

# Project Setup
For this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly
If you do not have an active ROS workspace, you can create one by:

```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the src directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Perception-Project.git
```
### Note: If you have the Kinematics Pick and Place project in the same ROS Workspace as this project, please remove the 'gazebo_grasp_plugin' directory from the `RoboND-Perception-Project/` directory otherwise ignore this note. 

Now install missing dependencies using rosdep install:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```
Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/models:$GAZEBO_MODEL_PATH
```

If you haven’t already, following line can be added to your .bashrc to auto-source all new terminals
```
source ~/catkin_ws/devel/setup.bash
```

To run the demo:
```sh
$ cd ~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts
$ chmod u+x pr2_safe_spawner.sh
$ ./pr2_safe_spawner.sh
```
![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)



Once Gazebo is up and running, make sure you see following in the gazebo world:
- Robot

- Table arrangement

- Three target objects on the table

- Dropboxes on either sides of the robot


If any of these items are missing, please report as an issue on [the waffle board](https://waffle.io/udacity/robotics-nanodegree-issues).

In your RViz window, you should see the robot and a partial collision map displayed:

![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Proceed through the demo by pressing the ‘Next’ button on the RViz window when a prompt appears in your active terminal

The demo ends when the robot has successfully picked and placed all objects into respective dropboxes (though sometimes the robot gets excited and throws objects across the room!)

Close all active terminal windows using **ctrl+c** before restarting the demo.

You can launch the project scenario like this:
```sh
$ roslaunch pr2_robot pick_place_project.launch
```
# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  See the example `output.yaml` for details on what the output should look like.  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

For all the step-by-step details on how to complete this project see the [RoboND 3D Perception Project Lesson](https://classroom.udacity.com/nanodegrees/nd209/parts/586e8e81-fc68-4f71-9cab-98ccd4766cfe/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/e3e5fd8e-2f76-4169-a5bc-5a128d380155/concepts/802deabb-7dbb-46be-bf21-6cb0a39a1961)
Note: The robot is a bit moody at times and might leave objects on the table or fling them across the room :D
As long as your pipeline performs succesful recognition, your project will be considered successful even if the robot feels otherwise!
