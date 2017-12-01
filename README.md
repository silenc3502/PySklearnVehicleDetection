# Vehicle Detection
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


In this project, your goal is to write a software pipeline to detect vehicles in a video (start with the test_video.mp4 and later implement on full project_video.mp4), but the main output or product we want you to create is a detailed writeup of the project.  Check out the [writeup template](https://github.com/udacity/CarND-Vehicle-Detection/blob/master/writeup_template.md) for this project and use it as a starting point for creating your own writeup.  

Creating a great writeup:
---
A great writeup should include the rubric points as well as your description of how you addressed each point.  You should include a detailed description of the code used in each step (with line-number references and code snippets where necessary), and links to other supporting documents or external references.  You should include images in your writeup to demonstrate how your code works with examples.  

All that said, please be concise!  We're not looking for you to write a book here, just a brief description of how you passed each rubric point, and references to the relevant code :). 

You can submit your writeup in markdown or use another method and submit a pdf instead.

The Project
---

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

Here are links to the labeled data for [vehicle](https://s3.amazonaws.com/udacity-sdc/Vehicle_Tracking/vehicles.zip) and [non-vehicle](https://s3.amazonaws.com/udacity-sdc/Vehicle_Tracking/non-vehicles.zip) examples to train your classifier.  These example images come from a combination of the [GTI vehicle image database](http://www.gti.ssr.upm.es/data/Vehicle_database.html), the [KITTI vision benchmark suite](http://www.cvlibs.net/datasets/kitti/), and examples extracted from the project video itself.   You are welcome and encouraged to take advantage of the recently released [Udacity labeled dataset](https://github.com/udacity/self-driving-car/tree/master/annotations) to augment your training data.  

Some example images for testing your pipeline on single frames are located in the `test_images` folder.  To help the reviewer examine your work, please save examples of the output from each stage of your pipeline in the folder called `ouput_images`, and include them in your writeup for the project by describing what each image shows.    The video called `project_video.mp4` is the video your pipeline should work well on.  

**As an optional challenge** Once you have a working pipeline for vehicle detection, add in your lane-finding algorithm from the last project to do simultaneous lane-finding and vehicle detection!

**If you're feeling ambitious** (also totally optional though), don't stop there!  We encourage you to go out and take video of your own, and show us how you would implement this project on a new video!

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).



#SW Pipeline Mechanism
We have many main classes to implement this function.
CameraCal, ImageFilters, ProjectionManager, Lane, Line, Vehicle, VehicleTracking, VehicleDetection, RoadGrid, RoadManager, DiagManager, and Main is the most important class to implement this project.
Now, let's understand each classes.

1. CameraCal

__init__():
This function makes matrix to undist image for undistortion and save this parameters.

setImageSize():
This function save image size at img_size.

get():
This function is getter that can get mtx, dist, img_size for undistortion.

getall():
This function is getter that can get all of parameters.

2. ImageFilters

__init__(): This function do Camera Calibration and save parameters at mtx, dist, img_size.
And initialize value for processing video(Sky, Road Division, and Brightness info).
However this values doesn't have any meaningful value, just zero.

makehalf(): This function divide image half based on horizontal. It helps to divide Sky and Ground.

makefull(): This function is the result of merging the makehalf() and get rid of the Sky Image.

image_only_yellow_white(): This function helps to masking Yellow and White lane lines. So we can detect the multiple lane of roads.

gaussian_blur(): It's for applying Guassian Blurring for reduce noise of image and video.

canny(): It's for Canny Edge Filter with Thresholding.

abs_sobel_thresh(): This mechanism use Sobel X and Y Filter to detect gradient. Gradient Based System has good to detect variation rate.

mag_thresh(): This function calculate magnitude of gradients. We can calculate it with Pythagoras Theorem. After calculating apply this Thresholding.

dir_threshold(): This function calculate Direction of the Gradients. And apply it for Thresholding.

miximg(): This mechanism is same as Blending.

hls_s(): We can convert RGB to HLS and extract S Channel of HLS with this function.

hls_h(): We can convert RGB to HLS and extract H Channel of HLS with this function.

edges(): Detection Edge with Filter Combination.

imageQ(): Check the condition of the Image or Video for proper process.

horizonDetect(): This function use for detect Horizon with Sobel Magnitude.

projectionThrowDistanceDetect(): We can use it to find distance of the projection.

balanceEx(): This function separate each of RGB color channels and get Y Channel from YUV Color Space. And adjust brightness for more easy to detect lane lines.

applyFilter1(): This is the Filter Combination. Sobel X Filter with Threshold(25, 100), Sobel Y Filter with Threshold(50, 150), Magnitude of Gradient with Threshold(50, 250), Direction of Gradient with Threshold(0.7, 1.3), S channel from RGB to HLS Converter with Threshold(88, 190), and H Channel from RGB to HLS Converter with Threshold(50, 100).

applyFilter2(): It's for Sobel X Filter with Threshold(25, 100), Sobel Y Filter with Threshold(50, 150), Magnitude of Gradient with Threshold(50, 250), Direction of Gradient with Threshold(0.7, 1.3), S channel from RGB to HLS Converter with Threshold(88, 250), and H Channel from RGB to HLS Converter with Threshold(50, 100).

applyFilter3(): It's for Sobel X Filter with Threshold(25, 100), Sobel Y Filter with Threshold(50, 150), Magnitude of Gradient with Threshold(30, 150), Direction of Gradient with Threshold(0.6, 1.3), S channel from RGB to HLS Converter with Threshold(20, 100), and H Channel from RGB to HLS Converter with Threshold(125, 175).

applyFilter4(): It's for Sobel X Filter with Threshold(30, 100), Sobel Y Filter with Threshold(75, 150), Magnitude of Gradient with Threshold(30, 150), Direction of Gradient with Threshold(0.6, 1.3), S channel from RGB to HLS Converter with Threshold(20, 100), and H Channel from RGB to HLS Converter with Threshold(125, 175).

applyFilter5(): It's for Sobel X Filter with Threshold(25, 100), Sobel Y Filter with Threshold(50, 150), Magnitude of Gradient with Threshold(30, 150), Direction of Gradient with Threshold(0.5, 1.3), S channel from RGB to HLS Converter with Threshold(20, 80), and H Channel from RGB to HLS Converter with Threshold(130, 175).

setEdgeProjection(): This function use for setting the edge projection.

getEdgeProjection(): Get the edge projection.

setRoadProjection(): This function use for setting full road projection image.

getRoadProjection(): This is getter of above function.

drawHorizon(): This function is important to draw Horizontal Line.

3. ProjectionManager

__init__(): This function is the constructor of the ProjectionManager Class. Anyway it saves Camera Calibration info at camCal. And calculate Projection Mask Area and setting it to its own initial value. And save projectedX and projectedY to 1080 and 1920 for Full HD Image and Video. So it can do more good detection.

set_image_filter(): It's just setter for save ImageFilter instance.

region_of_interest(): This function create a ROI for processing our interesting area of Image and Video. Use cv2.fillPoly() and cv2.bitwise_and() to make masked info for proper image processing and video processing.

draw_area_of_interest(): Use cv2.line() to draw the Outline of Interesting Area.

draw_area_of_interest_for_projection(): It's almost same as draw_area_of_interest(). However there are some difference that is the thickness of the line for debugging.

draw_masked_area(): This function is same as draw_area_of_interest().

draw_bounding_box(): This function use for make Bounding Box.

draw_parallel_lines_pre_projection(): This function draw parallel lines at the perspective image. It'll be projected at flat plane.

draw_estimated_lane_line_location(): This function draw Estimated Lane Line with cv2.line().

draw_lines(): This function draw lines on the road. We can calculate slope and midpoint.

hough_lines(): We can create hough line with this function and calculate estimation of the lane lines. We can use cv2.HoughLinesP() for Hough Transform. This function based on Probabilistic Hough Transform and it returns start point and end point of line. And use draw_lines to draw hough lines.

unwarp_lane(): We can call this function to just inverse persepctive transform.

unwarp_lane_back(): Same as unwarp_lane(). However it's focus on to project the undistorted image to plane looking down.

find_lane_locations(): We can use this function to find starting lane line positions. After we found it then return left and right column positions.

hough_lines1(): This function is the custom hough line code for the other parameters.

hough_lines2(): Same as hough lines1()

hough_lines3(): Same as hough lines1()

hough_lines4(): Same as hough lines1()

hough_lines5(): Same as hough lines1()

findInitialRoadCorners(): We can use this function to find initial road corners to find a projection matrix. After we find corners then we can project edges into a plane. First we make vertices and make masking area to use region_of_interest(). After it, we use hough_lines() series to detect line image and lane info. After that process, we can calculate the Area of Interesting.

project(): This function makes edge projection at the Image plane. Process is same as findInitialRoadCorners() likes make vertices and masking edges. So we can get the Area of Interesting then now generate gray scaled image and full color projection image. Now we use cv2.solvePnP() to estimation the pose.

curWarp(): Inverse Perspective Transform function.

curUnWarp(): Perspective Transform function

setSrcTop(): We use it to avoid damping of car and the road.

setSrcTopX(): Same as setSrcTop().

resetDestTop(): Same as setSrcTop(). However there are some difference that is the condition. If top is too low then reset it.

pixel2Meter(): Pixel to meter distance converter function

wireframe(): This function's role is same as OpenGL's Graphic Library. np.poly1d() is 1 Dimension Polynomial Function to make various eqautions. We use it to fit road of left and right. After it we setting Horizontal lines with cv2.line() and calculate Vertical lines.

projectPoints(): We can use Z info to make Birds-Eye View to 3D Format by this function. 

drawCalibrationCube(): This function draw 3D Cube with above informations like corner locations. And we use cv2.drawContours() to get outline for drawing bottom and top of cube. And use cv2.line() to draw sides of cube.

drawRoadSquares(): We can draw the squares at the road with this function.

4. Lane

__init__():

confidence():

setLineIndex():

getLineIndex():

getLineBasePos():

drawLanePoly():

getRadiusOfCurvature():

setMaskDelta():

findInitialLines():

calculateXCenter():

bottomY():

findExistingLines():

5. Line

__init__():

createPolyFitLeft():

creatPolyFitRight():

updatePolyFitLeft():

updatePolyFitRight():

findBottomOfLine():

find_lane_nearest_neighbors():

setBasePos():

find_lane_lines_points():

scatter_plot():

polyline():

fitpoly():

fitpoly2():

applyLineMask():

applyReverseLineMask():

getLineStats():

getTopPoint():

requestTopY();

setMaskDelta():

radius_in_meters():

meters_from_center_of_vehicle():

6. Vehicle

__init__():

updateVehicle():

closest_colour():

get_colour_name():

madeColor():

distance():

sortByDistance():

unwarp_vehicle():

unwarp_vehicle_back():

findCenter():

findMaxColor():

sampleColor():

getTextStats():

windowCenter():

vehicleInBox():

objectIsVehicle():

drawClosingCircle():

calculateRoughtBoundingCubes():

calculateMask():

draw3DBoundingCube():

drawScanning():

takeProfileSelfie():

7. VehicleTracking

__init__():

isVehicleThere():

8. VehicleDetection

__init__():

set_threshold():

bin_spatial():

color_hist():

get_hog_features():

extract_features():

slidingWindows():

draw_boxes():

drawPlots():

detectVehicles():

collectData():

9. RoadGrid

__init__():

map_boxes():

getMapping():

setVehicle():

setFound():

setOccluded():

getKey():

getBox():

getAllWindows():

getBoxWindow():

getFoundWindows():

getOccludedWindows():

getFoundAndNotOccludedWindows():

getFoundAndNotOccludedWindowsInObject():

getFoundAndNotOccludedWindowsInVehicle():

getFoundAndNotOccludedBoxesInObject():

getFoundAndNotOccludedBoxesInVehicle():

gridCoordinates():

gridSize():

generatePolyRay():

getNumObjects():

getObjects():

getObjectList():

getObjectListWindows():

calculateVoxelOcclusionAndObjectSeparation():

calculateObjectPosition():

insertTrackedObject():

10. RoadManager

__init__():

addLaneLeft():

addLaneRight():

updateLaneLeft():

updateLaneRight():

findLanes():

drawLaneStats():

11. DiagManager

__init__():

textOverlay():

fullDiag():

projectionHD():

projectionDiag():

filterDiag():

12. Main

process_road_image():

process_image():
