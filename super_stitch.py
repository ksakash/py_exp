#!/usr/bin/env python3

"""
1. be able to reject the noise data, like the images which are not at a particular height
2. to be able to take dimensions of the area to be covered and resolution of the images
    to decide the height at which the quadcopters should hover to get the perfect images,
    also to know the grid size for a specific overlap of the images
3. to handle the images with low matches properly
4. enque all the images coming from the cameras
5. to be able to estimate the overlapping area in the "stitched image" with the "to be stitched" image,
    which can make the algorithm bit faster
6. changing the resolutin of the image remotely
7. to be able to know area which are covered and which are remaining
8. how to measure the overlap using the corners of the image
"""

"""
to make a detailed map of an area we need to define a parameter to measure the detail in a map.
we define a parameter, p:= number of pixels per meter (it will be different for horizontal and vertical).
let's say we fix the resolution of the image being taken according to our covenience:= 2000 x 1500
then following from the horizontal FOV that is 90 degrees, we get 2h meters of the area horizontally
where h is the height of the camera. so, 2000 pixels / 2h = p. we get h = 1000/p. simple

for grid size we can just use the 70% or more overlap rule and then set an upperlimit for the length of the grid.
following this approach we get unit sizes as 0.6h, where h is the height of the camera
"""

"""
one of the core problems of the problem is, how we find the overlap of the rectangle with other rectangle
there is a clipping algorithm for polygons which we can use and do the, luckily the algorithm is implemented
and available as a python lib: pyclipper (https://pypi.org/project/pyclipper/)
"""

"""
one of the problem frequently encountered is what do we do when the no. of matches are way less than ideal numbers.
low numbers causes bad warping of images which further decrease the quality of images to be added later
so it becomes important to formulate a protocol for deciding what to do in that condition
"""

"""
for stitching images coming from 4 cameras I will be using queue, but have to figure out how to
enque or stitch the images so that it can make a good stitched image
"""

def main ():
    pass

if __name__ == '__main__':
    main ()
