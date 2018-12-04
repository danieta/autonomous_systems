import re
import numpy
import math

#libraries useful for showing images (debugging)
from scipy.misc import toimage
from matplotlib import pyplot


def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    
    return numpy.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))



# IMPORTANT NOTE: angles are assumed to be in radians
def ray_trace(map, pose, angle, threshold):
    #computes ray_angle from robot_angle and relative_angle
    ray_angle = pose[2] + angle
    
    #brings the ray_angle in the range 0-2pi
    ray_angle = ray_angle%(2*math.pi)
    if ray_angle < 0:
        ray_angle = 2 * math.pi - ray_angle
    
    # finds tan and cotan of the ray angle
    tan = math.tan(ray_angle)
    if tan != 0:
        cotan = 1/tan;
    
    map_height = map.shape[0]
    map_width = map.shape[1]
    
    start_pixel = [round(pose[0]), round(pose[1])]
    curr_pixel = [round(pose[0]), round(pose[1])] 
    
    map.flags.writeable = True
    #DEBUG: print("ray_angle:", ray_angle/math.pi*180,"tan:", tan)
    #       image.flags.writeable = True
    
    #while we are within the image boundaries
    while curr_pixel[0] < map_width-1 and curr_pixel[0] >= 0 and curr_pixel[1] < map_height-1 and curr_pixel[1] >= 0:
        
        if map[map_height-round(curr_pixel[1])-1,round(curr_pixel[0])] < threshold:    #if the inspected pixel is darker than a threshold returns the position of the pixel (collision detected)
           return math.sqrt((curr_pixel[0]-start_pixel[0])**2 + (curr_pixel[1]-start_pixel[1])**2) #euclidean distance
       
        map[map_height-round(curr_pixel[1])-1,round(curr_pixel[0])] = 0 # sets the inspected pixel to black (for debugging)
        
    #finds the next pixel to inspect:
        #if ray_angle is between -45 and +45 (included)
        if (ray_angle >= math.pi*7/4 or ray_angle <= math.pi/4):
            curr_pixel[0] += 1  #increment x
            curr_pixel[1] += tan  #increment y according to ray_angle
        
        #if ray_angle is between +45 and +135 (not included)
        elif (ray_angle > math.pi/4 and ray_angle < math.pi*3/4):
            curr_pixel[0] += cotan
            curr_pixel[1] += 1 #increment y
            
        #if ray_angle is between 135 and 225 degrees (included)
        elif (ray_angle >= math.pi*3/4 and ray_angle <= math.pi*5/4):
            curr_pixel[0] -= 1
            curr_pixel[1] -= tan
        
        #if ray_angle is between 225 and 315 degrees (not included)
        elif (ray_angle > math.pi*5/4 and ray_angle < math.pi*7/4):
            curr_pixel[0] -= cotan
            curr_pixel[1] -= 1
        
        else:
            return None

    return None



# for testing the ray_trace function
if __name__ == "__main__":
    relative_ray_angle = 0
    answer = "y"
   
    while answer == "y":
        map = read_pgm("map.pgm", byteorder='<')
        
        x = eval(input('x: '))
        y = eval(input('y: '))
        angle = eval(input('angle: '))
        
        my_pose = [x, y, angle]
         
        distance = ray_trace(map, my_pose, relative_ray_angle, 50)
    
        print("distance: ", distance)
        
        # plots image on the console (for debugging)
        pyplot.imshow(map, pyplot.cm.gray)
        pyplot.show()
        toimage(map).show()
        
        answer = input("continue? y/n")
        
     # DEBUG: plots image on a window
    #toimage(map).show()
            
    
