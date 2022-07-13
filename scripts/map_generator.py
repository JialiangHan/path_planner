# this file is  to generate map for hybrid a star planner,use pillow
from PIL import Image
from PIL import ImageDraw
import random
import math

def distance(ax:float,ay:float, bx:float, by:float)->float:
    """finds the straight-line distance between two points

    Args:
        ax (float): x coordinate for point a
        ay (float): y coordinate for point a
        bx (float): x coordinate for point b
        by (float): y coordinate for point b

    Returns:
        float: distance from point a to b
    """    
    return math.sqrt((by - ay)**2 + (bx - ax)**2)

def rotated_about(ax, ay, bx, by, angle):
    """rotates point `A` about point `B` by `angle` radians clockwise.

    Args:
        ax (float): x coordinate for point a
        ay (float): y coordinate for point a
        bx (float): x coordinate for point b
        by (float): y coordinate for point b
        angle (float): here angle is in degree

    Returns:
        tuple: a x,y coordinate after rotation
    """    
    radius = distance(ax,ay,bx,by)
    angle += math.atan2(ay-by, ax-bx)
    return (
        round((bx + radius * math.cos(angle)),2),
        round((by + radius * math.sin(angle)),2)
    )

# draw a obstacle(rectangle) on a given image,
def DrawObstacle(img,x,y,width,height,mode):
    """draw a rectangle on img
    Args:
        img (_type_): pillow image
        x (_type_): rectangle center x
        y (_type_): rectangle center y
        height (_type_): rectangle height
        width (_type_): rectangle width
        angle (_type_): rotation angle: in degree
    Returns:
        img: pillow img after drawing rectangle
    """    
    img1=ImageDraw.Draw(img)

    rectangle_center = (x,y)

    rectangle_vertices = (
    (rectangle_center[0] + width / 2, rectangle_center[1] + height / 2),
    (rectangle_center[0] + width / 2, rectangle_center[1] - height / 2),
    (rectangle_center[0] - width / 2, rectangle_center[1] - height/ 2),
    (rectangle_center[0] - width / 2, rectangle_center[1] + height/ 2)
)
    # for coordinate in rectangle_vertices:
    #     print(coordinate)
    if mode==0:
        angle=0
    if mode==1:
        angle=90
    rectangle_vertices = [rotated_about(x,y, rectangle_center[0], rectangle_center[1], math.radians(angle)) for x,y in rectangle_vertices]
    # for coordinate in rectangle_vertices:
    #     print(coordinate)
    # print(len(rectangle_vertices))
    img1.polygon(rectangle_vertices, fill=0)
    return img

# just create a new image(black white), white color, for size x,y
def CreateNewImage(x,y):
    image=Image.new('L', (x,y),255)
    return image


def CreateParkingSpace(width,height,number_of_cars):
    """create a parking space dictionary, key is parking space coordinate, value is 0,1 0: free, 1: occupant
    parking space size is 2.5*5.3, road width is 3.5m

    Args:
        width (_type_): the total image width
        height (_type_): the total image height
        number_of_cars (_type_): total number of cars
    Returns:
        parking_space(dictionary): key is parking space coordinate, value is 0,1 0: free, 1: occupant
    """    
    parking_space={}
    if number_of_cars<=0:
        return parking_space
    # a road width plus 1/2* parking space width
    low_limit_x=3.5+1.25
    # total map width minus a road width minus 1/2* parking space width
    high_limit_x=width-3.5-1.25
    # a road width plus 1/2* parking space height
    low_limit_y=3.5+5.3*0.5
    # map height minus a road width minus 2* parking space height minus a road width 
    high_limit_y=low_limit_y+5.3
    parking_space_center=[low_limit_x,low_limit_y]
    count=0
    
    # first row parking space 
    while parking_space_center[1]>=low_limit_y and parking_space_center[1]<=high_limit_y:
        while parking_space_center[0]>=low_limit_x and parking_space_center[0]<=high_limit_x:
            if count<number_of_cars/3:
                parking_space[tuple( parking_space_center)]=random.randint(0,1)
            else:
                parking_space[tuple( parking_space_center)]=0
            if parking_space[tuple( parking_space_center)]==1:
                count=count+1
            parking_space_center[0]=parking_space_center[0]+2.5
            # print(parking_space_center)
        parking_space_center[1]=parking_space_center[1]+5.3
        parking_space_center[0]=low_limit_x

    # second row parking space, recalculate limit y
    low_limit_y=3.5*2+5.3*2+0.5*5.3
    high_limit_y=low_limit_y+5.3
    parking_space_center=[low_limit_x,low_limit_y]
    while parking_space_center[1]>=low_limit_y and parking_space_center[1]<=high_limit_y:
        while parking_space_center[0]>=low_limit_x and parking_space_center[0]<=high_limit_x:
            if count<2/3*number_of_cars:
                parking_space[tuple( parking_space_center)]=random.randint(0,1)
            else:
                parking_space[tuple( parking_space_center)]=0
            if parking_space[tuple( parking_space_center)]==1:
                count=count+1
            parking_space_center[0]=parking_space_center[0]+2.5
            # print(parking_space_center)
        parking_space_center[1]=parking_space_center[1]+5.3
        parking_space_center[0]=low_limit_x
    # print(count)

  # third row parking space, recalculate limit y
    low_limit_y=3.5*3+5.3*4+0.5*5.3
    high_limit_y=low_limit_y+5.3
    parking_space_center=[low_limit_x,low_limit_y]
    while parking_space_center[1]>=low_limit_y and parking_space_center[1]<=high_limit_y:
        while parking_space_center[0]>=low_limit_x and parking_space_center[0]<=high_limit_x:
            if count<number_of_cars:
                parking_space[tuple( parking_space_center)]=random.randint(0,1)
            else:
                parking_space[tuple( parking_space_center)]=0
            if parking_space[tuple( parking_space_center)]==1:
                count=count+1
            parking_space_center[0]=parking_space_center[0]+2.5
            # print(parking_space_center)
        parking_space_center[1]=parking_space_center[1]+5.3
        parking_space_center[0]=low_limit_x
    # print(count)

    return parking_space




# create a map with several obstacle(vehicles)
def CreateMap(x,y,number_of_obstacle,show_map=False,save=False):
    img=CreateNewImage(x,y)
    count=0
    parking_space=CreateParkingSpace(x,y,number_of_obstacle)
    for pair in parking_space.keys():
        if parking_space[pair]==1:
            img=DrawObstacle(img, pair[0], pair[1],round(random.uniform(1.6, 1.8), 2),round(random.uniform(3.8,4.3), 2),random.randint(0,0))
    # while count<number_of_obstacle:
    #     # vehicle width is usually in 1600mm to 1800mm, length in 3800mm to 4300mm
    #     img=DrawObstacle(img, random.randint(8, x-8),random.randint(8, y-8),  round(random.uniform(1.6, 1.8), 2),round(random.uniform(3.8,4.3), 2),random.randint(0,0))
    #     count=count+1
    if show_map:
        img.show()
    if save:
        dir='/home/jialiang/Code/thesis_ws/src/hybrid_astar/maps/'
        map_name=str(x)+str(y)+str(number_of_obstacle)+'.png'
        img.save(dir+map_name,'PNG')

def main():
    map_height=80
    map_width=50
    max_number_of_obstacle=60
    count=0
    while count<max_number_of_obstacle:
        CreateMap(map_height, map_width, count,False,True)
        count=count+1

if __name__=='__main__':
    main()