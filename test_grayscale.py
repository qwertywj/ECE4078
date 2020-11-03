# test access to wheels and camera

# for physical robot use port 8080



import time

import numpy as np

import requests

import cv2 



def set_velocity(vel0, vel1):

    r = requests.get("http://localhost:40000/robot/set/velocity?value="+str(vel0)+","+str(vel1))



def get_image():

    r = requests.get("http://localhost:40000/camera/get")

    img = cv2.imdecode(np.fromstring(r.content,np.uint8), cv2.IMREAD_COLOR)



    return img

def CAM_distance_theta(img,c):
    counter = 0;
    discnt = 0;
    end_c = 0;
    total = 0;
    Flag = True;
    distance=[];
    theta=[];
    #A=[]
    while Flag:
        for i in range(img.shape[1]-end_c):
            
            if 255 in img[:,i+end_c]:
                
                counter+=1;
                #print(counter)
                if counter == 3:
                    total +=1;
                    discnt = 0;
                    start_c=i+end_c-2;
                    counter = 0;
                    for j in range(img.shape[1]-end_c-i):
                        if 255 not in img[:,i+end_c+j]:
                            if j > img.shape[1]-end_c-i-4:
                                counter=0;
                                end_c+=i+j;
                                break;
                            else:
                                counter+=1;
                                if counter == 3:
                                    end_c+=i+j-3;
                                    counter=0;
                                    break;
                        elif j == img.shape[1]-end_c-i-1:
                            end_c = 639;
                            break;
                    #A.append([start_c,end_c]);


                    if start_c == 0 or end_c == 639:
                        break
                    # height calculation forloop
                    max_h=img.shape[1]+1;
                    min_h=-1;
                    for h in range(img.shape[0]):
                        if 255 in img[h,start_c:end_c]:
                            if h>min_h:
                                min_h=h;
                    
                            if h<max_h:
                                max_h=h;
                                
                    if max_h != -1 and min_h != img.shape[1]+1:
                        img_height= min_h-max_h;
                        theta_col = np.floor((end_c-start_c)/2)+start_c;
                        theta_rad = theta_col/640 * 0.892 - 0.446;
                        if c == 0:
                            D = 64.2857/img_height+0.35;
                        elif c ==1:
                            D = 0.068+(106.27/img_height);
                        distance.append(D);
                        theta.append(theta_rad);
                    break

            # detect discotinue in a range of 3
            elif 255 not in img[:,i] and counter >0:
                discnt +=1;
                counter +=1;
                if discnt ==2:
                    counter = 0;
                    discnt = 0;

            if i == img.shape[1]-end_c -1:
                Flag = False; 
    return distance,theta




if __name__ == "__main__":

    print("capture image")
    # lower_sheep = np.array([22,4,20])

    # upper_sheep = np.array([121,51,201])



    lower_sheep = np.array([17,6,5])

    upper_sheep = np.array([150,254,255])



    lower_coke = np.array([155,50,50])

    upper_coke = np.array([175,254,254])


    image = get_image()

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    sheep = cv2.inRange(hsv, lower_sheep, upper_sheep)

    coke = cv2.inRange(hsv, lower_coke, upper_coke)
    combine_white = coke + sheep

    coke_dis,coke_theta = CAM_distance_theta(coke,0)
    sheep_dis,sheep_theta = CAM_distance_theta(sheep,1)

    sheep_world=[]
    coke_world=[]
    if(len(sheep_theta) is not 0):
        for i in range(len(sheep_theta)):
            robot_x =1 #self.slam.robot.state[0]
            robot_y =1 #self.slam.robot.state[1]
            robot_theta =0 #self.slam.robot.state[2]
            x_world = (sheep_dis[i] / np.cos(sheep_theta[i])) * np.cos(robot_theta - sheep_theta[i])
            y_world = (sheep_dis[i] / np.cos(sheep_theta[i])) * np.sin(robot_theta - sheep_theta[i])
            sheep_world.append([float(robot_x + x_world) , float(robot_y + y_world)])

    if(len(coke_theta) is not 0):
        for i in range(len(coke_theta)):
            robot_x = 1#self.slam.robot.state[0]
            robot_y = 1#self.slam.robot.state[1]
            robot_theta = 0#self.slam.robot.state[2]
            x_world = (coke_dis[i] / np.cos(coke_theta[i])) * np.cos(robot_theta - coke_theta[i])
            y_world = (coke_dis[i] / np.cos(coke_theta[i])) * np.sin(robot_theta - coke_theta[i])
            coke_world.append([float(robot_x + x_world) , float(robot_y + y_world)])


    print("coke distance:")

    print(coke_dis)

    print("coke theta:")

    print(coke_theta)

    print("sheep distance:")

    print(sheep_dis)

    print("sheep theta:")

    print(sheep_theta)

    print("world frame")
    print(coke_world)

    print(sheep_world)


    while True:

        cv2.imshow("yuanben",image)

        cv2.imshow('COKE white',coke)

        cv2.imshow('Sheep white',sheep)  

        cv2.imshow('Combine white',combine_white)



        cv2.waitKey(1)

        continue
