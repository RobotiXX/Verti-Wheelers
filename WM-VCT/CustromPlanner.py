#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point, Quaternion
from nav_msgs.msg import Path , Odometry
from grid_map_msgs.msg import GridMap

import numpy as np
import math
import cv2
import torch
import torch.nn as nn
import torch.optim as optim
import queue
import threading

from NN_estimator import NN_state_estimator
from cust_model import CustomModel

from Grid import MapProcessor
from utilities import utils, NN_batch, odom_processor, HashablePose
from copy import deepcopy



class LocalPlanner:
    def __init__(self):
        
        #Variables
        self.net1 = NN_state_estimator()
        self.mp = MapProcessor()
        self.uts = utils()
        self.net1_batch = NN_batch()
        self.odp = odom_processor()
        self.counter = 0
        self.startpose = Pose()
        self.goalpose = Pose()
        self.robotpose = Pose()
        self.val_goal = False
        self.map_list = []
        self.map_img_list = []
        self.plan_trajectory = []  
        self.trajectory_thread = threading.Thread()
        self.replan_count = 0      

        #---------------Parameters------------------------------------------------------------------------------
        #---------------0----1---2----3---4--5--6--7---------------------------------------------------------------
        self.cost_w = [0.4, 0.4, 4, 0.2, 8, 4, 1, 5]      # 0:roll, 1:pitch, 2:map_bound, 3:elev_change, 4:stuck_cond, 5:dist_to_goal 
        self.mp.fill = 2                                # 0:min_val   1:max_val   2:zero   3:(max_val-min_val)/fill_val_div
        self.mp.fill_val_div = 4                        #only applicable if fill = 3
        self.mp.robot_footprint = (0.321, 0.801)        #Robot footprint in meters width, length
        self.odp.cal_delay = 10                         #count odom for calibration msg 
        self.odp.initialized = False                    #whether to calibrate the odom msg
        self.min_str_ang = -45                          #Max steering angle
        self.max_str_ang = 45                           #Min steering angle
        self.max_itr = 8                               #Maximum loop iteration till goal
        self.sample_count = 11                          #No. of samples of steering angle from min to max
        self.step_depth = 5                             #step depth to calculate        
        self.step_size = 0.20                             #step size in meter
        self.robot_wheelbase = 0.67                     #Robot wheelbase in meter
        self.goal_tollerance = 0.1                     #goal position tollerance in meter
        self.replan_freqency = 5                        #Replan Frequency in Hz  
        self.path_points = 4                            #Max no of points in path
        self.net1_batch.fill_array(self.sample_count)
        rospy.init_node('local_planner')
        
        #ROS topic suscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        rospy.Subscriber('/elevation_mapping/elevation_map_raw', GridMap, self.gridMap_callback, queue_size=1)
        rospy.Subscriber('dlio/odom_node/odom', Odometry, self.odom_callback, queue_size=1)

        #ROS topic publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)

        #ROS Rate
        rate = rospy.Rate(30)
        print("Planner initialized\n Waiting for gridmap and odometry")
        #print(cv2.useOptimized())
        #cv2.setNumThreads(20)
        #print(cv2.getNumThreads())
        
        '''if(self.mp.map_init):
                elev_ft, elev_ft_img  = self.mp.get_elev_footprint(self.robotpose, (160,400))
        '''


        #loop
        while not rospy.is_shutdown():
            if (self.val_goal == True) :
                self.execute_trajectory()
                if self.replan_count == (20/self.replan_freqency):
                    self.replan_count = 0
                    if self.trajectory_thread.is_alive() == False:
                        self.trajectory_thread = threading.Thread(target=self.generate_trajectory, args=(self.robotpose, self.goalpose))
                        #self.trajectory_thread = threading.Thread(target=self.ucs, args=(self.robotpose, self.goalpose))
                        self.trajectory_thread.start()
                        pass
                else:
                    self.replan_count += 1
                
            rate.sleep()
      
    #call-back function for odom msg
    def odom_callback(self, odom_msg):
        self.odp.update(odom_msg)
        self.robotpose = deepcopy(self.odp.robot_pose)
        self.mp.robot_pose = deepcopy(self.odp.robot_pose)
    
    #call-back function for goal msg
    def goal_callback(self, goal_msg):
        if self.mp.map_init == True:
            print("Goal Received")
            self.val_goal = False
            start_time =rospy.Time.now()
            delay = True
            while(delay):
                elapsed_time = rospy.Time.now() - start_time
                execution_time = elapsed_time.to_sec()
                if execution_time > 2:
                    delay = False

            self.goalpose = deepcopy(goal_msg.pose)
            self.startpose = deepcopy(self.robotpose)
            #self.generate_trajectory(self.startpose, self.goalpose)
            
            self.trajectory_thread = threading.Thread(target=self.generate_trajectory, args=(self.robotpose, self.goalpose))
            self.trajectory_thread.start()
            #start_time = rospy.Time.now()
            #traj = self.ucs(self.startpose, self.goalpose)
            #elapsed_time = rospy.Time.now() - start_time  
            #execution_time = elapsed_time.to_sec()
            #print("Execution time total: %.4f seconds", execution_time)
            #publish the path
            self.val_goal = True

    #call-back function for gridmap_msg
    def gridMap_callback(self, gridmap_msg):
            self.mp.update_map(gridmap_msg)
            if self.mp.map_init == False:
                print("Received gridmap, planner ready to receive goal")
                
            self.mp.map_init = True
            '''
            if self.trajectory_thread.is_alive() == False and self.val_goal == True:
                # Create a new thread for executing generate_trajectory
                self.trajectory_thread = threading.Thread(target=self.generate_trajectory, args=(self.robotpose, self.goalpose))
                self.trajectory_thread.start()'''

    #-----------------Sampling based single depth tree---------------------
    #Generate trajectory using sampling-based planner
    def generate_trajectory(self, start_pose, goal_pose):
        #timer for debugging
        start_time = rospy.Time.now()
        
        self.counter = 0

        min_a = np.radians(self.min_str_ang)
        max_a = np.radians(self.max_str_ang)
        a_div = self.sample_count
        steering_angles = np.linspace(min_a, max_a, a_div)

        final_trajectory = []
        traj_publish = []

        #distance = self.uts.get_dist(start_pose, goal_pose)

        cur_pose = deepcopy(start_pose)

        
        #self.net1_batch.batch_init(self.mp.get_elev_footprint(cur_pose, (40,100))[0], deepcopy(cur_pose), Twist(), a_div)
        
        #main compute loop
        for cnt in range(self.max_itr):
            trajectories = []
            start_time1 = rospy.Time.now()
            
            self.net1_batch.batch_init(self.mp.get_elev_footprint(cur_pose, (40,100))[0], deepcopy(cur_pose), Twist(), a_div)

            #compute the trajectory in given steering angle sweep
            for angle in steering_angles:
                trajectories.append(self.compute_trajectory(cur_pose, goal_pose, angle, cnt))
            

            elapsed_time = rospy.Time.now()  
            execution_time = (rospy.Time.now() - start_time1).to_sec()
            #print("\n\nExecution time traj computation: %.4f seconds", execution_time)

            self.get_image_array(trajectories)

             
            execution_time = (rospy.Time.now() - elapsed_time).to_sec()
            elapsed_time = rospy.Time.now() 
            #print("Execution time for rotate: %.4f seconds", execution_time)

            #Get the Roll and Pitch prediction for from Neural Net
            for sn in range(len(trajectories[0])):
                if cnt == 0:
                    pass#break
                if self.mp.is_on_map(trajectories[0][sn]) == False:
                     break

                self.assemble_img_batch_torch(trajectories, sn)

                #start_time = rospy.Time.now()

                image, p_usless, twist_usless = self.net1_batch.get_batch_dsc()
                pose = self.net1_batch.get_rp_batch_dsc()
                pred = self.net1.estimate(np.array(image), np.array(pose)).tolist()
                for i in range(len(pred)):
                    pred[i][0] = 1.1 * pred[i][0]
                    pred[i][1] = 1.1 * pred[i][1]

                orientation = []
                for p in range(len(pred)):
                    orientation.append(self.net1_batch.get_rp_to_orientation(pred[p], self.uts.quaternion_to_yaw(trajectories[p][sn].orientation)))
                    trajectories[p][sn].orientation = orientation[p]  
                self.update_pose_batch(trajectories, orientation, sn)
                
                execution_time = (rospy.Time.now() - elapsed_time).to_sec()
                elapsed_time = rospy.Time.now() 
                #print("Execution time for NN-prediction: %.4f seconds", execution_time)
                
            #calculate the cost of each trajectory
            costs = []

            for tr in range(len(trajectories)):
                costs.append(self.calculate_cost_sample(trajectories, tr))

            execution_time = (rospy.Time.now() - elapsed_time).to_sec()
            elapsed_time = rospy.Time.now() 
            #print("Execution time cost calculation: %.4f seconds", execution_time)

            #Get the best trajectory
            best_traj = deepcopy(trajectories[np.argmin(costs)])
            
            #Add the trajectory points in the final trajectory
            for pos in best_traj:
                final_trajectory.append(pos)

            #publish the path
            path_msg = Path()
            path_msg.header.frame_id = 'odom'  # Replace 'world' with your desired frame ID
            path_msg.poses = [self.create_pose_stamped(pose) for pose in final_trajectory]
            #self.path_pub.publish(path_msg)

            #Set the current pose as the last pose in the selected trajectory
            cur_pose = deepcopy(best_traj[len(best_traj)-3])
            for p in range(len(best_traj)-2):
                traj_publish.append(best_traj[p])

            if self.goal_tollerance >= self.uts.get_dist(cur_pose, goal_pose):
                break

            execution_time = (rospy.Time.now() - start_time1).to_sec() 
            #print("Execution time sample loop: %.4f seconds", execution_time)
        
        elapsed_time = rospy.Time.now() - start_time  
        execution_time = elapsed_time.to_sec()
        print("Execution time total: %.4f seconds", execution_time)
        #print(self.counter)

        path_msg = Path()
        path_msg.header.frame_id = 'odom'  # Replace 'world' with your desired frame ID
        path_msg.poses = [self.create_pose_stamped(pose) for pose in traj_publish]
        self.path_pub.publish(path_msg)

        self.val_goal = True
        
        self.plan_trajectory = traj_publish  

    #Compute trajectory given the steering angle
    def compute_trajectory(self, start_pose, goal_pose, st_angle, cnt):
        step = deepcopy(self.step_size)
        s_num = self.step_depth

        
        #initialize the trajectory with the current pose as first pose
        trajectory =[]
        if cnt == 0:
            posestart = Pose()
            posestart = deepcopy(start_pose)
            trajectory = [posestart]
        

        cur_pose = deepcopy(start_pose)

        for s in range(s_num):
            if s_num - s == 1:
                step = step
            #break if the distance to goal is within tollerace
            if self.uts.get_dist(cur_pose, goal_pose) <= self.goal_tollerance:
                for i in range(s,s_num):
                    trajectory.append(goal_pose)
                return trajectory

            if s % 2 == 0:
                st_angle = - st_angle
            #calculate the next orientation based on the steering angle
            del_yaw = (step * math.tan(st_angle)) / self.robot_wheelbase
            cmd_yaw = self.uts.quaternion_to_yaw(cur_pose.orientation) + del_yaw
            
            #calculate the next position and add that in the trajectory
            next_pose = Pose()
            next_pose = deepcopy(cur_pose)
            next_pose.position.x = cur_pose.position.x + step * math.cos(cmd_yaw)
            next_pose.position.y = cur_pose.position.y + step * math.sin(cmd_yaw)
            next_pose.position.z = self.mp.get_pose_height(next_pose) + (cur_pose.position.z - self.mp.get_pose_height(cur_pose))
            #next_pose.position.z = cur_pose.position.z
            next_pose.orientation = self.uts.yaw_to_quaternion(cmd_yaw)
            trajectory.append(next_pose)

            #update the current pose to calculated one for the next step
            cur_pose = deepcopy(next_pose)

        #return the trajectory
        return trajectory
    
    #Cost function for the trajectory evaluation 
    def calculate_cost_sample(self, trajectories, tr):
        
        roll, pitch, map_bound, elev_change, stuck_cond, dist_to_goal = 0, 0, 0, 0, 0, 0
        
        dist_to_goal  = self.uts.get_dist(trajectories[tr][-1], self.goalpose)
        if dist_to_goal < self.goal_tollerance:
            return -1

        for pos in range(len(trajectories[tr])):
            c_pose = trajectories[tr][pos]
            r, p, y  = np.degrees(self.uts.quaternion_to_rpy(c_pose.orientation))

            if self.mp.is_on_map(c_pose):
                elev_ft = self.map_list[tr][pos]
                diff = np.abs(elev_ft)
                diff[diff <= 0.25] = 0
                map_bound = int(np.sum(diff))

                front, mid, back = self.get_sections(elev_ft)

                stuck_cond = abs( ((front.mean() + back.mean())/2) - mid.mean() ) *40

                if pos > 0:
                    elev_ft_prev = self.map_list[tr][pos-1]
                    elev_prev = np.abs(elev_ft_prev)
                    average_elev_prev = sum(elev_prev[elev_prev >= 0.01])
                    elev_cur = np.abs(elev_ft)
                    average_elev_cur = sum(elev_cur[elev_cur >= 0.01])
                    
                    elev_change = abs(average_elev_cur - average_elev_prev)
 

                if abs(r) > 10:
                    roll += abs(r)
                if abs(p) > 10:
                    pitch += abs(p)

            else:
                map_bound += 10
                roll += 4
                pitch += 4

            

        return  self.cost_w[0]*roll + self.cost_w[1]*pitch + self.cost_w[2] * map_bound + self.cost_w[3] * elev_change + self.cost_w[4] *stuck_cond + self.cost_w[5]* dist_to_goal      #h*4 + 2*h1 + 1*h2  + 7*dist_to_goal

    def get_sections(self, elev_ft):
        x = elev_ft.shape[0]
        y = elev_ft.shape[1]

        fr = int(y // 4)
        mi = int(y // 2) + fr

        front = elev_ft[0:x, 0:fr]
        mid = elev_ft[0:x, fr:mi]
        back = elev_ft[0:x, mi:y]
        return front, mid, back      

    #assemble image batch for neural net
    def assemble_img_batch(self, trajectories, sn):
        for idx in range(len(trajectories)):
            if len(trajectories[idx]) > sn:
                if self.mp.is_on_map(trajectories[idx][sn]) == False:
                     break
                elev_ft, elev_ft_img  = self.mp.get_elev_footprint(trajectories[idx][sn], (40,100))
                self.net1_batch.add_image(elev_ft, idx)
                self.counter += 1
                #self.net1_batch.add_image(np.zeros((80,200)), idx)

    #assemble image batch for neural net when footprint is retrived by torch
    def assemble_img_batch_torch(self, trajectories, sn):
        #start_time = rospy.Time.now()
        #map_array , map_img_array = self.mp.get_elev_footprint_torch(trajectories, sn, (40,100))
        map_array = self.map_list
        for idx in range(len(trajectories)):
            self.net1_batch.add_image(map_array[idx][sn], idx)
        self.counter += 1

    #Update footprint array for the set of trajectorys
    def get_image_array(self, trajectories):
        
        pose_list = []
        tr = len(trajectories)
        st = len(trajectories[0])
        for path in trajectories:
            for pose in path:
                p = [pose]
                pose_list.append(p)

        map_array = self.mp.get_elev_footprint_torch(pose_list, 0, (40,100))[0]

        self.map_list = []
        self.map_img_list = []

        for i in range(0, len(map_array), st):
            map_t = []
            im_t = []
            for j in range(st):
                map_t.append(map_array[i+j])
                #im_t.append(map_img_array[i+j])
            self.map_list.append(map_t)
            #self.map_img_list.append(im_t)

        '''
        for i in self.map_img_list[6]:
            cv2.imshow("Elevation Image", i)
            cv2.waitKey(200)
        '''

    def get_image_array_ucs(self, trajectories):
        
        pose_list = []
        tr = len(trajectories)
        for pose in trajectories:
                p = [pose]
                pose_list.append(p)

        map_array = self.mp.get_elev_footprint_torch(pose_list, 0, (40,100))[0]

        self.map_list = map_array
        #self.map_img_list = []

        '''
        for i in self.map_img_list[6]:
            cv2.imshow("Elevation Image", i)
            cv2.waitKey(200)
        '''

    #Update pose based on NN prediction
    def update_pose_batch(self, trajectories, orientation, sn):
        for t in range(len(trajectories)):
            #print(np.degrees(self.uts.quaternion_to_rpy(trajectories[t][sn].orientation)))
            trajectories[t][sn].orientation = deepcopy(orientation[t])
            #print(np.degrees(self.uts.quaternion_to_rpy(trajectories[t][sn].orientation)))
            self.net1_batch.add_pose(trajectories[t][sn], t)
    
    #---------------------UCS algorithm-------------------------------------
    #assemble image batch for he ucs algorithm
    def assemble_img_batch_ucs(self, poses):
        for idx in range(len(poses)):
            map_array = self.map_list
            self.net1_batch.add_image(map_array[idx], idx)
    
    #Update pose based on NN prediction
    def update_pose_batch_ucs(self, Poses, orientation):
        for idx in range(len(Poses)):
            #print(np.degrees(self.uts.quaternion_to_rpy(Poses[idx].orientation)))
            Poses[idx].orientation = deepcopy(orientation[idx])
            #print(np.degrees(self.uts.quaternion_to_rpy(Poses[idx].orientation)))
            self.net1_batch.add_pose(Poses[idx], idx)

    #Update UCS for path generation
    def ucs(self, start_pose, goal_pose):
        start_time = rospy.Time.now()
        count = 0    
        min_a = np.radians(self.min_str_ang)
        max_a = np.radians(self.max_str_ang)
        a_div = self.sample_count
        steering_angles = np.linspace(min_a, max_a, a_div)
        cur_pose = deepcopy(start_pose)
        step = self.step_size
        itr = 0
        cur_p = HashablePose(cur_pose, 0)

        def calculate_cost(Parent_pose, new_pose, idx):
            dist = (self.uts.get_dist(new_pose, self.goalpose))
            roll = 0
            pitch = 0
            h = 0
            h1 = 0
            h2 = 0

            if dist < self.goal_tollerance:
                return cur_p.cost - 1
            
            if self.mp.is_on_map(new_pose):
                if dist > self.step_size * self.step_depth * self.max_itr*0.5:
                    dist = self.step_size * self.step_depth * self.max_itr * 3

            if self.mp.is_on_map(new_pose):
                elev_ft = self.map_list[idx]
                cur_h = self.mp.get_pose_height(Parent_pose)
                h = abs(elev_ft.mean() - Parent_pose.h1_cost)
                #h1 = max(elev_ft.max(), abs(elev_ft.min()))
                #tst = abs(h1 - cur_h)
                h1 = 0 
              
                diff = np.abs(elev_ft - cur_h)
                # Set elements that satisfy the condition to 0
                diff[diff <= 0.25] = 0
                # Sum all non-zero elements
                h1 = int(np.sum(diff))


                #Sectional Analysis for higher mid section
                front, mid, back = self.get_sections(elev_ft)
                h2 = abs(front.mean() + back.mean()) - abs(mid.mean())

            
            
            r, p, y  = np.degrees(self.uts.quaternion_to_rpy(new_pose.orientation))

            r_o, p_o, y_o = np.degrees(self.uts.quaternion_to_rpy(Parent_pose.orientation))

            r = abs(r_o - r)

            p = abs(p_o - p) 


            if r > 3 or r < -3:
                roll = abs(Parent_pose.roll_cost - abs(r))
            
            else:
                roll += 0

            if p > 3 or p < -3:
                pitch = abs(Parent_pose.pitch_cost - abs(p))
            else:
                pitch += 0


            return cur_p.cost + 0.8 * roll +  0.8 * pitch + 4 * h + h1**10 + 12*h2 + self.uts.get_dist(Parent_pose, new_pose)*0.8 + dist *2, [roll, pitch, h, h1, h2, dist]



        visited = set()  # To track visited points
        priority_queue = queue.PriorityQueue()
        priority_queue.put((cur_p, []))  # (cost, point, path)

        while not priority_queue.empty():
            
            cur_pose, path = priority_queue.get()
            self.net1_batch.batch_init(self.mp.get_elev_footprint(cur_pose, (40,100))[0], deepcopy(cur_pose), Twist(), a_div)
            #step = step + 0.003    
            traj = self.get_path(path)
            path_msg = Path()
            path_msg.header.frame_id = 'odom'  # Replace 'world' with your desired frame ID
            path_msg.poses = [self.create_pose_stamped(pose) for pose in traj]
            #self.path_pub.publish(path_msg)

            if  (len(path) > self.path_points) or (itr > self.max_itr) :
                final_path = self.get_path(path)
                print(count)
                path_msg = Path()
                path_msg.header.frame_id = 'odom'  # Replace 'world' with your desired frame ID
                path_msg.poses = [self.create_pose_stamped(pose) for pose in final_path]
                self.path_pub.publish(path_msg)
                elapsed_time = rospy.Time.now() - start_time  
                execution_time = elapsed_time.to_sec()
                print("Execution time total: %.4f seconds", execution_time)
                self.plan_trajectory = final_path
                return final_path
            
            #break if the distance to goal is within tollerace
            if self.uts.get_dist(cur_p.get_pose(), goal_pose) <= self.goal_tollerance:
                final_path = self.get_path(path)
                print(count)
                path_msg = Path()
                path_msg.header.frame_id = 'odom'  # Replace 'world' with your desired frame ID
                path_msg.poses = [self.create_pose_stamped(pose) for pose in final_path]
                self.path_pub.publish(path_msg)
                elapsed_time = rospy.Time.now() - start_time  
                execution_time = elapsed_time.to_sec()
                print("Execution time total: %.4f seconds", execution_time)
                self.plan_trajectory = final_path
                return final_path

            if cur_pose not in visited:
                visited.add(cur_pose)
                
                Pose_ar = []
                for st_angle in steering_angles:
                    count += 1
                    del_yaw = (step * math.tan(st_angle)) / self.robot_wheelbase
                    cmd_yaw = self.uts.quaternion_to_yaw(cur_pose.orientation) + del_yaw

                    #calculate the next position and add that in the trajectory
                    next_pose = Pose()
                    next_pose = deepcopy(cur_pose.Pose)
                    next_pose.position.x = cur_pose.position.x + step * math.cos(cmd_yaw)
                    next_pose.position.y = cur_pose.position.y + step * math.sin(cmd_yaw)
                    next_pose.position.z = self.mp.get_pose_height(next_pose) + (cur_pose.position.z - self.mp.get_pose_height(cur_pose))
                    #next_pose.position.z = cur_pose.position.z
                    next_pose.orientation = self.uts.yaw_to_quaternion(cmd_yaw)
                    nxt_p = HashablePose(next_pose, 0)
                    
                    if nxt_p not in visited:
                        Pose_ar.append(next_pose)

                self.get_image_array_ucs(Pose_ar)
                self.assemble_img_batch_ucs(Pose_ar)

                image, p_usless, twist_usless = self.net1_batch.get_batch_dsc()
                pose = self.net1_batch.get_rp_batch_dsc()
                pred = self.net1.estimate(np.array(image), np.array(pose)).tolist()
                orientation = []
                for p in range(len(pred)):
                    orientation.append(self.net1_batch.get_rp_to_orientation(pred[p], self.uts.quaternion_to_yaw(Pose_ar[p].orientation)))
                self.update_pose_batch_ucs(Pose_ar, orientation)

                priority_queue_temp = queue.PriorityQueue()
                for n_pos in range(len(Pose_ar)):
                    nxt_p = HashablePose(Pose_ar[n_pos], 0)
                    nxt_p.cost, cost_list = calculate_cost(cur_pose, nxt_p, n_pos)
                    nxt_p.cost = round(nxt_p.cost, 1)
                    nxt_p.roll_cost = cost_list[0]
                    nxt_p.pitch_cost = cost_list[1]
                    nxt_p.h_cost = cost_list[2]
                    nxt_p.h1_cost = cost_list[3]
                    nxt_p.h2_cost = cost_list[4]
                    nxt_p.dist_to_goal_cost = cost_list[5]
                    new_path = path + [cur_pose]
                    priority_queue_temp.put((nxt_p, new_path))

                for g in range(int(len(steering_angles)//2)):
                    priority_queue.put((priority_queue_temp.get()))

            itr += 1
        return []  # No trajectory found

    #get path from hashable pose
    def get_path(self, traj):
        tr = []
        for t in traj:
            tr.append(t.Pose)

        return tr

    #helper function to create stamped pose for path message
    def create_pose_stamped(self, pose):
        # Create a PoseStamped message from a Pose message
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = 'odom'  # Replace 'world' with your desired frame ID

        return pose_stamped

    def execute_trajectory(self):
        
        if self.uts.get_dist(self.robotpose, self.goalpose) < self.goal_tollerance:
            #self.val_goal = False
            cmd = Twist()
            cmd.linear.x = 0
            self.cmd_pub.publish(cmd)
            self.plan_trajectory = []
            path_msg = Path()
            path_msg.header.frame_id = 'odom'  # Replace 'world' with your desired frame ID
            path_msg.poses = []
            self.path_pub.publish(path_msg)
            self.replan_count = 0

            return True

        # Iterate over the intermediate poses and send control commands
        if(len(self.plan_trajectory) > 1):
            pose = self.plan_trajectory[1]
            dist = self.uts.get_dist(self.robotpose, pose)

            if (dist < 0.075):
                    self.plan_trajectory.remove(self.plan_trajectory[0])
                    path_msg = Path()
                    path_msg.header.frame_id = 'odom'  # Replace 'world' with your desired frame ID
                    path_msg.poses = [self.create_pose_stamped(pose) for pose in self.plan_trajectory]
                    self.path_pub.publish(path_msg)

            if (dist > self.step_size*3):                   
                    if self.trajectory_thread.is_alive() == False:
                        self.val_goal = False
                        self.trajectory = []
                        self.trajectory_thread = threading.Thread(target=self.generate_trajectory, args=(self.robotpose, self.goalpose))
                        #self.trajectory_thread = threading.Thread(target=self.ucs, args=(self.robotpose, self.goalpose))
                        self.trajectory_thread.start()
            
            cmd = self.compute_control_commands(pose)
            self.cmd_pub.publish(cmd)

        else:
            
            if self.trajectory_thread.is_alive() == False:
                self.val_goal = False
                self.trajectory_thread = threading.Thread(target=self.generate_trajectory, args=(self.robotpose, self.goalpose))
                #self.trajectory_thread = threading.Thread(target=self.ucs, args=(self.robotpose, self.goalpose))
                self.trajectory_thread.start()
        
        return False
        
    def compute_control_commands(self, next_pose):
        cmd = Twist()
        cur_head = np.degrees(self.uts.quaternion_to_yaw(self.robotpose.orientation))
        del_x = next_pose.position.x - self.robotpose.position.x
        del_y = next_pose.position.y - self.robotpose.position.y
        target_angle = np.degrees(math.atan2(del_y, del_x))
        st_ang = target_angle - cur_head
        #print("before correction " + str(st_ang))
        if st_ang < -180:
            st_ang = (360 + st_ang)
        
        if st_ang > 180: 
            st_ang = -(360 - st_ang)
        
        #print("after correction" + str(st_ang))
        st_ang = max(-45, min(45, st_ang))

        st_ang = self.uts.map_value(st_ang, -50, 50, -1, 1)
        
        r_pitch = np.degrees(self.uts.quaternion_to_pitch(self.robotpose.orientation))       

        if r_pitch > -3 and r_pitch < 5:
            cmd.linear.x = 0.08
        
        else:
            if r_pitch < 8:
                cmd.linear.x = 0.08
            else:
                cmd.linear.x = 0.08
        cmd.angular.z = -st_ang

        return cmd

if __name__ == '__main__':
    try:
        lp = LocalPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
