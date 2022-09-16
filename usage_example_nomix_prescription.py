from centermask.grasp import grasp
from demo.demo_pill import pill_segmentation, pill_segmentation_mask_first
from figure.utils import create_axs
from detectron2.data.detection_utils import read_image
from centermask.realsense_capture import d415_frames
from auboi5_controller import AuboController
from motor_control import MotorController
import time
# import math
from copy import deepcopy
# import cv2
# import serial

import matplotlib.pyplot as plt

def place_manager(pick_todo_num, presc_list, day):
    target_timing = len(presc_list) - pick_todo_num
    place_row = day  # one day of a week (0:Mon, ..., 6:Sun)
    place_col = presc_list[target_timing]  # one timing of a day

    return place_row, place_col



if __name__ == "__main__":
    # robot
    auboi5_controller = AuboController('192.168.1.115')
    # waypoint = auboi5_controller.getCurrentWaypoint()
    # print(waypoint)
    # exit()

    # motor
    mc = MotorController('/dev/ttyUSB1')
    # camera
    cam = d415_frames.camera()
    for i in range(20):
        cam.capture()
    vision_x_error = -0.016
    vis_grasp = True  # visualize grasp detection results
    vis_checkexist = False
    push_experiment = False
    
    # Prescription for one day
    prescription_timing = {
                            'pill_A':[1,2], 'pill_B':[2], 'pill_C':[1,3], 'pill_D':[1], 
                            'pill_E':[3], 'pill_F':[1,4], 'pill_G':[1,4], 'pill_H':[1]
                          }  # TIMING: 0 for 8:00, 1 for 12:00, 2 for 17:00, 3 for 21:00         

    # Process the prescription
    for pill_name, presc_list in prescription_timing.items():
        
        pick_todo_num = len(presc_list)
        if pick_todo_num<= 0:
            continue  # skip

        if push_experiment == False:
            # Grasp SOME without looking for the first time
            mc.go_position(mc.find_box(pill_name[-1]))
            time.sleep(0.1)
            auboi5_controller.moveJ(auboi5_controller.find_capture_position(pill_name[-1])) 
            auboi5_controller.pick_one_time(auboi5_controller.find_grasping_pose(pill_name[-1]),5, 0.08-0.003) # z_offset has nothing to do with depth that it 'digs' into canister
            auboi5_controller.place_one_time(auboi5_controller.find_place_pose(pill_name[-1]), 0.04)  # normally use this
            # auboi5_controller.place_one_time(auboi5_controller.medicine_box_position(1, 1))

        while True:
            # Look at the platform
            auboi5_controller.moveJ(auboi5_controller.find_capture_position(pill_name[-1])) 
            
            cam.capture()
            cam.capture()
            img = cam.color_image
            img_ins_seg, img_sem_seg, centroids = pill_segmentation_mask_first(img)
            # img_ins_seg, img_sem_seg = pill_segmentation(img)
            num_pill_on_platform = grasp.check_obj_exist(img, img_ins_seg, img_sem_seg, vis_checkexist)

            
            # Pick&Place-loop on platform
            if num_pill_on_platform > 0:
                # If there is any pill on the platform

                
                if pick_todo_num <= 0:
                # Reflow (Existed but order fulfilled)
                    # reflow to storehouse from platform
                    print("\n reflow ...")

                    plt.close('all') 
                    fig, axs = create_axs(cam.color_image, 1, 'Human-in-loop pushing')

                    motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.push(img, auboi5_controller.cTo_z_)
                
                else:
                # Pick (Existed and order not fulfilled)
                    # Create axs if the six subplots were closed / modified
                    if len(plt.get_fignums())==0 or len(fig.figure.axes) != 2:
                        fig, axs = create_axs(cam.color_image, 2, 'Segmentation Results')
                    
                    motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.think(img, img_ins_seg,
                                                                                            img_sem_seg, auboi5_controller.cTo_z_, vis_grasp, axs)
                
                fig.canvas.draw()
                fig.canvas.flush_events()
                
                # revise the sys x error in vision
                push_start[0] += vision_x_error
                push_end[0] += vision_x_error
                grasp_coord[0] += vision_x_error

                # Results
                # 0:pushing, 1:swiping, >=2:picking
                if motion_command == 0: 
                    print('push_start:', push_start)
                    print('push_end:', push_end)
                    ps_xy = auboi5_controller.eye_hand_transfer(push_start)
                    pe_xy = auboi5_controller.eye_hand_transfer(push_end)
                    auboi5_controller.set_aside(ps_xy, pe_xy)

                elif motion_command == 1:
                    print('sweep_start:', push_start)
                    print('sweep_end:', push_end)
                    ps_xy = auboi5_controller.eye_hand_transfer(push_start)
                    pe_xy = auboi5_controller.eye_hand_transfer(push_end)
                    auboi5_controller.set_sweep(ps_xy, pe_xy)
                    

                elif motion_command >= 2 and motion_command<=7:
                    print('grasp_coord:', grasp_coord)
                    print('grasp_angle:', grasp_angle)
                    print('grasp_opening:', grasp_opening)
                    print('class:', motion_command)  # pill_a:2, pill_b:3, pill_c:4, pill_d:5, pill_e:6, pill_f:7

                    p1_xy = auboi5_controller.eye_hand_transfer(grasp_coord)
                    rad = auboi5_controller.rad_transfer(grasp_angle)
                    p1 = auboi5_controller.assign_pick_point(p1_xy[0], p1_xy[1], rad, auboi5_controller.pick_z_)
                    width1 = auboi5_controller.opening_width_mapping(grasp_opening)
                    print('opening width =', width1, grasp_opening)
                    auboi5_controller.pick_one_time(p1,width1)

                    # place_row, place_col = auboi5_controller.decide_place_row_col(motion_command)
                    day = 1
                    place_row, place_col = place_manager(pick_todo_num, presc_list, day)
                    place = auboi5_controller.medicine_box_position(place_row, place_col)
                    auboi5_controller.place_one_time(place)
                    # auboi5_controller.led_control(place_row, place_col)
                    
                    # Update
                    pick_todo_num -= 1
                    
            else:
                # If nothing is on the platform
                if pick_todo_num <= 0:
                    # no need to grasp SOME or reflow
                    break
                
                # Grasp SOME without looking
                mc.go_position(mc.find_box(pill_name[-1]))
                time.sleep(0.1)
                auboi5_controller.moveJ(auboi5_controller.find_capture_position(pill_name[-1])) 
                auboi5_controller.pick_one_time(auboi5_controller.find_grasping_pose(pill_name[-1]),5)
                auboi5_controller.place_one_time(auboi5_controller.find_place_pose(pill_name[-1]))
            


        # while pick_todo_num > 0 and platform_clear:
        #     # Realsense; Segmentation; Grasp Generation
        #     cam.capture()  # get one frame
        #     img = cam.color_image
        #     img_ins_seg, img_sem_seg = pill_segmentation(img)
        #     vis = False  # visualize grasp detection results
        #     motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.think(img, img_ins_seg,
        #                                                                                                 img_sem_seg, auboi5_controller.cTo_z_, vis)

        #     # revise the sys x error in vision
        #     push_start[0] += vision_x_error
        #     push_end[0] += vision_x_error
        #     grasp_coord[0] += vision_x_error

        #     # Results
        #     # 0:pushing, 1:swiping, >=2:picking
        #     if motion_command == 0:
        #         print('push_start:', push_start)
        #         print('push_end:', push_end)
        #         ps_xy = auboi5_controller.eye_hand_transfer(push_start)
        #         pe_xy = auboi5_controller.eye_hand_transfer(push_end)
        #         auboi5_controller.set_aside(ps_xy, pe_xy)

        #     elif motion_command == 1:
        #         print('sweep_start:', push_start)
        #         print('sweep_end:', push_end)
        #         ps_xy = auboi5_controller.eye_hand_transfer(push_start)
        #         pe_xy = auboi5_controller.eye_hand_transfer(push_end)
        #         auboi5_controller.set_sweep(ps_xy, pe_xy)

        #     elif motion_command >= 2 and motion_command<=7:
        #         print('grasp_coord:', grasp_coord)
        #         print('grasp_angle:', grasp_angle)
        #         print('grasp_opening:', grasp_opening)
        #         print('class:', motion_command)  # pill_a:2, pill_b:3, pill_c:4, pill_d:5, pill_e:6, pill_f:7

        #         p1_xy = auboi5_controller.eye_hand_transfer(grasp_coord)
        #         rad = auboi5_controller.rad_transfer(grasp_angle)
        #         p1 = auboi5_controller.assign_pick_point(p1_xy[0], p1_xy[1], rad, auboi5_controller.pick_z_)
        #         width1 = auboi5_controller.opening_width_mapping(grasp_opening)
        #         print('opening width =', width1, grasp_opening)
        #         auboi5_controller.pick_one_time(p1,width1)

        #         # place_row, place_col = auboi5_controller.decide_place_row_col(motion_command)
        #         place_row, place_col = place_manager(pick_todo_num, presc_list, day)
        #         place = auboi5_controller.medicine_box_position(place_row, place_col)
        #         auboi5_controller.place_one_time(place)
        #         # auboi5_controller.led_control(place_row, place_col)
        #         pick_todo_num -= 1  # update
                
        #     else:
        #         print('Unknown motion type.')

            


