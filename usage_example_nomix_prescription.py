from centermask.grasp import grasp
from demo.demo_pill import pill_segmentation
from detectron2.data.detection_utils import read_image
from centermask.realsense_capture import d415_frames
from auboi5_controller import AuboController
import time
import math
from copy import deepcopy
import cv2
import serial


def place_manager(pick_todo_num, presc_list, day):
    target_timing = len(presc_list) - pick_todo_num
    place_row = day  # one day of a week (0:Mon, ..., 6:Sun)
    place_col = presc_list[target_timing]  # one timing of a day

    return place_row, place_col



if __name__ == "__main__":
    push_exp_flag = False

    auboi5_controller = AuboController('192.168.1.115')
    # waypoint = auboi5_controller.getCurrentWaypoint()
    # print(waypoint['joint'])
    
    # Prescription for one day
    # prescription = {'pill_A':2, 'pill_B':3, 'pill_C':0, 'pill_D':1, 'pill_E':1, 'pill_F':1, 'pill_G':1, 'pill_H':1, 'pill_I':1, 'pill_J':1}
    prescription_timing = {'pill_A':[0,2], 'pill_B':[3,3], 'pill_C':[0,1,2,3], 'pill_D':[0,2], 'pill_E':[1,1], 'pill_F':[0,3], 'pill_G':[1,3], 'pill_H':[0,0,2,2], 'pill_I':[1], 'pill_J':[3]}
    # TIMING: 0 for 8:00, 1 for 12:00, 2 for 17:00, 3 for 21:00 
    
    cam = d415_frames.camera()
    for i in range(20):
        cam.capture()   # get one frame
    
    vision_x_error = -0.016

    for day in range(7):
        # From Monday to Sunday

        #Process the prescription
        for pill_name, presc_list in prescription_timing.items():
            print(pill_name, presc_list)
            
            pick_todo_num = len(presc_list)

            while pick_todo_num > 0:
                
                #init
                auboi5_controller.moveJ(auboi5_controller.find_capture_position(pill_name[-1]))

                # Realsense; Segmentation; Grasp Generation
                cam.capture()  # get one frame
                cam.capture()
                img = cam.color_image
                img_ins_seg, img_sem_seg = pill_segmentation(img)
                vis = False  # visualize grasp detection results
                motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.think(img, img_ins_seg,
                                                                                                            img_sem_seg, auboi5_controller.cTo_z_, vis)

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

                    if push_exp_flag == False:
                        p1_xy = auboi5_controller.eye_hand_transfer(grasp_coord)
                        rad = auboi5_controller.rad_transfer(grasp_angle)
                        p1 = auboi5_controller.assign_pick_point(p1_xy[0], p1_xy[1], rad, auboi5_controller.pick_z_)
                        width1 = auboi5_controller.opening_width_mapping(grasp_opening)
                        print('opening width =', width1, grasp_opening)
                        auboi5_controller.pick_one_time(p1,width1)

                        # place_row, place_col = auboi5_controller.decide_place_row_col(motion_command)
                        place_row, place_col = place_manager(pick_todo_num, presc_list, day)
                        place = auboi5_controller.medicine_box_position(place_row, place_col)
                        auboi5_controller.place_one_time(place)
                        # auboi5_controller.led_control(place_row, place_col)
                        pick_todo_num -= 1  # update
                    
                else:
                    print('Unknown motion type.')


