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


if __name__ == "__main__":
    push_exp_flag = False

    auboi5_controller = AuboController('192.168.1.115')
    # waypoint = auboi5_controller.getCurrentWaypoint()
    # print(waypoint['joint'])
    
    # Prescription for one day
    prescription = {'pill_1':1, 'pill_2':1, 'pill_3':1, 'pill_4':1, 'pill_5':1, 'pill_6':1, 'pill_7':1, 'pill_8':1, 'pill_9':1, 'pill_0':1}
    
    cam = d415_frames.camera()
    for i in range(20):
        cam.capture()   # get one frame
    
    vision_x_error = -0.016


    #Process the prescription
    for key, values in prescription.items():
        print(key,values)
        
        pick_up_flag = False

        while not pick_up_flag:
            
            #init
            auboi5_controller.moveJ(auboi5_controller.find_capture_position(int(key[-1])))

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

                    place_row, place_col = auboi5_controller.decide_place_row_col(motion_command)
                    place = auboi5_controller.medicine_box_position(place_row, place_col)
                    auboi5_controller.place_one_time(place)
                    # auboi5_controller.led_control(place_row, place_col)
                    pick_up_flag = True
                
            else:
                print('Unknown motion type.')


