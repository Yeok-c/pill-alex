from demo_pill import pill_segmentation
from centermask.grasp import grasp
from centermask.realsense_capture import d415_frames
import matplotlib.pyplot as plt 
import cv2 as cv
cam = d415_frames.camera()

for i in range(20):
    cam.capture()

mask_corners = [[550,190], [870,445]]
# mask_corners = [[540,225], [831,445]]

for i in range(50):
    cam.capture()
    img = cam.color_image
    # img_ins_seg, img_sem_seg = pill_segmentation(img)
    # motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.think(img, img_ins_seg,
    #                                                                     img_sem_seg, 0.1, True)
                                                                        # img_sem_seg, auboi5_controller.cTo_z_, vis_grasp)
    # img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # _, opening = cv.threshold(img, 1, 255, cv.THRESH_BINARY)

    fig, axs = plt.subplots(2, 1, constrained_layout=True)
    axs = axs.flatten()
    axs[0].imshow(img)

    # wipe all detections outside the manipulation area
    img[:,:mask_corners[0][0]  ,:] = 1
    img[:,mask_corners[1][0]:-1,:] = 1
    img[:mask_corners[0][1],:  ,:] = 1
    img[mask_corners[1][1]:-1,:,:] = 1

    # do not detect gripper as pill
    img[0:45, 780:920,:] = 1
    axs[1].imshow(img)
    plt.show()

