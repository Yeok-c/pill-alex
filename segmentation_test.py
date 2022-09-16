from demo.demo_pill import pill_segmentation, pill_segmentation_mask_first
from centermask.grasp import grasp
from centermask.realsense_capture import d415_frames
import matplotlib.pyplot as plt
from figure.utils import create_axs

cam = d415_frames.camera() # Init camera
for i in range(20): # Capture many images for camera to calibrate itself
    cam.capture()

for i in range(50):
    cam.capture()
    img = cam.color_image
    img_ins_seg, img_sem_seg, centroids = pill_segmentation_mask_first(img)
    


    if len(plt.get_fignums())==0 or len(fig.figure.axes) != 2: # There is no figure open or number of axes is incorrect
        fig, axs = create_axs(cam.color_image, 2, 'Segmentation Results') # Init figure

    motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.think(img, img_ins_seg,
                                                                        img_sem_seg, 0.1, True, axs)
                                                                        # img_sem_seg, auboi5_controller.cTo_z_, vis_grasp)
    fig.canvas.draw()
    fig.canvas.flush_events()

