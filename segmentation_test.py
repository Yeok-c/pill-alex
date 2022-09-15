from demo.demo_pill import pill_segmentation, pill_segmentation_mask_first
from centermask.grasp import grasp
from centermask.realsense_capture import d415_frames
import matplotlib.pyplot as plt

def create_axs(init_img):
    fig, AXS = plt.subplots(2, 3, constrained_layout=True)
    AXS = AXS.flatten()
    axs=[]
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    for i, AX in enumerate(AXS):
        axs.append(AX.imshow(init_img))    
    fig.suptitle('Segmentation Results')
    return fig, axs

cam = d415_frames.camera() # Init camera
for i in range(20): # Capture many images for camera to calibrate itself
    cam.capture()

fig, axs = create_axs(cam.color_image) # Init figure

for i in range(50):
    cam.capture()
    img = cam.color_image
    img_ins_seg, img_sem_seg = pill_segmentation_mask_first(img)

    # fig._suptitle._text == 'Segmentation Results'
    if len(plt.get_fignums())==0 or len(fig.figure.axes) != 6: # There is no figure open or number of axes is incorrect
        fig, axs = create_axs(cam.color_image) # Init figure

    motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.think(img, img_ins_seg,
                                                                        img_sem_seg, 0.1, True, axs)
                                                                        # img_sem_seg, auboi5_controller.cTo_z_, vis_grasp)
    fig.canvas.draw()
    fig.canvas.flush_events()

