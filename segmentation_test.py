
from demo.demo_pill import pill_segmentation, pill_segmentation_mask_first, get_parser, setup_cfg, VisualizationDemo
from centermask.grasp import grasp
from centermask.realsense_capture import d415_frames
import matplotlib.pyplot as plt
from figure.utils import create_axs

# Create obj
args = get_parser().parse_args()
cfg = setup_cfg(args)    
demo = VisualizationDemo(cfg, parallel=False)


cam = d415_frames.camera() # Init camera
for i in range(20): # Capture many images for camera to calibrate itself
    cam.capture()
    
for i in range(50):
    cam.capture()
    img = cam.color_image
    img_ins_seg, img_sem_seg, centroids = pill_segmentation_mask_first(img, demo)

    if len(plt.get_fignums())==0 or len(fig.figure.axes) != 1: # There is no figure open or number of axes is incorrect
        fig, axs = create_axs(cam.color_image, 1, 'Segmentation Results') # Init figure

    motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.think(img, img_ins_seg,
                                                                            img_sem_seg, 0.1, True, axs, centroids, 'A')

    fig.canvas.draw()
    fig.canvas.flush_events()

