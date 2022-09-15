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