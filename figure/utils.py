import matplotlib.pyplot as plt
import numpy as np

def create_axs(init_img, subplot_num, suptitle_text=''):

    plt.close('all') 

    if subplot_num > 1:
        fig, AXS = plt.subplots(2, subplot_num//2, constrained_layout=True)
        AXS = AXS.flatten()
        axs=[]
        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
        for i, AX in enumerate(AXS):
            axs.append(AX.imshow(init_img))    
    else:
        fig, ax = plt.subplots(constrained_layout=True)
        axs = ax.imshow(init_img)
        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
    
    fig.suptitle(suptitle_text)
    plt.draw()
    plt.pause(0.001)

    return fig, axs

