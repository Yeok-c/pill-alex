import matplotlib.pyplot as plt

def create_axs(init_img, subplot_num, suptitle_text):
    if subplot_num > 1:
        fig, AXS = plt.subplots(2, subplot_num//2, constrained_layout=True)
        AXS = AXS.flatten()
        axs=[]
        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
        for i, AX in enumerate(AXS):
            axs.append(AX.imshow(init_img))    
    else:
        fig, axs = plt.subplots(1, 1, constrained_layout=True)
    
    fig.suptitle(suptitle_text)
    return fig, axs

