from auboi5_controller import AuboController
import time
import numpy as np
'''
Full pseudocode
1. Start suction
while not suc_cess 
    2. Go to point above and go down to position based on distribution
    3. Update suc_cess based on sensor readings
4. Return to plate
'''
x_rand_range = 0.0
y_rand_range = 0.0
def get_grasping_pose(id):
    R =  [0.00032001149681460804, -0.9998324819174745, 0.01829231174002348, 0.0005450014594178438]
    X = 0.164, 0.224, 0.287, 0.349 #canister1 to 8
    Y = -0.376
    Z = 0.21 # - 0.015 #+0.005 # drawer 1,2,3
    grasp_transform = [X[id], Y, Z]
    grasp_transform[0]+=np.random.uniform(low=-x_rand_range, high=x_rand_range, size=None)
    grasp_transform[1]+=np.random.uniform(low=-y_rand_range, high=y_rand_range, size=None)
    grasp_transform.extend(R)
    return grasp_transform

def get_destination_pose():
    R =  [0.00032001149681460804, -0.9998324819174745, 0.01829231174002348, 0.0005450014594178438]
    X = 0.2986 # 4 rows 
    Y = -0.292 # 7 cols
    Z = 0.22+0.03
    grasp_transform = [X, Y, Z]
    grasp_transform[0]+=np.random.uniform(low=-x_rand_range, high=x_rand_range, size=None)
    grasp_transform[1]+=np.random.uniform(low=-y_rand_range, high=y_rand_range, size=None)
    grasp_transform.extend(R)
    return grasp_transform


t = time.time()
def tim(last_time, task_name):
    current_time = time.time()
    past_time = current_time - last_time
    print("Time elapsed for: ", task_name, " - ", past_time)
    return current_time
# usage
# t = tim(t)
# both prints out time elapsed and records down current time

if __name__ == "__main__":
    # robot
    auboi5_controller = AuboController('192.168.1.115')# 115
    time_start = time.time()
    suck_counter = 0
    place_counter = 0
    id = 1 # 8 canister 3 drawer 

    while 1:
        suc_cess=False 
        auboi5_controller.go_to_above_picking_pose(get_grasping_pose(id),5, zoffset=0.10)
        auboi5_controller.pick_one_time(get_grasping_pose(id),5, zoffset=0.03)
    #     # auboi5_controller.moveJ(auboi5_controller.find_capture_position(pill_name)) 
        # while suc_cess == False: # nothing is gripped
        #     auboi5_controller.pick_one_time(get_grasping_pose(id),5, zoffset=0.03)
        #     time.sleep(0.01)
        #     suc_cess = auboi5_controller.suction.read_sucker_state()
        #     suck_counter = suck_counter + 1
        auboi5_controller.go_to_above_picking_pose(get_grasping_pose(id),5, zoffset=0.10)
        # Exit loop when ==1 so when something is picked
        print("placing")
        auboi5_controller.place_one_time(get_destination_pose())
        place_counter = place_counter + 1

        elapsed_time = time.time() - time_start
        print("Elapsed time: ", elapsed_time)
        if elapsed_time > 60*5: # 5 min time
            print("5 minutes completed. Attempted suction: ", suck_counter)
            print(". Successful suction and placement: ", place_counter)
            break
    #     time.sleep(0.2)

# Elapsed time:  304.1273694038391
# 5 minutes completed. Attempted suction:  60
# . Successful suction and placement:  27

# Elapsed time:  303.1065378189087
# 5 minutes completed. Attempted suction:  79
# . Successful suction and placement:  20

# Elapsed time:  300.9124104976654
# 5 minutes completed. Attempted suction:  70
# . Successful suction and placement:  23

# Peppermint (soft, yellow gel-capsules)
# Attempts: 72, Success: 25

# Pregabalin (small white/red capsules)
# Attempts: 50, Success: 31, +1 false positive

# AT20 (Oval, small white tablets)  
# Attempts: 58, Success:  27

# Quetiapine-Teva (orange, very small round tablets)
# Attempts: 49, Success: 30, +6 false positive
