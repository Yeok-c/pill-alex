from auboi5_controller import AuboController

auboi5_controller = AuboController('192.168.1.115')

while True:
    auboi5_controller.pick_one_time(auboi5_controller.find_grasping_pose('A'),5, 0.08-0.003) # z_offset has nothing to do with depth that it 'digs' into canister
    auboi5_controller.place_one_time(auboi5_controller.find_place_pose('A'), 0.04)  # normally use this