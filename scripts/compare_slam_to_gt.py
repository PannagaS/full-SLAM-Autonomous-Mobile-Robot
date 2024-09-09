import time
import numpy as np
import lcm
import sys
import os
sys.path.append('/usr/lib/python3.9/site-packages/')
from mbot_lcm_msgs.pose2D_t import pose2D_t
from mbot_lcm_msgs.twist2D_t import twist2D_t

gt_pose   = []
slam_pose = []

def gt_pose_handler(channel, data):
    msg = pose2D_t.decode(data)
    gt_pose.append([time.time_ns()*10e-9, msg.x, msg.y, 0, 0, 0, 0, 1]) 
    print("Recoding gt odom... Length: %f, %f, %f, %f" % (time.time_ns()*10e-9, msg.x, msg.y, msg.theta))

def slam_pose_handler(channel, data):
    msg = pose2D_t.decode(data)
    slam_pose.append([time.time_ns()*10e-9, msg.x, msg.y, 0, 0, 0, 0, 1]) 
    print("Recoding slam odom... Length: %f, %f, %f, %f" % (time.time()*10e-9, msg.x, msg.y, msg.theta))
    
print('Argument List:', str(sys.argv))
filename = str(sys.argv[1])
gt_pose_filename, slam_pose_filename = "gt_"+filename, "slam_"+filename

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
time.sleep(0.5)
gt_pose_subscription = lc.subscribe("GROUND_TRUTH_POSE", gt_pose_handler)
slam_pose_subscription = lc.subscribe("SLAM_POSE", slam_pose_handler)

try:
    while True:
        lc.handle()
        time.sleep(0.05)
except KeyboardInterrupt:
    print("Done")
    try:
        os.remove(gt_pose_filename)
        print("%s exsits, removing..." % gt_pose_filename)
        os.remove(slam_pose_filename)
        print("%s exsits, removing..." % slam_pose_filename)
    except OSError: 
        pass
    gt_pose = np.array(gt_pose)
    gt_t0 = gt_pose[0,0]
    gt_xy0 = gt_pose[0,1:3]
    gt_pose[:,0] = (gt_pose[:,0] - gt_t0)
    gt_pose[:,1:3] = gt_pose[:,1:3] - gt_xy0

    slam_pose = np.array(slam_pose)
    slam_t0 = slam_pose[0,0]
    slam_xy0 = slam_pose[0,1:3]
    slam_pose[:,0] = (slam_pose[:,0] - slam_t0)
    slam_pose[:,1:3] = slam_pose[:,1:3] - slam_xy0

    print("Saving to file: %s" % gt_pose_filename)
    np.savetxt(gt_pose_filename, gt_pose)
    print("Saving to file: %s" % slam_pose_filename)
    np.savetxt(slam_pose_filename, slam_pose)
    pass