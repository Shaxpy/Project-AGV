from tf.transformations import quaternion_from_euler

roll = -1.563324

pitch = -0.012061

yaw = 2.247722

quat = quaternion_from_euler(roll, pitch, yaw)

print quat
