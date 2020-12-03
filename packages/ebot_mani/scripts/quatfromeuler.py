from tf.transformations import quaternion_from_euler

roll = -1.546654

pitch = -0.035205

yaw = -2.590874

quat = quaternion_from_euler(roll, pitch, yaw)

print quat
