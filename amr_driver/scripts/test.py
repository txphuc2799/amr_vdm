import tf.transformations as tf

# Define the yaw angle (in radians)
yaw = 0.0  # Example yaw angle (90 degrees in radians)

# Create a quaternion from yaw (roll and pitch are set to 0)
quaternion = tf.quaternion_from_euler(0, 0, yaw)

print("Quaternion:", quaternion)

import tf.transformations as tf

# Example quaternion [x, y, z, w]
quaternion = [0,0, 0.708727, 0.705483]

# Convert quaternion to Euler angles (roll, pitch, yaw)
euler = tf.euler_from_quaternion(quaternion)

# Extract yaw (third component of Euler angles)
yaw = euler[2]

print("Yaw (radians):", yaw)


c = b'\x00\x02\x03'

print(int.from_bytes(c, byteorder='little'))

