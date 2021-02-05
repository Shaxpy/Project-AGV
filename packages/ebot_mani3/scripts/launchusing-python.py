import time
import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(
    uuid, ['/home/shaxpy/arm_ws/src/ebot_description/launch/image_save.launch'])
launch.start()

time.sleep(1.3)

launch.shutdown()
