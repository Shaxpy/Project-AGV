def akhil:
	rospy.init_node('taskfive', anonymous=True)

        ur5 = Ur5Moveit()

        north = [0.0, 1.0]
        south = [1.0, 0.0]
        east = [-0.7, 0.7]
        west = [0.7, 0.7]
        northeast = [-0.289, 0.957]
        southwest = [0.949, 0.315]

        # Arm pose for better movement
        movearm = [math.radians(-22),
                   math.radians(-25),
                   math.radians(20),
                   math.radians(-25),
                   math.radians(-29),
                   math.radians(-17)]

        # pantry = geometry_msgs.msg.Pose()
        # pantry.position.x = 0.463088
        # pantry.position.y = 0.226275
        # pantry.position.z = 0.870000
        # pantry.orientation.x = -0.30060515
        # pantry.orientation.y = -0.63710335
        # pantry.orientation.z = 0.63818203
        # pantry.orientation.w = 0.31057938

        ur5.set_joint_angles(movearm)

        # # pantry outside
        client_coordinates(12.96, 1.27, east)

        # # pantry inside
        client_coordinates(13.12, -1.0, north)

        # # pantry left table
        client_coordinates(14.4, -1.0, north)

        # ur5.set_joint_angles(pantry)

        # pantry left table turn toward south
        client_coordinates(14.4, -1.0, south)

        # # pantry right table
        client_coordinates(11.6, -1.0, south)

        # # pantry right table turn toward north
        client_coordinates(11.6, -1.0, north)

        # # # pantry inside
        client_coordinates(13.12, -1.0, west)

        # # # pantry outside
        client_coordinates(12.96, 1.27, south)

        # # # meeting outside
        client_coordinates(8.57, 1.09, west)

        # # # # meeting inside
        client_coordinates(8.58, 2.71, south)

        # # # # meeting drop box
        client_coordinates(7.4, 2.71, north)

        # # # meeting inside
        client_coordinates(8.58, 2.71, east)

        # # # meeting outside
        client_coordinates(8.57, 1.09, north)

        # research
        client_coordinates(10.6, 9.48, east)

        # store
        client_coordinates(25.8, -3.05, northeast)

        # store return
        client_coordinates(25.8, -3.05, southwest)

        # last room outside
        client_coordinates(5.19, 0.85, east)

        # last room inside v1
        client_coordinates(5.6, -0.67, east)

        # # last room inside v2
        # client_coordinates(4.93, -1.18, east)
