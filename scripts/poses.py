# angle = math.pi/2.0
angle = 0.0
theta = math.pi/4
max_r = 0.50
translate = numpy.array([[1, 0, 0, max_r * math.cos(theta)],
                        [0, 1, 0, max_r * math.sin(theta)],
                        [0, 0, 1, 0.25],
                        [0, 0, 0, 1]]) #optimal height and a random x y translation
end_pose_y = numpy.array([[math.cos(angle), 0, math.sin(angle), 0.0],
                        [0, 1, 0, 0.0],
                        [-math.sin(angle), 0, math.cos(angle), 0.0],
                        [0, 0, 0, 1]])
end_pose_x = numpy.array([[1, 0, 0, 0.0],
                        [0, math.cos(angle), -math.sin(angle), 0.0],
                        [0, math.sin(angle), math.cos(angle), 0.0],
                        [0, 0, 0, 1]])

# z_angle = 0.4 #math.pi
z_angle = math.pi/2 + theta
z_rot = numpy.array([[math.cos(z_angle), -math.sin(z_angle), 0, 0.0],
                    [math.sin(z_angle), math.cos(z_angle), 0, 0.0],
                    [0, 0, 1, 0.0],
                    [0, 0, 0, 1]])



end_pose_final = numpy.dot(numpy.dot(translate, end_pose_x),z_rot)

traj = PlanToTransform(env,robot,end_pose_final)

# y_angle = math.pi/3+.45
# y_angle = 0.0
# y_angle = -math.pi/2
# y_angle = 0 # -math.pi/4
# y_rot = numpy.array([[math.cos(y_angle), 0, math.sin(y_angle), 0.0],
# 	                [0, 1, 0, 0.0],
# 	                [-math.sin(y_angle), 0, math.cos(y_angle), 0.0],
# 	                [0, 0, 0, 1]])