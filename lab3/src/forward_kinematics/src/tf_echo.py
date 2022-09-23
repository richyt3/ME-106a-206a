import sys
import rospy
import tf2_ros

def tf_echo_func():
	if len(sys.argv) > 2:
		target_frame = sys.argv[1] 
		source_frame = sys.argv[2]
	else:
		print('Need both target frame and source frame as arguments.')
		return

	# Stores transform information.
	tfBuffer = tf2_ros.Buffer()
	# Subscribes to `tf` topic and maintains the `tf` graph inside the Buffer.
	tfListener = tf2_ros.TransformListener(tfBuffer)

	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
			print(trans.transform)
		except tf2_ros.LookupException:
			# print('Lookup Exception occured.')
			continue
		except tf2_ros.ConnectivityException:
			# print('Connectivity Exception occured.')
			continue
		except tf2_ros.ExtrapolationException:
			# print('Extrapolation Exception occured.')
			continue

	# print(f"Target Frame:\n{target_frame}")
	# print(f"Source Frame:\n{source_frame}")
	

if __name__ == "__main__":
	rospy.init_node('listener', anonymous=True)

	tf_echo_func()
