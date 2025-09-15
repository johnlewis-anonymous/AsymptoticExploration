#!/usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def plot_bounding_box(corners, world_frame, fixed_frame):
    rospy.init_node('bounding_box_plotter', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    listener = tf.TransformListener()

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            # Wait for the transform to be available
            listener.waitForTransform(fixed_frame, world_frame, rospy.Time(), rospy.Duration(4.0))
            
            # Create a marker
            marker = Marker()
            marker.header.frame_id = fixed_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "bounding_box"
            marker.id = 0
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1  # Size of the points
            marker.scale.y = 0.1
            marker.color.a = 1.0  # Alpha
            marker.color.r = 1.0  # Red

            # Transform each corner and add to marker points
            for corner in corners:
                (trans, rot) = listener.lookupTransform(fixed_frame, world_frame, rospy.Time(0))
                transform_matrix = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans),
                tf.transformations.quaternion_matrix(rot)
            )
            for corner in corners:
                # Convert corner to homogeneous coordinates
                corner_homogeneous = list(corner) + [1.0]
                
                # Apply the transformation matrix
                transformed_corner_homogeneous = transform_matrix.dot(corner_homogeneous)
                
                # Convert back to 3D point
                transformed_corner = Point(*transformed_corner_homogeneous[:3])
                
                marker.points.append(transformed_corner)
                marker.scale.x = 5
                marker.scale.y = 5
                marker.scale.z = 5

            # Publish the marker
            marker_pub.publish(marker)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    # Example corners of a bounding box in the world_frame
    corners = [
    (55.0, 55.0, 0.0),
    (55.0, -55.0, 0.0),
    (-55.0, -55.0, 0.0),
    (-55.0, 55.0, 0.0),
    (55.0, 55.0, 3.0),
    (55.0, -55.0, 3.0),
    (-55.0, -55.0, 3.0),
    (-55.0, 55.0, 3.0)]

    world_frame = "uav1/world_origin"
    fixed_frame = "uav1/map_locus"

    try:
        plot_bounding_box(corners, world_frame, fixed_frame)
    except rospy.ROSInterruptException:
        pass
