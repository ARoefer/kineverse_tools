from visualization_msgs.msg import InteractiveMarkerControl as InteractiveMarkerControlMsg

def make6DOFGimbal(intMarker):
    """Creates 6DOF gimbal marker controls."""
    pitch = InteractiveMarkerControlMsg()
    pitch.orientation.x = 1
    pitch.orientation.y = 0
    pitch.orientation.z = 0
    pitch.orientation.w = 1
    pitch.interaction_mode = InteractiveMarkerControlMsg.MOVE_3D
    pitch.orientation_mode = InteractiveMarkerControlMsg.FIXED
    intMarker.controls.append(pitch)
    
    pitch = InteractiveMarkerControlMsg()
    pitch.orientation.x = 1
    pitch.orientation.y = 0
    pitch.orientation.z = 0
    pitch.orientation.w = 1
    pitch.interaction_mode = InteractiveMarkerControlMsg.ROTATE_AXIS
    pitch.orientation_mode = InteractiveMarkerControlMsg.FIXED
    intMarker.controls.append(pitch)

    yaw = InteractiveMarkerControlMsg()
    yaw.orientation.x = 0
    yaw.orientation.y = 1
    yaw.orientation.z = 0
    yaw.orientation.w = 1
    yaw.interaction_mode = InteractiveMarkerControlMsg.ROTATE_AXIS
    yaw.orientation_mode = InteractiveMarkerControlMsg.FIXED
    intMarker.controls.append(yaw)

    roll = InteractiveMarkerControlMsg()
    roll.orientation.x = 0
    roll.orientation.y = 0
    roll.orientation.z = 1
    roll.orientation.w = 1
    roll.interaction_mode = InteractiveMarkerControlMsg.ROTATE_AXIS
    roll.orientation_mode = InteractiveMarkerControlMsg.FIXED
    intMarker.controls.append(roll)

    transX = InteractiveMarkerControlMsg()
    transX.orientation.x = 1
    transX.orientation.y = 0
    transX.orientation.z = 0
    transX.orientation.w = 1
    transX.interaction_mode = InteractiveMarkerControlMsg.MOVE_AXIS
    transX.orientation_mode = InteractiveMarkerControlMsg.FIXED
    intMarker.controls.append(transX)

    transY = InteractiveMarkerControlMsg()
    transY.orientation.x = 0
    transY.orientation.y = 1
    transY.orientation.z = 0
    transY.orientation.w = 1
    transY.interaction_mode = InteractiveMarkerControlMsg.MOVE_AXIS
    transY.orientation_mode = InteractiveMarkerControlMsg.FIXED
    intMarker.controls.append(transY)

    transZ = InteractiveMarkerControlMsg()
    transZ.orientation.x = 0
    transZ.orientation.y = 0
    transZ.orientation.z = 1
    transZ.orientation.w = 1
    transZ.interaction_mode = InteractiveMarkerControlMsg.MOVE_AXIS
    transZ.orientation_mode = InteractiveMarkerControlMsg.FIXED
    intMarker.controls.append(transZ)