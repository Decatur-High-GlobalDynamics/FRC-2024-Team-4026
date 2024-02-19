package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;

public final class VisionConstants
{

    public static final String CameraTableName = "camera_front";

    public static final Pose3d CameraOffset = new Pose3d();

    public static final int BLUE_SPEAKER_TAG_ID = 7, RED_SPEAKER_TAG_ID = 4;

    /** How close the chassis needs to be to shoot a note. In radians */
    public static final double CHASSIS_AIM_THRESHOLD = Math.toRadians(3);

    public static final boolean ADJUST_SPEAKER_POSE_FOR_VELOCITY = true;

}
