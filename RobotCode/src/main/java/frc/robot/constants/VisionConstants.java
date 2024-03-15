package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class VisionConstants
{

    public static final String CameraTableName = "camera_front";

    public static final Pose3d CameraOffset = new Pose3d();

    public static final int BLUE_SPEAKER_TAG_ID = 7, RED_SPEAKER_TAG_ID = 4;

    /** How close the chassis needs to be to shoot a note. In radians */
    public static final double CHASSIS_AIM_THRESHOLD = Math.toRadians(3);

    public static final boolean ADJUST_SPEAKER_POSE_FOR_VELOCITY = true;

	public static final Transform3d ROBOT_TO_CAMERA_OFFSET = new Transform3d(new Translation3d(0.14732, 0.0, 0.26035),
				new Rotation3d(0, 1.02974, 1.32645));

}
