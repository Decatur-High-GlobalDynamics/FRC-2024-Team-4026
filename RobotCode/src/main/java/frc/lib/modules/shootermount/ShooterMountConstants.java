package frc.robot.constants;

public class ShooterMountConstants
{

	/** Highest position the shooter mount target position can be set to in rotations */
	public static final double SHOOTER_MOUNT_MAX_ANGLE_OFFSET = 0.3;

	/** Lowest position of the shooter in radians */
	public static final double SHOOTER_MOUNT_MIN_ANGLE_IN_RADIANS = Math.toRadians(9.57);

	/** Angle in rotations for shooting at amp */
	public static final double SHOOTER_MOUNT_INITIAL_AMP_ANGLE_OFFSET = 0.16;
	public static final double SHOOTER_MOUNT_ENDING_AMP_ANGLE_OFFSET = 0.05;
	/** Angle in rotations for shooting at speaker */
	public static final double SHOOTER_MOUNT_SPEAKER_ANGLE_FIXED_OFFSET = 0.13; //0.125;
	public static final double SHOOTER_MOUNT_NOTE_CENTER_ANGLE_FIXED_OFFSET = 0.071; //0.066;
	public static final double SHOOTER_MOUNT_PODIUM_ANGLE_FIXED_OFFSET = 0.066; //0.061;
	public static final double SHOOTER_MOUNT_PASSING_ANGLE_FIXED_OFFSET = 0.11; //0.105;

	public static final double SHOOTER_MOUNT_KP = 0.38;
	public static final double SHOOTER_MOUNT_KI = 0;
	public static final double SHOOTER_MOUNT_KD = 0.1;
	public static final double SHOOTER_MOUNT_KS = 0;
	public static final double SHOOTER_MOUNT_KV = 0.71;
	public static final double SHOOTER_MOUNT_KA = 0;
	public static final double SHOOTER_MOUNT_KG = 0.0225;

	/** Rotations per second */
	public static final double SHOOTER_MOUNT_CRUISE_VELOCITY = 3;
	/** Rotations per second per second */
	public static final double SHOOTER_MOUNT_ACCELERATION = 2;

	/** Tolerance for determining mount is within range to shoot in rotations */
	public static final double AIMING_DEADBAND = 0.05;

	/** Distance to speaker in meters */
	public static final double[] SpeakerDistanceTreeMapKeys =
	{
			1, 2, 3
	};
	/** Rotation for aiming in rotations */
	public static final double[] ShooterMountAngleTreeMapValues =
	{
			0.1, 0.1, 0.1
	};
	/** Note flight time in seconds */
	public static final double[] NoteFlightTimeEstimateTreeMapValues =
	{
			0, 0, 0
	};
}
