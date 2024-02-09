package frc.robot.constants;

public class ShooterMountConstants
{

	public static final double TICKS_IN_ONE_DEGREE = 4096 / 360; // Update to reflect gear ratios

	/**
	 * Lowest possible position for the shooter mount to be at in degrees
	 */
	public static final double SHOOTER_MOUNT_OFFSET_DEGREES = 10;
	/**
	 * Lowest position the shooter mount target position can be set to in degrees
	 */
	public static final double SHOOTER_MOUNT_MIN_ANGLE = 0;

	/**
	 * Speaker height plus note height minus shooter mount height (meters)
	 */
	public static final double SHOOTER_MOUNT_TO_SPEAKER = 1.98 + 0.05 - 0.29;

	/**
	 * Angle in degrees for shooting at amp
	 */
	public static final double SHOOTER_MOUNT_AMP_ANGLE = 10;
	/**
	 * Angle in degrees for shooting at speaker
	 */
	public static final double SHOOTER_MOUNT_SPEAKER_ANGLE_FIXED = 10;

	public static final double SHOOTER_MOUNT_KP = 0.1;
	public static final double SHOOTER_MOUNT_KI = 0;
	public static final double SHOOTER_MOUNT_KD = 0.1;
	public static final double SHOOTER_MOUNT_KF = 0.1;
	/**
	 * Encoder ticks per 100 ms
	 */
	public static final double SHOOTER_MOUNT_CRUISE_VELOCITY = 2000;
	/**
	 * Encoder ticks per 100 ms per second
	 */
	public static final double SHOOTER_MOUNT_ACCELERATION = 1000;

	/**
	 * Tolerance for determining mount is within range to shoot in encoder ticks
	 */
	public static final double AIMING_DEADBAND = 100;

	/** Distance to speaker in meters */
	public static final double[] GRAVITY_COMPENSATION_TREE_MAP_KEYS = {1, 2, 3};
	/** Rotation compensation in degrees */
	public static final double[] GRAVITY_COMPENSATION_TREE_MAP_VALUES = {1, 2, 3};

}
