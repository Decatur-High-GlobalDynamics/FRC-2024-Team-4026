package frc.robot.constants;

/**
 * 
 */
public class ShooterConstants
{

	public static final double SHOOTER_KP = 0.001;
	public static final double SHOOTER_KI = 0;
	public static final double SHOOTER_KD = 0;
	public static final double SHOOTER_KF = 0.01;

	public static final double SHOOTER_VELOCITY_TOLERANCE = 10;

	// Velocity in RPM
	/**
	 * Maximum safe velocity in RPM
	 */
	public static final double SHOOTER_MAX_VELOCITY = 10;
	/**
	 * Velocity for shooting at speaker in RPM
	 */
	public static final double SHOOTER_SPEAKER_VELOCITY = 10;
	/**
	 * Velocity for shooting at amp in RPM
	 */
	public static final double SHOOTER_AMP_VELOCITY = 5;
	/**
	 * Velocity when not in use in RPM
	 */
	public static final double SHOOTER_REST_VELOCITY = 2.5;

}
