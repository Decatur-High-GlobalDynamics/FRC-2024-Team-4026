package frc.robot.constants;

/**
 * 
 */
public class ShooterConstants
{

	public static final double SHOOTER_KP = 0;
	public static final double SHOOTER_KI = 0;
	public static final double SHOOTER_KD = 0;
	public static final double SHOOTER_KS = 0;
	public static final double SHOOTER_KV = 0.0105;
	public static final double SHOOTER_KA = 0;

	public static final double SHOOTER_CRUISE_VELOCITY = 96;
	public static final double SHOOTER_ACCELERATION = 200;

	public static final double SHOOTER_VELOCITY_TOLERANCE = 20;

	// Velocity in RPM
	/**
	 * Velocity for shooting at speaker in RPS
	 */
	public static final double SHOOTER_SPEAKER_VELOCITY = -96;
	/**
	 * Velocity for shooting at amp in RPS
	 */
	public static final double SHOOTER_AMP_VELOCITY = -20;
	/**
	 * Velocity when not in use in RPS
	 */
	public static final double SHOOTER_REST_VELOCITY = 0;

	public static final double SHOOTER_PASSING_VELOCITY = -25;

	public static final double SHOOTER_REVERSE_VELOCITY = 20;

	/** Number of milliseconds to wait when shooting in autonomous */
	public static final long SHOOT_TIME = 800;

}
