package frc.lib.modules.shooter;

/**
 * 
 */
public class ShooterConstants
{

	public static final double SHOOTER_KP = 0.02;
	public static final double SHOOTER_KI = 0;
	public static final double SHOOTER_KD = 0.0;
	public static final double SHOOTER_KS = 0;
	public static final double SHOOTER_KV = 0.01;
	public static final double SHOOTER_KA = 0;
	//cruise velocity is velocity when robot isn't shooting
	public static final double SHOOTER_CRUISE_VELOCITY = 100;
	public static final double SHOOTER_ACCELERATION = 200;

	public static final double SHOOTER_VELOCITY_TOLERANCE = 5;

	// Velocity in RPM
	/**
	 * Velocity for shooting at speaker in RPS
	 */
	public static final double SHOOTER_SPEAKER_VELOCITY = -100;
	/**
	 * Velocity for shooting at amp in RPS
	 */
	public static final double SHOOTER_AMP_VELOCITY = -10.5;
	/**
	 * Velocity when not in use in RPS
	 */
	public static final double SHOOTER_REST_VELOCITY = 0;
	//yes, you can pass in frc, this year it was a winning strategy
	public static final double SHOOTER_PASSING_VELOCITY = -60; //-58;

	public static final double SHOOTER_REVERSE_VELOCITY = 20;

	/** Number of milliseconds to wait when shooting in autonomous */
	public static final long SHOOT_TIME = 800;

}
