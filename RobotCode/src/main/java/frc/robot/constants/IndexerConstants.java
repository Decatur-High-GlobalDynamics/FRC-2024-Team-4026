package frc.robot.constants;

public class IndexerConstants
{

    public static final double INDEXER_KP = 0.001;
    public static final double INDEXER_KI = 0;
    public static final double INDEXER_KD = 0;
    public static final double INDEXER_KF = 0.01;

    // Velocities in RPM
	/**
	 * Maximum safe velocity in RPM
	 */
    public static final double INDEXER_MAX_VELOCITY = 10;
	/**
	 * Velocity for ejecting in RPM
	 */
    public static final double INDEXER_SHOOT_VELOCITY = 10;
	/**
	 * Velocity for intaking in RPM
	 */
    public static final double INDEXER_INTAKE_VELOCITY = 10;
	/**
	 * Velocity when not in use in RPM
	 */
    public static final double INDEXER_REST_VELOCITY = 0;

}
