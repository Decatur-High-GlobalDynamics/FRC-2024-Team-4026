package frc.lib.modules.elevator;

public final class ElevatorConstants
{

	public static final double CLIMBER_KP = 0;
	public static final double CLIMBER_KI = 0;
	public static final double CLIMBER_KD = 0;
	public static final double CLIMBER_KS = 1;
	public static final double CLIMBER_KV = 0;
	public static final double CLIMBER_KA = 0;
	public static final double CLIMBER_ACCELERATION = 120;
	public static final double CLIMBER_CRUISE_VELOCITY = 120;

	public static final double DEADBAND_JOYSTICK = 0.1;

	/** Deadband for autobalancing, in degrees */
	public static final double BALANCE_DEADBAND = 5;

	/** Minimum rotations of left climber */
	public static final double LEFT_CLIMBER_MINIMUM = -524.32;
	/** Minimum rotations of right climber */
	public static final double RIGHT_CLIMBER_MINIMUM = 873.01;

	/** Maximum rotations of left climber */
	public static final double LEFT_CLIMBER_MAXIMUM = -26.57;
	/** Maximum rotations of right climber */
	public static final double RIGHT_CLIMBER_MAXIMUM = 314.67;

}
