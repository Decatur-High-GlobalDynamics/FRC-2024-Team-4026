package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamSparkMAX;
import frc.robot.constants.Ports;

public class ShooterMountSubsystem extends SubsystemBase
{

	private TeamSparkMAX mainMotor, followMotor;

	/** In degrees */
	private double goalRotation, distance;

	private static final double DEGREES_IN_ONE_TICK = 360 / 42, SPEED = 1;
	public static final double DEADBAND = 0.5;

	private boolean autoAim;

	public ShooterMountSubsystem()
	{
		mainMotor = new TeamSparkMAX("SHOOTER_MOUNT_MOTOR_LEFT", Ports.SHOOTER_MOUNT_MOTOR_LEFT);
		followMotor = new TeamSparkMAX("SHOOTER_MOUNT_MOTOR_RIGHT",
				Ports.SHOOTER_MOUNT_MOTOR_RIGHT);

		followMotor.follow(mainMotor);
		followMotor.setInverted(true);

		// This is the # of ticks in a rotation and the relative position
		mainMotor.getEncoder().setPositionConversionFactor(42);
		mainMotor.getEncoder().setPosition(0);
		mainMotor.set(0);

		goalRotation = 0.0;

		autoAim = false;
	}

	@Override
	public void periodic()
	{
		double difference = goalRotation - getCurrentRotation();

		if (Math.abs(difference) < DEADBAND)
		{
			setMotors(0, "Shooter Mount (Deadbanded): Difference: " + difference + ", Distance: "
					+ distance);
			return;
		}

		double power = distance * Math.sin(difference * Math.PI / distance);
		power = Math.max(-1, Math.min(power, 1));
		setMotors(power, "Shooter Mount: Difference: " + difference + ", Distance: " + distance);
	}

	public void setGoalRotation(double degrees)
	{
		distance = degrees - getCurrentRotation();
		goalRotation = degrees;
	}

	public double getCurrentRotation()
	{
		return ticksToDegrees(mainMotor.getEncoder().getPosition());
	}

	private static double degreesToTicks(double degrees)
	{
		return degrees / DEGREES_IN_ONE_TICK;
	}

	public static double ticksToDegrees(double ticks)
	{
		return ticks * DEGREES_IN_ONE_TICK;
	}

	public void setMotors(double power, String reason)
	{
		mainMotor.set(Math.max(-1, Math.min(power, 1)) * SPEED, reason);
	}

	public void setAutoAim(boolean autoAim) {
		this.autoAim = autoAim;
	}

}