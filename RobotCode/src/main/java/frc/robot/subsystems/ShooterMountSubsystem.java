package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamSparkMAX;
import frc.robot.constants.Ports;
import frc.robot.constants.ShooterMountConstants;

public class ShooterMountSubsystem extends SubsystemBase
{

	private TeamSparkMAX mainMotor, followMotor;

	/** In degrees */
	private double goalRotation;

	private ProfiledPIDController pidController;

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

		pidController = new ProfiledPIDController(ShooterMountConstants.KP,
				ShooterMountConstants.KI, ShooterMountConstants.KD,
				new TrapezoidProfile.Constraints(ShooterMountConstants.K_MAX_VELOCITy,
						ShooterMountConstants.K_MAX_ACCELERATION));
	}

	@Override
	public void periodic()
	{
		mainMotor.set(pidController.calculate(mainMotor.getCurrentEncoderValue(),
				degreesToTicks(goalRotation)));
	}

	public void setGoalRotation(double degrees)
	{
		goalRotation = degrees;
	}

	public double getCurrentRotation()
	{
		return ticksToDegrees(mainMotor.getEncoder().getPosition());
	}

	private static double degreesToTicks(double degrees)
	{
		return degrees / ShooterMountConstants.DEGREES_IN_ONE_TICK;
	}

	public static double ticksToDegrees(double ticks)
	{
		return ticks * ShooterMountConstants.DEGREES_IN_ONE_TICK;
	}

}