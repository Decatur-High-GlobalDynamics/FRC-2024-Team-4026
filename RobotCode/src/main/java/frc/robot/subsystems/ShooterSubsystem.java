package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import frc.robot.constants.ShooterConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamSparkMAX;

public class ShooterSubsystem extends SubsystemBase
{
	// Creates objects
	private double shooterMotorPower, feederMotorPower;

	private final PIDController shooterPid, feederPid;
	private final TeamSparkMAX ShooterMotorMain, ShooterMotorSub, FeederMotorMain, FeederMotorSub;

	private static final double VOLTAGE = 12;

	public ShooterSubsystem()
	{
		// Sets default shooter motor power to 0.25 and feeder to 0
		shooterMotorPower = 0.25;
		feederMotorPower = 0;

		// Initializes the Pid
		shooterPid = new PIDController(ShooterConstants.SHOOTER_KP, ShooterConstants.SHOOTER_KI,
				ShooterConstants.SHOOTER_KD);
		feederPid = new PIDController(ShooterConstants.FEEDER_KP, ShooterConstants.FEEDER_KI,
				ShooterConstants.FEEDER_KD);

		// Initializes motor object
		ShooterMotorMain = new TeamSparkMAX("Left Shooter Motor Main", Ports.SHOOTER_MOTOR_MAIN);
		ShooterMotorSub = new TeamSparkMAX("Right Shooter Motor Main", Ports.SHOOTER_MOTOR_SUB);

		FeederMotorMain = new TeamSparkMAX("Left Shooter Motor Sub", Ports.FEEDER_MOTOR_MAIN);
		FeederMotorSub = new TeamSparkMAX("Right Shooter Motor Sub", Ports.FEEDER_MOTOR_SUB);

		// Inverts the right side
		ShooterMotorSub.setInverted(true);
		FeederMotorSub.setInverted(true);

		// Sets neutral mode for the motors
		ShooterMotorSub.setIdleMode(IdleMode.kBrake);
		FeederMotorSub.setIdleMode(IdleMode.kBrake);

		ShooterMotorMain.setIdleMode(IdleMode.kBrake);
		FeederMotorMain.setIdleMode(IdleMode.kBrake);
	}

	public double getShooterMotorPower()
	{
		return ShooterMotorMain.get();
	}

	/**
	 * This is clamping the shooter motor power to be within the range of -1 to 1
	 */
	public void setShooterMotorPower(double power, String reason)
	{
		shooterMotorPower = Math.max(Math.min(1, power), -1);

	}

	/**
	 * This is clamping the feeder motor power to be within the range of -1 to 1
	 */
	public void setFeedMotorPower(double power, String reason)
	{
		feederMotorPower = Math.max(Math.min(1, power), -1);
	}

	/**
	 * Continuously updates shooter speed based on the commands above.
	 */
	@Override
	public void periodic()
	{
		double newShooterPower = shooterPid.calculate(ShooterMotorSub.get(), shooterMotorPower);
		ShooterMotorMain.set(newShooterPower);
		ShooterMotorSub.set(newShooterPower * ShooterConstants.MOTOR_SPEED_MOD);

		double newFeederPower = feederPid.calculate(FeederMotorSub.get(), feederMotorPower);
		FeederMotorMain.set(newFeederPower);
		FeederMotorSub.set(newFeederPower * ShooterConstants.MOTOR_SPEED_MOD);
	}
}
