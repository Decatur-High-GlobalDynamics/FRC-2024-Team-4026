package frc.robot.subsystems;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import frc.robot.constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamSparkMAX;

public class ShooterSubsystem extends SubsystemBase
{
	// Creates objects
	private double desiredShooterVelocity;

	private SparkPIDController shooterPid;
	private TeamSparkMAX shooterMotorMain, shooterMotorSub;

	public ShooterSubsystem()
	{
		// Sets default shooter motor power
		desiredShooterVelocity = ShooterConstants.SHOOTER_REST_VELOCITY;

		// Initializes motor object
		shooterMotorMain = new TeamSparkMAX("Left Shooter Motor Main", Ports.SHOOTER_MOTOR_MAIN);
		shooterMotorSub = new TeamSparkMAX("Right Shooter Motor Main", Ports.SHOOTER_MOTOR_SUB);

		shooterMotorSub.follow(shooterMotorMain, true);
		shooterMotorMain.setIdleMode(IdleMode.kBrake);
		shooterMotorSub.setIdleMode(IdleMode.kBrake);
		shooterMotorMain.setSmartCurrentLimit(Constants.MAX_CURRENT);
		shooterMotorSub.setSmartCurrentLimit(Constants.MAX_CURRENT);

		shooterPid = shooterMotorMain.getPidController();

		shooterPid.setP(ShooterConstants.SHOOTER_KP);
		shooterPid.setI(ShooterConstants.SHOOTER_KI);
		shooterPid.setD(ShooterConstants.SHOOTER_KD);
		shooterPid.setFF(ShooterConstants.SHOOTER_KF);
	}

	public double getShooterMotorVelocityError() 
	{
		return shooterMotorMain.getVelocityError();
	}

	/**
	 * This is clamping the shooter motor power
	 */
	public void setShooterMotorVelocity(double desiredShooterVelocity, String reason)
	{
		this.desiredShooterVelocity = Math.max(
				Math.min(ShooterConstants.SHOOTER_MAX_VELOCITY, desiredShooterVelocity),
				-ShooterConstants.SHOOTER_MAX_VELOCITY);
	}

	/**
	 * Continuously updates shooter speed based on the commands above.
	 */
	@Override
	public void periodic()
	{
		shooterPid.setReference(desiredShooterVelocity, ControlType.kVelocity);
	}
}
