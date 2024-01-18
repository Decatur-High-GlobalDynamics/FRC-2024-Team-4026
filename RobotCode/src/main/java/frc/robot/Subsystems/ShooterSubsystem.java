package frc.robot.Subsystems;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamSparkMAX;

public class ShooterSubsystem extends SubsystemBase
{
	private double shooterMotorPower;
	private double feederMotorPower;
	
	private PIDController shooterPID;
	private PIDController feederPID;
	public TeamSparkMAX shooterMotorMain, shooterMotorSub, feederMotorMain,
			feederMotorSub;

	private static double voltage = 12;

	public ShooterSubsystem()
	{
		shooterMotorPower = 0.25;
		feederMotorPower =  0;
		
		shooterPID = new PIDController(Constants.SHOOTER_KP, Constants.SHOOTER_KI,
				Constants.SHOOTER_KD);
		feederPID = new PIDController(Constants.FEEDER_KP, Constants.FEEDER_KI,
				Constants.FEEDER_KD);

		shooterMotorMain = new TeamSparkMAX("Left Shooter Motor Main",
				Ports.LEFT_SHOOTER_MOTOR_MAIN);
		shooterMotorSub = new TeamSparkMAX("Right Shooter Motor Main",
				Ports.RIGHT_SHOOTER_MOTOR_MAIN);
		feederMotorMain = new TeamSparkMAX("Left Shooter Motor Sub",
				Ports.LEFT_SHOOTER_MOTOR_SUB);
		feederMotorSub = new TeamSparkMAX("Right Shooter Motor Sub",
				Ports.RIGHT_SHOOTER_MOTOR_SUB);

		shooterMotorMain.enableVoltageCompensation(voltage);
		shooterMotorSub.enableVoltageCompensation(voltage);
		feederMotorMain.enableVoltageCompensation(voltage);
		feederMotorSub.enableVoltageCompensation(voltage);

		feederMotorSub.follow(feederMotorMain);
		shooterMotorSub.follow(shooterMotorMain);

		shooterMotorSub.setInverted(true);
		feederMotorSub.setInverted(true);

		shooterMotorSub.setIdleMode(IdleMode.kBrake);
		feederMotorSub.setIdleMode(IdleMode.kBrake);
		shooterMotorMain.setIdleMode(IdleMode.kBrake);
		feederMotorMain.setIdleMode(IdleMode.kBrake);
	}

	public double getShooterMotorPower()
	{
		return shooterMotorMain.get();
	}

	public void setShooterMotorPower(double power, String reason)
	{
		shooterMotorPower = Math.max(Math.min(1, power), -1);

	}

	public void setFeedMotorPower(double power, String reason)
	{
		feederMotorPower = Math.max(Math.min(1, power), -1);
	}
	public void periodic()
	{
		double newShooterPower = shooterPID.calculate(shooterMotorSub.get(), shooterMotorPower);
		shooterMotorMain.set(newShooterPower);

		double newFeederPower = shooterPID.calculate(feederMotorSub.get(), feederMotorPower);
		feederMotorMain.set(newFeederPower);
	}
}
