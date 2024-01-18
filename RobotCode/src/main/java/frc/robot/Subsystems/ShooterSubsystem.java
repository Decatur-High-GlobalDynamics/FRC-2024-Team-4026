package frc.robot.Subsystems;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.constants.Ports;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamSparkMAX;

public class ShooterSubsystem extends SubsystemBase
{
	public TeamSparkMAX leftShooterMotorMain, rightShooterMotorMain, leftShooterMotorSub,
			rightShooterMotorSub;

	private static double voltage = 12;

	public ShooterSubsystem()
	{

		leftShooterMotorMain = new TeamSparkMAX("Left Shooter Motor Main",
				Ports.LEFT_SHOOTER_MOTOR_MAIN);
		rightShooterMotorMain = new TeamSparkMAX("Right Shooter Motor Main",
				Ports.RIGHT_SHOOTER_MOTOR_MAIN);
		leftShooterMotorSub = new TeamSparkMAX("Left Shooter Motor Sub",
				Ports.LEFT_SHOOTER_MOTOR_SUB);
		rightShooterMotorSub = new TeamSparkMAX("Right Shooter Motor Sub",
				Ports.RIGHT_SHOOTER_MOTOR_SUB);

		leftShooterMotorMain.enableVoltageCompensation(voltage);
		rightShooterMotorMain.enableVoltageCompensation(voltage);
		leftShooterMotorSub.enableVoltageCompensation(voltage);
		rightShooterMotorSub.enableVoltageCompensation(voltage);

		leftShooterMotorSub.follow(rightShooterMotorSub);
		leftShooterMotorMain.follow(rightShooterMotorMain);

		rightShooterMotorMain.setInverted(true);
		rightShooterMotorSub.setInverted(true);

		rightShooterMotorMain.setIdleMode(IdleMode.kBrake);
		rightShooterMotorSub.setIdleMode(IdleMode.kBrake);
		leftShooterMotorMain.setIdleMode(IdleMode.kBrake);
		leftShooterMotorSub.setIdleMode(IdleMode.kBrake);
	}

	public double getShooterMotorPower()
	{
		return rightShooterMotorMain.get();
	}

	public void setShooterMotorPower(double power, String reason)
	{
		rightShooterMotorMain.set(power, reason);

	}

	public void setFeedMotorPower(double power, String reason)
	{
		rightShooterMotorSub.set(power, reason);
	}

}
