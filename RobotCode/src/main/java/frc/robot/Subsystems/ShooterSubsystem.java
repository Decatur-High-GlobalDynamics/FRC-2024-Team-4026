package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.constants.Ports;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamSparkMAX;

public class ShooterSubsystem extends SubsystemBase
{
	public TeamSparkMAX shooterMotor, preMotor;

	private static double voltage = 12;

	public ShooterSubsystem()
	{

		shooterMotor = new TeamSparkMAX("Shooter Motor", Ports.SHOOTER_MOTOR);
		preMotor = new TeamSparkMAX("preMotor", Ports.PRE_MOTOR);
		shooterMotor.enableVoltageCompensation(voltage);
		preMotor.enableVoltageCompensation(voltage);

		shooterMotor.setIdleMode(IdleMode.kBrake);
		preMotor.setIdleMode(IdleMode.kBrake);

	}

	public void setShooterMotorPower(double power, String reason)
	{
		shooterMotor.set(power, reason);

	}

	public void setPreMotorPower(double power, String reason)
	{
		preMotor.set(power, reason);

	}

}
