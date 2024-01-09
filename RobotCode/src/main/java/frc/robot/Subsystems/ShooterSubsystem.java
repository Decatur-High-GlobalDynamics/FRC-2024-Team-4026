package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamSparkMAX;

public class ShooterSubsystem extends SubsystemBase
{
	public TeamSparkMAX shooterMotor, preMotor;

	private static double voltage = 12;

	public ShooterSubsystem()
	{

		shooterMotor = new TeamSparkMAX("Shooter Motor", 0);
		preMotor = new TeamSparkMAX("preMotor", 0);
		shooterMotor.enableVoltageCompensation(voltage);
		preMotor.enableVoltageCompensation(voltage);

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
