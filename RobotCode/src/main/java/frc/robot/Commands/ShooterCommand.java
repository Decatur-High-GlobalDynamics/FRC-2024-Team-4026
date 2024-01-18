package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase
{

	private ShooterSubsystem shooter;

	public ShooterCommand(ShooterSubsystem shoot)
	{

		shooter = shoot;
		addRequirements(shooter);
	}

	public void execute()
	{
		shooter.setShooterMotorPower(1.0, "joystick said to shoot");

		if (shooter.getShooterMotorPower() >= 0.95)
		{
			shooter.setFeedMotorPower(1.0, "motor is spun");
		}
		else
		{
			shooter.setFeedMotorPower(0, "motor is not spun");
		}
	}

	public void end(boolean interrupted)
	{
		shooter.setShooterMotorPower(0.25, "command is over");
		shooter.setFeedMotorPower(0, "Command is over");
	}
}
