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
	}

	public void end(boolean interrupted)
	{
		shooter.setShooterMotorPower(0, "command is over");
	}
}
