package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShooterFeederCommand extends CommandBase
{

	private ShooterSubsystem shooter;

	public ShooterFeederCommand(ShooterSubsystem shoot)
	{

		shooter = shoot;
		addRequirements(shooter);
	}

	public void execute()
	{
		shooter.setPreMotorPower(1, "Feed the ring");
	}
}
