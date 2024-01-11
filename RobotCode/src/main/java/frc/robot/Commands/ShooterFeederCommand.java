package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShooterFeederCommand extends CommandBase
{

	private ShooterSubsystem feeder;

	public ShooterFeederCommand(ShooterSubsystem feed)
	{

		feeder = feed;
		addRequirements(feeder);
	}

	public void execute()
	{
		feeder.setPreMotorPower(1, "Feed the ring");
	}

	public void end(boolean interrupted)
	{
		feeder.setPreMotorPower(0, "command finished");
	}
}
