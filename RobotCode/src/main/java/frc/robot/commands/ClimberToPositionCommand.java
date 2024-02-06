package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberToPositionCommand extends Command
{
	/* targetPosition refers to where we want the climber to end.
	This is a surprise tool that will help us later. */
	private double targetPosition;
	private ClimberSubsystem climber;

	public ClimberToPositionCommand(ClimberSubsystem climber, double targetPosition)
	{
		this.climber = climber;
		this.targetPosition = targetPosition;
		addRequirements(climber);

	}

	public void instantiate()
	{
		climber.setPosition(targetPosition);
	}
}
