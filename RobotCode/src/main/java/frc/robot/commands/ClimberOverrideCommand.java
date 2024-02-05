package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberOverrideCommand extends Command
{
	private ClimberSubsystem climber;

	public ClimberOverrideCommand(ClimberSubsystem climber)
	{
		this.climber = climber;
		addRequirements(climber);

	}

	//Enables override for the duration of the command
	public void instantiate()
	{
		climber.setOverride(true);
	}

	public void end()
	{
		climber.setOverride(false);
	}
}
