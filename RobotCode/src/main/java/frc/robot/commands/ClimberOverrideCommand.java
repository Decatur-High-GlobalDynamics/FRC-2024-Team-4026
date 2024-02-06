package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.lib.core.ILogSource;

public class ClimberOverrideCommand extends Command implements ILogSource
{
	private ClimberSubsystem climber;

	public ClimberOverrideCommand(ClimberSubsystem climber)
	{
		this.climber = climber;
		addRequirements(climber);

	}

	// forgot what this does but renato said its important
	public void instantiate()
	{
		climber.setOverride(true);
		logFine("Climber in manual control");
	}

	public void end()
	{
		climber.setOverride(false);
		logFine("Climber is back to normal");
	}
}
