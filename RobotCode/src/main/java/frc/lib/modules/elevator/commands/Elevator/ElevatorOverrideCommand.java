package frc.lib.modules.elevator.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.elevator.ElevatorSubsystem;

public class ElevatorOverrideCommand extends Command {
	private ElevatorSubsystem climber;

	public ElevatorOverrideCommand(ElevatorSubsystem climber) {
		this.climber = climber;
		addRequirements(climber);
	}

	@Override
	public void initialize() {
		climber.setOverride(true);
	}

	@Override
	public void end(boolean interrupted) {
		climber.setOverride(false);
	}
}
