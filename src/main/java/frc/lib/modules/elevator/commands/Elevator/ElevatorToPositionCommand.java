package frc.lib.modules.elevator.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.modules.elevator.ElevatorSubsystem;

public class ElevatorToPositionCommand extends InstantCommand
{

	private double leftTargetPosition, rightTargetPosition;
	private ElevatorSubsystem climber;
	
	public ElevatorToPositionCommand(ElevatorSubsystem climber, double leftTargetPosition, double rightTargetPosition)
	{
		this.climber = climber;
		this.leftTargetPosition = leftTargetPosition;
		this.rightTargetPosition = rightTargetPosition;
		addRequirements(climber);

	}


	public void initialize()
	{
		climber.setRightTargetPosition(rightTargetPosition);
		climber.setLeftTargetPosition(leftTargetPosition);
	}
}
