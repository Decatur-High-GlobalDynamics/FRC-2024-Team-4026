package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberToPositionCommand extends InstantCommand
{

	private double leftTargetPosition, rightTargetPosition;
	private ClimberSubsystem climber;
	
	public ClimberToPositionCommand(ClimberSubsystem climber, double leftTargetPosition, double rightTargetPosition)
	{
		this.climber = climber;
		this.leftTargetPosition = leftTargetPosition;
		this.rightTargetPosition = rightTargetPosition;
		climberMotorLeft = climber.climberMotorLeft;
		climberMotorRight = climber.climberMotorRight;
		addRequirements(climber);

	}

	@Override
	public void initialize()
	{
		climber.setPosition(leftTargetPosition, rightTargetPosition);
		
		climberMotorLeft.setMinumumPosition(LEFT_CLIMBER_MAXIMUM, leftTargetPosition);
		climberMotorRight.setMinumumPosition(RIGHT_CLIMBER_MAXIMUM, rightTargetPosition);
		
		climberMotorLeft.setMaximumPosition(LEFT_CLIMBER_MINIMUM, leftTargetPosition);
		climberMotorRight.setMaximumPosition(RIGHT_CLIMBER_MINIMUM, rightTargetPosition);
	}
}
