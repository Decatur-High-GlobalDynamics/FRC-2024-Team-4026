package frc.lib.modules.elevator.commands.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.modules.elevator.ElevatorConstants;
import frc.lib.modules.elevator.ElevatorSubsystem;

public class ElevatorToPositionCommand extends InstantCommand
{

	private double leftTargetPosition, rightTargetPosition;
	private ElevatorSubsystem climber;
	private TalonFX climberMotorLeft, climberMotorRight;
	
	public ElevatorToPositionCommand(ElevatorSubsystem climber, double leftTargetPosition, double rightTargetPosition)
	{
		this.climber = climber;
		this.leftTargetPosition = leftTargetPosition;
		this.rightTargetPosition = rightTargetPosition;
		addRequirements(climber);

	}


	public void initialize()
	{
		climberMotorLeft.setPosition(leftTargetPosition);
		climberMotorRight.setPosition(rightTargetPosition);
		
		climberMotorLeft.setPosition(ElevatorConstants.LEFT_CLIMBER_MAXIMUM, leftTargetPosition);
		climberMotorRight.setPosition(ElevatorConstants.RIGHT_CLIMBER_MAXIMUM, rightTargetPosition);
		
		climberMotorLeft.setPosition(ElevatorConstants.LEFT_CLIMBER_MAXIMUM, leftTargetPosition);
		climberMotorRight.setPosition(ElevatorConstants.RIGHT_CLIMBER_MINIMUM, rightTargetPosition);
	}
}
