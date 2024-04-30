package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberToPositionCommand extends InstantCommand
{

	private double leftTargetPosition, rightTargetPosition;
	private ClimberSubsystem climber;
	private TalonFX climberMotorLeft, climberMotorRight;
	
	public ClimberToPositionCommand(ClimberSubsystem climber, double leftTargetPosition, double rightTargetPosition)
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
		
		climberMotorLeft.setPosition(ClimberConstants.LEFT_CLIMBER_MAXIMUM, leftTargetPosition);
		climberMotorRight.setPosition(ClimberConstants.RIGHT_CLIMBER_MAXIMUM, rightTargetPosition);
		
		climberMotorLeft.setPosition(ClimberConstants.LEFT_CLIMBER_MAXIMUM, leftTargetPosition);
		climberMotorRight.setPosition(ClimberConstants.RIGHT_CLIMBER_MINIMUM, rightTargetPosition);
	}
}
