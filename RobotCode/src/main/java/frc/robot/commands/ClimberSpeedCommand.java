package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberSpeedCommand extends Command
{
	ClimberSubsystem climber;
	DoubleSupplier leftInput;
	DoubleSupplier rightInput;

	public ClimberSpeedCommand(ClimberSubsystem climber, DoubleSupplier leftInput,
			DoubleSupplier rightInput)
	{
		this.climber = climber;
		this.leftInput = leftInput;
		this.rightInput = rightInput;
		addRequirements(climber);
	}

	public void execute()
	{
		double leftInputFinal = leftInput.getAsDouble();
		double rightInputFinal = rightInput.getAsDouble();
		if (Math.abs(leftInputFinal) < ClimberConstants.DEADBAND_JOYSTICK)
		{
			leftInputFinal = 0;
		}
		if (Math.abs(rightInputFinal) < ClimberConstants.DEADBAND_JOYSTICK)
		{
			rightInputFinal = 0;
		}
		climber.setPowers(leftInputFinal, rightInputFinal, "Manual Movement Override");
	}
}
