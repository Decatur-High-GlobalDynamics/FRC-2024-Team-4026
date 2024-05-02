package frc.lib.modules.elevator.commands.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.elevator.ElevatorConstants;
import frc.lib.modules.elevator.ElevatorSubsystem;

public class ElevatorSpeedCommand extends Command
{
	ElevatorSubsystem climber;
	DoubleSupplier leftInput;
	DoubleSupplier rightInput;

	public ElevatorSpeedCommand(ElevatorSubsystem climber, DoubleSupplier leftInput,
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
		if (Math.abs(leftInputFinal) < ElevatorConstants.DEADBAND_JOYSTICK)
		{
			leftInputFinal = 0;
		}
		if (Math.abs(rightInputFinal) < ElevatorConstants.DEADBAND_JOYSTICK)
		{
			rightInputFinal = 0;
		}
		climber.setLeftVelocity(leftInputFinal);
		climber.setRightVelocity(rightInputFinal);
	}
}
