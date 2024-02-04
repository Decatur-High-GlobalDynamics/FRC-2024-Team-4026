package frc.lib.modules.tankdrivetrain.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.tankdrivetrain.TankDrivetrainConstants;
import frc.lib.modules.tankdrivetrain.TankDrivetrainSubsystem;

// never
public class TankDriveCommand extends Command
{

	private TankDrivetrainSubsystem drivetrain;
	private DoubleSupplier leftInputY, rightInputY;

	public TankDriveCommand(DoubleSupplier leftInputY, DoubleSupplier rightInputY,
			TankDrivetrainSubsystem drivetrain)
	{
		this.leftInputY = leftInputY;
		this.rightInputY = rightInputY;
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);
	}

	private double calculateDeadZonedPower(double input)
	{
		// Dead zone power by setting it to 0 if it is below a threshold
		if (Math.abs(input) <= TankDrivetrainConstants.DEADZONE_AMOUNT)
		{
			return 0;
		}
		else
			return input;
	}

	@Override
	public void execute()
	{
		// Get the dead zoned powers
		double finalLeftInputY = calculateDeadZonedPower(leftInputY.getAsDouble());
		double finalRightInputY = calculateDeadZonedPower(rightInputY.getAsDouble());

		// Get the cubed powers
		finalLeftInputY = Math.pow(finalLeftInputY,
				TankDrivetrainConstants.DRIVETRAIN_POWER_EXPONENT);
		finalRightInputY = Math.pow(finalRightInputY,
				TankDrivetrainConstants.DRIVETRAIN_POWER_EXPONENT);

		// Set the motor powers in the drivetrain subsystem
		drivetrain.setMotorPowers(finalLeftInputY, finalRightInputY, "TankDriveCommand Joysticks");
	}

}
