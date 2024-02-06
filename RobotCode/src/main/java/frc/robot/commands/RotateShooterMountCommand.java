package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.lib.core.ILogSource;

/** Continuously rotates based on a speed */
public class RotateShooterMountCommand extends Command implements ILogSource
{

	/** In degrees per {@link #execute()} call (20 ms) */
	private DoubleSupplier getSpeed;

	private ShooterMountSubsystem shooterMountSubsystem;

	/**
	 * This constructor is the version that allows for variable speeds (ex: based on joystick input)
	 * 
	 * @param speed in degrees per second
	 */
	public RotateShooterMountCommand(ShooterMountSubsystem subsystem, DoubleSupplier getSpeed)
	{
		shooterMountSubsystem = subsystem;
		this.getSpeed = getSpeed;

		addRequirements(subsystem);
	}

	/**
	 * This constructor is version that uses a constant speed
	 * 
	 * @param speed in degrees per second
	 */
	public RotateShooterMountCommand(ShooterMountSubsystem subsystem, double speed)
	{
		this(subsystem, () -> speed / 50); // Convert from degrees per second to degrees per 20ms
	}
	@Override
	public void initialize()
	{
		logFine("Shooter Mount Rotating...");
	}
	public void execute()
	{
		double rotation = Math.max(0,
				Math.min(180, shooterMountSubsystem.getCurrentRotation() + getSpeed.getAsDouble()));
		shooterMountSubsystem.setGoalRotation(rotation);
	}

	@Override

	public void end(boolean interrupted)
	{
		logFine("Shooter Mount Stopped.");
	}

}