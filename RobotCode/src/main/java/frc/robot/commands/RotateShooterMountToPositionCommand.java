package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterMountConstants;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.lib.core.ILogSource;

public class RotateShooterMountToPositionCommand extends Command implements ILogSource
{

	private ShooterMountSubsystem shooterMount;
	private double targetPosition;

	/**
	 * This constructor is the version that uses a constant target
	 * 
	 * @param position in degrees
	 */
	public RotateShooterMountToPositionCommand(ShooterMountSubsystem shooterMount,
			double targetPosition)
	{
		this.shooterMount = shooterMount;
		this.targetPosition = targetPosition;

		addRequirements(shooterMount);
	}

	@Override
	public void initialize()
	{
		logFine("Shooter Mount Rotating...");
		shooterMount.setTargetRotation(targetPosition);
	}

	public void end()
	{
		logFine("Shooter Mount Stopped");
		shooterMount.setTargetRotation(ShooterMountConstants.SHOOTER_MOUNT_MIN_ANGLE);
	}
}
