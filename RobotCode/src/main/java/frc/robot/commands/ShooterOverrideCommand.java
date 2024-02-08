package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterOverrideCommand extends Command
{
	// Initializes the subsystem objects
	private ShooterSubsystem shooter;
	private IndexerSubsystem indexer;

	private double desiredShooterVelocity;

	public ShooterOverrideCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, double desiredShooterVelocity)
	{
		this.desiredShooterVelocity = desiredShooterVelocity;

		this.shooter = shooter;
		this.indexer = indexer;
		addRequirements(shooter, indexer);
	}

	@Override
	public void execute()
	{
		// Spins up the motor
		shooter.setShooterMotorVelocity(desiredShooterVelocity, "joystick said to shoot");

		// If-statement to see if motor is spun up
		if (Math.abs(shooter.getShooterMotorVelocityError()) <= ShooterConstants.SHOOTER_VELOCITY_TOLERANCE)
		{
			indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_SHOOT_VELOCITY, "motor is spun");
		}
		else
		{
			indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REST_VELOCITY, "motor is not spun");
		}
	}

	@Override
	public void end(boolean interrupted)
	{
		shooter.setShooterMotorVelocity(ShooterConstants.SHOOTER_REST_VELOCITY, "command is over");
		indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REST_VELOCITY, "command is over");
	}
}
