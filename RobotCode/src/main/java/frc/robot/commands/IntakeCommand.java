package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterMountConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class IntakeCommand extends Command
{
	private IntakeSubsystem intake;
	private IndexerSubsystem indexer;
	private ShooterMountSubsystem shooterMount;

	public IntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterMountSubsystem shooterMount)
	{
		this.intake = intake;
		this.indexer = indexer;
		this.shooterMount = shooterMount;

		addRequirements(intake, indexer, shooterMount);
	}

	@Override
	public void initialize()
	{
		intake.setDesiredRotation(IntakeConstants.INTAKE_DEPLOYED_ROTATION);
		intake.setDesiredVelocity(IntakeConstants.INTAKE_DEPLOYED_VELOCITY);
		indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_INTAKE_VELOCITY, "Intaking");
		shooterMount.setTargetRotation(ShooterMountConstants.SHOOTER_MOUNT_MIN_ANGLE);
	}

	@Override
	public void end(boolean stop)
	{
		intake.setDesiredRotation(IntakeConstants.INTAKE_RETRACTED_ROTATION);
		intake.setDesiredVelocity(IntakeConstants.INTAKE_REST_VELOCITY);
		indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REST_VELOCITY, "Intaking done");
	}

	@Override
	public boolean isFinished()
	{
		return indexer.hasNote();
	}
}
