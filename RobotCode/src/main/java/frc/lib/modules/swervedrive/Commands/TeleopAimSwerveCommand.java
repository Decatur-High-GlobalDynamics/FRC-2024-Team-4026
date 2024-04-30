package frc.lib.modules.swervedrive.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.lib.modules.shootermount.ShooterMountSubsystem;

/**
 * Rotates the chassis towards the speaker but allows the driver to control translation and strafe
 */
public class TeleopAimSwerveCommand extends TeleopSwerveCommand
{

	private final SwerveDriveSubsystem Swerve;
	private final ShooterMountSubsystem ShooterMount;
	private final IndexerSubsystem Indexer;

	public TeleopAimSwerveCommand(SwerveDriveSubsystem swerve, ShooterMountSubsystem shooterMount,
			IndexerSubsystem indexer, DoubleSupplier translationSup, DoubleSupplier strafeSup,
			BooleanSupplier slowSpeedSupplier)
	{
		super(swerve, translationSup, strafeSup,
				() -> swerve.getRotationalVelocityToSpeaker(shooterMount), slowSpeedSupplier);

		Swerve = swerve;
		ShooterMount = shooterMount;
		Indexer = indexer;

		addRequirements(Indexer);
	}

	@Override
	public void execute()
	{
		super.execute();

		// Spin feeder motors if in target
		if (Swerve.isInShooterRange()
				&& Math.abs(Swerve
						.getRotationToSpeaker(ShooterMount)) < VisionConstants.CHASSIS_AIM_THRESHOLD
				&& ShooterMount.isAtTargetRotation())
		{
			// Spin feeder motors
			Indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_SHOOT_VELOCITY);
		}
	}

	@Override
	public void end(boolean interrupted)
	{
		Swerve.setRotationController(null);
		Indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REST_VELOCITY);
	}

}
