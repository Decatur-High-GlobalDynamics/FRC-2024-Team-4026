package frc.lib.modules.shooter.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.shooter.ShooterSubsystem;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.lib.modules.indexer.IndexerConstants;
import frc.lib.modules.shooter.ShooterConstants;
import frc.lib.modules.shootermount.ShooterMountSubsystem;

/**
 * Points shooter mount at speaker Returns shooter mount to rest position when done
 */
public class AimShooterCommand extends Command
{

	private ShooterSubsystem shooter;
	private ShooterMountSubsystem shooterMount;
	private SwerveDriveSubsystem swerve;

	public AimShooterCommand(ShooterSubsystem shooter, ShooterMountSubsystem shooterMount,
			SwerveDriveSubsystem swerve)
	{
		this.shooter = shooter;
		this.shooterMount = shooterMount;
		this.swerve = swerve;

		addRequirements(shooter, shooterMount);
	}

	@Override
	public void initialize()
	{
		shooter.setShooterMotorVelocity(IndexerConstants.INDEXER_SHOOT_VELOCITY);
	}

	@Override
	public void execute()
	{
		Pose2d robotPose = swerve.getPose();
		Translation2d velocityAdjustedSpeakerPose = swerve
				.getSpeakerPoseAdjustedForVelocity(shooterMount);

		// Calculate ground distance adjusting for velocity
		double velocityAdjustedGroundDistance = robotPose.getTranslation()
				.getDistance(velocityAdjustedSpeakerPose);

		// Calculate the target rotation of the shooter mount in encoder ticks
		double targetRotation = shooterMount.getShooterMountAngleTreeMap()
				.get(velocityAdjustedGroundDistance);

		shooterMount.setTargetRotation(targetRotation);
	}

	@Override
	public void end(boolean interrupted)
	{
		shooter.setShooterMotorVelocity(ShooterConstants.SHOOTER_REST_VELOCITY);
		shooterMount.setTargetRotation(0);
	}

}
