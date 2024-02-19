package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterMountConstants;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Points shooter mount at speaker Returns shooter mount to rest position when done
 */
public class AimShooterCommand extends Command
{

	private ShooterSubsystem shooter;
	private ShooterMountSubsystem shooterMount;
	private VisionSubsystem vision;
	private SwerveDriveSubsystem swerveDrive;

	public AimShooterCommand(ShooterSubsystem shooter, ShooterMountSubsystem shooterMount,
			VisionSubsystem vision, SwerveDriveSubsystem swerveDrive)
	{
		this.shooter = shooter;
		this.shooterMount = shooterMount;
		this.vision = vision;
		this.swerveDrive = swerveDrive;

		addRequirements(shooter, shooterMount, vision);

	}

	@Override
	public void initialize()
	{
		shooter.setShooterMotorVelocity(IndexerConstants.INDEXER_SHOOT_VELOCITY,
				"Auto aiming at speaker");
	}

	@Override
	public void execute()
	{
		Pose2d shooterMountPose = vision.getShooterMountPose2d().orElse(new Pose2d());
		Translation2d velocityAdjustedSpeakerPose = vision.getSpeakerPoseAdjustedForVelocity();


		// Calculate ground distance adjusting for velocity
		double velocityAdjustedGroundDistance = shooterMountPose.getTranslation()
				.getDistance(velocityAdjustedSpeakerPose);


		// Calculate the target rotation of the shooter mount in degrees
		double targetRotation = shooterMount.getShooterMountAngleTreeMap().get(velocityAdjustedGroundDistance);

		shooterMount.setTargetRotation(targetRotation);
	}

	@Override
	public void end(boolean interrupted)
	{
		shooter.setShooterMotorVelocity(ShooterConstants.SHOOTER_REST_VELOCITY,
				"Ending auto aiming at speaker");
		shooterMount.setTargetRotation(ShooterMountConstants.SHOOTER_MOUNT_MIN_ANGLE);
	}

}
