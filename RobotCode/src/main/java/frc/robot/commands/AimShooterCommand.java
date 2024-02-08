package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterMountConstants;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Points shooter mount at speaker
 * Returns shooter mount to rest position when done
 */
public class AimShooterCommand extends Command
{

	private ShooterSubsystem shooter;
	private ShooterMountSubsystem shooterMount;
	private VisionSubsystem vision;

	private Translation2d shooterMountPose, speakerPose;

	private AprilTagFieldLayout aprilTagFieldLayout;

	public AimShooterCommand(ShooterSubsystem shooter, ShooterMountSubsystem shooterMount,
			VisionSubsystem vision)
	{
		this.shooter = shooter;
		this.shooterMount = shooterMount;
		this.vision = vision;

		addRequirements(shooter, shooterMount, vision);

		aprilTagFieldLayout = vision.getAprilTagFieldLayout();

		shooterMountPose = new Translation2d();
		speakerPose = new Translation2d();
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
		DriverStation.Alliance allianceColor = DriverStation.getAlliance().orElse(null);

		if (allianceColor == DriverStation.Alliance.Red)
		{
			Pose3d tagPose = aprilTagFieldLayout.getTagPose(4).orElse(new Pose3d());

			speakerPose = new Translation2d(tagPose.getX(), tagPose.getY());
		}
		else if (allianceColor == DriverStation.Alliance.Blue)
		{
			Pose3d tagPose = aprilTagFieldLayout.getTagPose(7).orElse(new Pose3d());

			speakerPose = new Translation2d(tagPose.getX(), tagPose.getY());
		}

		shooterMountPose = new Translation2d(
				vision.getShooterMountPose().orElse(new Pose2d()).getX(),
				vision.getShooterMountPose().orElse(new Pose2d()).getY());

		// Get the distance from the shooter mount to the base of the speaker
		double groundDistance = speakerPose.getDistance(shooterMountPose);

		double targetRotation = Math
				.atan(ShooterMountConstants.SHOOTER_MOUNT_TO_SPEAKER / groundDistance) * (180/ Math.PI);

		// Get the distance from the shooter mount to speaker opening
		double hypotenuse = Math.sqrt(Math.pow(ShooterMountConstants.SHOOTER_MOUNT_TO_SPEAKER, 2)
				+ Math.pow(groundDistance, 2));

		double gravityCompensation = (2 * Math.pow(hypotenuse, 2)) + (2 * hypotenuse) + (2);

		shooterMount.setTargetRotation(targetRotation + gravityCompensation);
	}

	@Override
	public void end(boolean interrupted)
	{
		shooter.setShooterMotorVelocity(ShooterConstants.SHOOTER_REST_VELOCITY,
				"Ending auto aiming at speaker");
		shooterMount.setTargetRotation(ShooterMountConstants.SHOOTER_MOUNT_MIN_ANGLE);
	}

}
