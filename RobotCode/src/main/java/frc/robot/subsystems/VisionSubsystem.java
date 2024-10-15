package frc.robot.subsystems;

import java.util.NoSuchElementException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase
{

	private final PhotonCamera Camera;
	private PhotonPoseEstimator robotPoseEstimator;

	private final CommandSwerveDrivetrain Swerve;

	public VisionSubsystem(CommandSwerveDrivetrain swerve)
	{
		Swerve = swerve;

		Camera = new PhotonCamera(VisionConstants.CAMERA_TABLE_NAME);

		robotPoseEstimator = new PhotonPoseEstimator(Constants.AprilTagFieldLayout,
				PoseStrategy.LOWEST_AMBIGUITY, Camera,
				VisionConstants.ROBOT_TO_CAMERA_OFFSET);
	}

	@Override
	public void periodic()
	{
		try {
			EstimatedRobotPose estimatedRobotPose = robotPoseEstimator.update().get();
			// Swerve.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);1

			// System.out.println(estimatedRobotPose.estimatedPose.toPose2d().toString());
		}
		catch (NoSuchElementException e) {
			
		}
		finally {

		}
	}

}