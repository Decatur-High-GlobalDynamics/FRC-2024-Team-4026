package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase
{

	// private final PhotonCamera Camera;
	// private PhotonPoseEstimator robotPoseEstimator;

	private final CommandSwerveDrivetrain Swerve;

	public VisionSubsystem(CommandSwerveDrivetrain swerve)
	{
		Swerve = swerve;

		// Camera = new PhotonCamera(VisionConstants.CAMERA_TABLE_NAME);

		// robotPoseEstimator = new PhotonPoseEstimator(Constants.AprilTagFieldLayout,
		// 		PoseStrategy.LOWEST_AMBIGUITY, Camera,
		// 		VisionConstants.ROBOT_TO_CAMERA_OFFSET);
	}

	@Override
	public void periodic()
	{
		// Swerve.updatePoseWithVision(robotPoseEstimator.update());
		// try {
		// 	EstimatedRobotPose estimatedRobotPose = robotPoseEstimator.update().get();
		// 	Swerve.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
		// }
		// finally {

		// }
	}

}