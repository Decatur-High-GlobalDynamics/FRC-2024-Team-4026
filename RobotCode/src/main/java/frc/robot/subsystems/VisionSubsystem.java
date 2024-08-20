package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase
{

	private final PhotonCamera Camera;
	private final PhotonPoseEstimator robotPoseEstimator;

	private final SwerveDriveSubsystem Swerve;

	public VisionSubsystem(SwerveDriveSubsystem swerve)
	{
		
		Swerve = swerve;

		PhotonCamera Camera = new PhotonCamera(VisionConstants.CAMERA_TABLE_NAME);

		 robotPoseEstimator = new PhotonPoseEstimator(Constants.AprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, Camera, VisionConstants.ROBOT_TO_CAMERA_OFFSET);
	}

	public boolean noteObjectFound(boolean b)
	{
		return Camera.hasTargets();
	
	}

	@Override
	public void periodic()
	{
		Swerve.updatePoseWithVision(robotPoseEstimator.update());
	}

}