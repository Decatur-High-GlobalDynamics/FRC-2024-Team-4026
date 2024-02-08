package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase
{

    private final PhotonCamera Camera;
    private PhotonPoseEstimator robotPoseEstimator, shooterMountPoseEstimator;

    private PhotonPipelineResult latestPipelineResult;
    private Optional<PhotonTrackedTarget> bestTarget;

    private final AprilTagFieldLayout FieldLayout;

    public VisionSubsystem()
    {
        Camera = new PhotonCamera(VisionConstants.CameraTableName);

        FieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                new Rotation3d(0, 0, 0));
        Transform3d shooterMountToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                new Rotation3d(0, 0, 0));

        robotPoseEstimator = new PhotonPoseEstimator(FieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Camera, robotToCam);
        shooterMountPoseEstimator = new PhotonPoseEstimator(FieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Camera, shooterMountToCam);
    }

    @Override
    public void periodic()
    {
        latestPipelineResult = Camera.getLatestResult();

        // Pay attention to PhotonTargetSortMode!
        bestTarget = Optional.ofNullable(latestPipelineResult.getBestTarget());
    }

    /**
     * @return the yaw offset of the {@link #bestTarget}, or 0 if no target is found
     */
    public double getYawOffset()
    {
        return bestTarget.map(PhotonTrackedTarget::getYaw).orElse(0.0);
    }

    /**
     * @return the pitch offset of the {@link #bestTarget} , or 0 if no target is found
     */
    public double getPitchOffset()
    {
        return bestTarget.map(PhotonTrackedTarget::getPitch).orElse(0.0);
    }

    public Optional<Pose2d> getRobotPose()
    {
        Optional<EstimatedRobotPose> estPose = robotPoseEstimator.update();
        if (estPose.isPresent())
        {
            EstimatedRobotPose pose = estPose.get();
            Pose3d pose3d = pose.estimatedPose;

            return Optional.of(pose3dtoPose2d(pose3d));
        }

        return Optional.empty();
    }

    public Optional<Pose2d> getShooterMountPose()
    {
        Optional<EstimatedRobotPose> estPose = shooterMountPoseEstimator.update();
        if (estPose.isPresent())
        {
            EstimatedRobotPose pose = estPose.get();
            Pose3d pose3d = pose.estimatedPose;

            return Optional.of(pose3dtoPose2d(pose3d));
        }

        return Optional.empty();
    }

    /**
     * Convert a Pose3d to a Pose2d. Conversion: Pose3d.X -> Pose2d.X Pose3d.Z -> Pose2d.Y
     * Pose3d.Rotation.Y -> Pose2d.Rotation
     */
    private Pose2d pose3dtoPose2d(Pose3d pose3d)
    {
        // We only care about the x and z, and the yaw
        Rotation2d rot2d = new Rotation2d(pose3d.getRotation().getY());
        return new Pose2d(pose3d.getX(), pose3d.getZ(), rot2d);
    }

    public double angleTowardsPose(Pose2d targetPose)
    {
        Optional<Pose2d> robotPoseOptional = getRobotPose();

        if (robotPoseOptional.isEmpty())
        {
            return 0.0;
        }

        return angleBetweenPoses(robotPoseOptional.get(), targetPose);
    }

    /**
     * @return the angle to the target pose in radians. Counterclockwise rotation is negative.
     */
    public static double angleBetweenPoses(Pose2d thisPose, Pose2d targetPose)
    {
        // Calculate the distance to the target
        Pose2d distance = new Pose2d(targetPose.getX() - thisPose.getX(),
                targetPose.getY() - thisPose.getY(), new Rotation2d());

        // Use the inverse tangent to calculate the angle
        // Atan2 accounts for the sign of the x and y values
        // We want the angle for (0, 0) to (0, 1) to be 0, so we add 90 degrees
        // We multiply by -1 to make the angle negative when the target is to the left
        return -Math.atan2(distance.getY(), distance.getX()) + Math.PI / 2;
    }

    /** @return What angle to turn to to face the speaker */
    public double getAngleToSpeaker()
    {
        // Get the target position
        int targetId = Alliance.Blue == DriverStation.getAlliance().get()
                ? VisionConstants.BLUE_SPEAKER_TAG_ID
                : VisionConstants.RED_SPEAKER_TAG_ID;
        Pose2d targetPose = pose3dtoPose2d(FieldLayout.getTagPose(targetId).orElse(new Pose3d()));

        double angle = angleTowardsPose(targetPose);

        SmartDashboard.putNumber("Angle to Speaker", angle);
        return angle;
    }

    public AprilTagFieldLayout getAprilTagFieldLayout()
    {
        return aprilTagFieldLayout;
    }

}
