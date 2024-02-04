package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase
{

    private final PhotonCamera Camera;

    private PhotonPipelineResult latestPipelineResult;
    private Optional<PhotonTrackedTarget> bestTarget;

    public VisionSubsystem()
    {
        Camera = new PhotonCamera(VisionConstants.CameraTableName);
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

}
