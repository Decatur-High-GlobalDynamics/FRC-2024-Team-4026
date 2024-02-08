package frc.lib.modules.swervedrive.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Rotates the chassis towards the speaker but allows the driver to control translation and strafe
 */
public class TeleopAimSwerveCommand extends TeleopSwerveCommand
{

    private final SwerveDriveSubsystem Swerve;
    private final VisionSubsystem Vision;

    public TeleopAimSwerveCommand(SwerveDriveSubsystem swerve, VisionSubsystem vision,
            DoubleSupplier translationSup, DoubleSupplier strafeSup,
            BooleanSupplier slowSpeedSupplier, BooleanSupplier fastSpeedSupplier)
    {
        super(swerve, translationSup, strafeSup, () -> swerve.getRotationToSpeaker(vision),
                slowSpeedSupplier, fastSpeedSupplier);

        Swerve = swerve;
        Vision = vision;
    }

    @Override
    public void execute()
    {
        super.execute();

        // Spin feeder motors if in target
        if (Math.abs(Swerve.getRotationToSpeaker(Vision)) < VisionConstants.CHASSIS_AIM_THRESHOLD)
        {
            // Spin feeder motors
        }
    }

}
