package frc.lib.modules.swervedrive.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAimSwerveCommand extends Command
{

    private final SwerveDriveSubsystem Swerve;
    private final VisionSubsystem Vision;

    public AutoAimSwerveCommand(SwerveDriveSubsystem swerve, VisionSubsystem vision)
    {
        Swerve = swerve;
        Vision = vision;
    }

    @Override
    public void initialize()
    {
        Swerve.setRotationController(Vision::getRotationToSpeaker);
    }

    @Override
    public void execute()
    {
        // Spin feeder motors if in target
        if (Math.abs(Vision.getRotationToSpeaker()) < VisionConstants.CHASSIS_AIM_THRESHOLD)
        {
            // Spin feeder motors
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        Swerve.setRotationController(null);
    }
}