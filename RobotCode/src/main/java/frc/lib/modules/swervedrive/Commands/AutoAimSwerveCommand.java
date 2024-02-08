package frc.lib.modules.swervedrive.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** Rotates the chassis towards the speaker. Intended to work with PathPlanner paths */
public class AutoAimSwerveCommand extends Command
{

    private final SwerveDriveSubsystem Swerve;
    private final VisionSubsystem Vision;
    private final IndexerSubsystem Indexer;

    private boolean shooting;

    public AutoAimSwerveCommand(SwerveDriveSubsystem swerve, VisionSubsystem vision,
            IndexerSubsystem indexer)
    {
        Swerve = swerve;
        Vision = vision;
        Indexer = indexer;
    }

    @Override
    public void initialize()
    {
        Swerve.setRotationController(() -> Swerve.getRotationToSpeaker(Vision));
    }

    @Override
    public void execute()
    {
        // Spin feeder motors if in target
        if (Math.abs(Swerve.getRotationToSpeaker(Vision)) < VisionConstants.CHASSIS_AIM_THRESHOLD)
        {
            // Spin feeder motors
            Indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_SHOOT_VELOCITY,
                    "Within aim threshold");

            shooting = true;
        }
        else if (!Indexer.hasNote())
        {
            Indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REST_VELOCITY,
                    "Not within aim threshold");
            shooting = false;
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        Swerve.setRotationController(null);
    }
}