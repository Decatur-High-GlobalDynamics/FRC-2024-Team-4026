package frc.lib.modules.shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.leds.TeamColor;
import frc.lib.modules.leds.LedSubsystem;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * <p>
 * Spins indexer motors until the note has been fired.
 * </p>
 * <p>
 * Requires {@link IndexerSubsystem}.
 * </p>
 */
public class ShootCommand extends Command
{

    private final IndexerSubsystem Indexer;
    private final LedSubsystem Leds;

    public ShootCommand(IndexerSubsystem indexer, LedSubsystem leds)
    {

        Indexer = indexer;
        Leds = leds;

        addRequirements(Indexer);
    }

    @Override
    public void initialize()
    {
        Indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_SHOOT_VELOCITY);
    }

    @Override
    public void end(boolean interrupted)
    {
         /*we made our leds signal different states of the robot, this one is probably for
        when the robot fires. (in the hole)
        */
        Indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REST_VELOCITY);
        if(Leds != null)
            Leds.flashAllPixels(TeamColor.Blue, 5);
    }

    @Override
    public boolean isFinished()
    {
        return !Indexer.hasNote();
    }

}
