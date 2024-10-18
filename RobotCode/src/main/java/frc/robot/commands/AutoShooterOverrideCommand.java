package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.core.util.TeamCountdown;
import frc.lib.modules.leds.TeamColor;
import frc.lib.modules.leds.LedSubsystem;
import frc.robot.constants.IndexerConstants;
import frc.lib.modules.shooter.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.lib.modules.shootermount.ShooterMountSubsystem;
import frc.lib.modules.shooter.ShooterSubsystem;

public class AutoShooterOverrideCommand extends Command
{

    private ShooterMountSubsystem shooterMount;
    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private LedSubsystem led;
    private TeamCountdown countdown;
    private double targetRotation;

    public AutoShooterOverrideCommand(ShooterMountSubsystem shooterMount, ShooterSubsystem shooter,
            IndexerSubsystem indexer, LedSubsystem led, double targetRotation)
    {
        this.shooterMount = shooterMount;
        this.shooter = shooter;
        this.indexer = indexer;
        this.led = led;
        this.targetRotation = targetRotation;

        addRequirements(shooterMount, shooter, indexer);
    }

    @Override
    public void initialize()
    {
        shooterMount.setTargetRotation(targetRotation);

        countdown = new TeamCountdown(1500);
    }

    @Override
    public void execute()
    {
        // Spins up the motor
        shooter.setShooterMotorVelocity(ShooterConstants.SHOOTER_SPEAKER_VELOCITY);

        // If-statement to see if motor is spun up
        if (shooter.isUpToSpeed() && shooterMount.isAtTargetRotation())
        {
            indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_SHOOT_VELOCITY);
        }
    }

    @Override
    public void end(boolean isFinished)
    {
        shooterMount.setTargetRotation(0);
        shooter.setShooterMotorVelocity(ShooterConstants.SHOOTER_REST_VELOCITY);
        indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REST_VELOCITY);
        led.flashAllPixels(TeamColor.Blue, 5);
    }

    @Override
    public boolean isFinished()
    {
        return countdown.isDone();
    }

}
