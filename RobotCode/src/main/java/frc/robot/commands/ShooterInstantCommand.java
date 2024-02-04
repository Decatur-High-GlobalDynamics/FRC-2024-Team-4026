package frc.robot.commands;

import frc.lib.core.util.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** Runs the shooter and then ends. Semi-auto firing mode. */
public class ShooterInstantCommand extends Command
{

    private ShooterSubsystem shooter;
    private Timer timer;

    public ShooterInstantCommand(ShooterSubsystem shooter)
    {
        this.shooter = shooter;
        timer = new Timer(500);
    }

    @Override
    public void initialize()
    {
        shooter.setShooterMotorPower(1.0, "instant shooter command started");
        timer.reset();
    }

    @Override
    public boolean isFinished()
    {
        return timer.isDone();
    }

    @Override
    public void end(boolean interrupted)
    {
        shooter.setShooterMotorPower(0, "instant shooter command ended");
    }

}