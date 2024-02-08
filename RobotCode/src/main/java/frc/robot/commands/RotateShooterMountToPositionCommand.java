package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterMountConstants;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.lib.core.ILogSource;

public class RotateShooterMountToPositionCommand extends Command implements ILogSource
{

    private ShooterMountSubsystem shooterMount;
	  private double targetPosition;

    /**
     * This constructor is the version that uses a constant target
     * 
     * @param position in degrees
     */
    public RotateShooterMountToPositionCommand(ShooterMountSubsystem shooterMount, double targetPosition)
    {
        this.shooterMount = shooterMount;
		    this.targetPosition = targetPosition;

		addRequirements(shooterMount);
    }

    public void initialize()
    {
        logFine("Shooter Mount Rotating...");
    }

    @Override
    public void initialize()
    {
        shooterMount.setTargetRotation(targetPosition);
    }

    @Override
    public boolean isFinished()
    {
        // We do the weird static access to avoid going through the variable
        return endAutomatically && Math.abs(ShooterMountSubsystem.getCurrentRotation()
                - getPosition.getAsDouble()) < frc.robot.subsystems.ShooterMountSubsystem.DEADBAND;
    }

    @Override
    public void end(boolean interrupted)
    {
        logFine("Shooter Mount Stopped");
        shooterMount.setTargetRotation(ShooterMountConstants.SHOOTER_MOUNT_MIN_ANGLE);
    }
}
