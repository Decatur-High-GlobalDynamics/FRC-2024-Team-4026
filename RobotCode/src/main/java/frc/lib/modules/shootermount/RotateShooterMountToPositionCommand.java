package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.robot.constants.ShooterMountConstants;

/**
 * Rotates the shooter mount to a position and then ends once within a number of
 * ticks specified by
 * {@link ShooterMountConstants#AIMING_DEADBAND}. Holds position after ending.
 */
public class RotateShooterMountToPositionCommand extends Command {

    private ShooterMountSubsystem shooterMount;
    private double targetPosition;

    /**
     * This constructor is the version that uses a constant target
     * 
     * @param position in encoder ticks
     */
    public RotateShooterMountToPositionCommand(ShooterMountSubsystem shooterMount,
            double targetPosition) {
        this.shooterMount = shooterMount;
        this.targetPosition = targetPosition;

        addRequirements(shooterMount);
    }

    @Override
    public void initialize() {
        shooterMount.setTargetRotation(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        shooterMount.setTargetRotation(0);
    }

}
