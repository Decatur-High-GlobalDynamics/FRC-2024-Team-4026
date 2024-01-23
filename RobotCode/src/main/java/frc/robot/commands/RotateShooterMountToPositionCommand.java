package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterMountSubsystem;

public class RotateShooterMountToPositionCommand extends Command
{

    private final ShooterMountSubsystem ShooterMountSubsystem;

    private DoubleSupplier getPosition;

    /**
     * This constructor is the version that allows for variable targets (ex: based on joystick)
     * 
     * @param getPosition in degrees
     */
    public RotateShooterMountToPositionCommand(ShooterMountSubsystem subsystem,
            DoubleSupplier getPosition)
    {
        ShooterMountSubsystem = subsystem;
        this.getPosition = getPosition;

        // We do this before setting the rotation so that any other
        // command controlling the shooter mount will end before we set the rotation
        addRequirements(subsystem);
    }

    /**
     * This constructor is the version that uses a constant target
     * 
     * @param position in degrees
     */
    public RotateShooterMountToPositionCommand(ShooterMountSubsystem subsystem, double position)
    {
        this(subsystem, () -> position);
    }

    @Override
    public void execute()
    {
        ShooterMountSubsystem.setGoalRotation(getPosition.getAsDouble());
    }

    @Override
    public boolean isFinished()
    {
        // We do the weird static access to avoid going through the variable
        return Math.abs(ShooterMountSubsystem.getCurrentRotation()
                - getPosition.getAsDouble()) < frc.robot.subsystems.ShooterMountSubsystem.DEADBAND;
    }

}
