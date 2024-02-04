package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.constants.ClimberConstants;
import frc.lib.core.ILogSource;

public class ClimberCommand extends Command implements ILogSource
{
    private ClimberSubsystem climber;
    private DoubleSupplier leftInput, rightInput;
    private final double DEADBAND_RANGE = 0.1;

    public ClimberCommand(ClimberSubsystem c1, DoubleSupplier leftInput, DoubleSupplier rightInput)
    {
        climber = c1;
        this.leftInput = leftInput;
        this.rightInput = rightInput;
        addRequirements(climber);
    }

    @Override
    public void initialize()
    {
        logFine("Command started");
    }
    
    @Override
    public void execute()
    {
        double realLeftPower = 0, realRightPower = 0;
        if (Math.abs(leftInput.getAsDouble()) > DEADBAND_RANGE)
        {
            realLeftPower = ClimberConstants.MAX_SPEED * -leftInput.getAsDouble();
        }

        if (Math.abs(rightInput.getAsDouble()) > DEADBAND_RANGE)
        {
            realRightPower = ClimberConstants.MAX_SPEED * -rightInput.getAsDouble();
        }

        climber.setPowers(realLeftPower, realRightPower, "climbing");

    } 
    
    @Override
    public void end(boolean interrupted)
    {
        logFine("Command finished.");

    }
}
