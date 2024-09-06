package frc.lib.core;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class SimpleAuto extends Autonomous{

    private final SendableChooser<Command> AutoChooser;

    public SimpleAuto(RobotContainer RobotContainer) {
        super(RobotContainer);
        AutoChooser = AutoBuilder.buildAutoChooser();
    }

    @Override
    public Optional<Command> buildAutoCommand()
    {
        return Optional.ofNullable(AutoChooser.getSelected());
    }
    
}
