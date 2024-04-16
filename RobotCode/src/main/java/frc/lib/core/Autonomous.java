package frc.lib.core;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.modules.swervedrive.SwerveConstants;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoShooterOverrideCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateShooterMountToPositionCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterOverrideCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterMountConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * <p>
 * </p>
 * <p>
 * <b>Usage:</b> Call {@link #Autonomous} in RobotContainer's constructor. Then, to actually get the
 * autonomous command, call {@link #getAutoCommand()}.
 * </p>
 */
public abstract class Autonomous implements ILogSource
{

    private final RobotContainer RobotContainer;

    public Autonomous(final RobotContainer RobotContainer)
    {
        this.RobotContainer = RobotContainer;
        registerNamedCommands();
    }

    /** Registers commands for building autos through PathPlanner. */
    private void registerNamedCommands()
    {
        logFine("Registering named commands...");

        // Get subsystems
        final ShooterMountSubsystem ShooterMount = RobotContainer.getShooterMount();
        final IndexerSubsystem Indexer = RobotContainer.getIndexer();
        final IntakeSubsystem Intake = RobotContainer.getIntake();
        final LedSubsystem Leds = RobotContainer.getLeds();
        final ShooterSubsystem Shooter = RobotContainer.getShooter();
        final SwerveDriveSubsystem Swerve = RobotContainer.getSwerveDrive();

        // Initialize commands
        NamedCommands.registerCommand("Intake",
                new IntakeCommand(Intake, Indexer, ShooterMount, Shooter, Leds));

        NamedCommands.registerCommand("Shoot from Subwoofer",
                new AutoShooterOverrideCommand(ShooterMount, Shooter, Indexer, Leds,
                ShooterMountConstants.SHOOTER_MOUNT_SPEAKER_ANGLE_FIXED_OFFSET));

        NamedCommands.registerCommand("Shoot from Note Center",
                new AutoShooterOverrideCommand(ShooterMount, Shooter, Indexer, Leds,
                ShooterMountConstants.SHOOTER_MOUNT_NOTE_CENTER_ANGLE_FIXED_OFFSET));

        NamedCommands.registerCommand("Drop Note", new ShooterOverrideCommand(Shooter, Indexer,
                Leds, ShooterConstants.SHOOTER_AMP_VELOCITY, false));

        NamedCommands.registerCommand("Shoot from Note Side",
                new AutoShooterOverrideCommand(ShooterMount, Shooter, Indexer, Leds,
                ShooterMountConstants.SHOOTER_MOUNT_PODIUM_ANGLE_FIXED_OFFSET));

        NamedCommands.registerCommand("Aim from Note Center", Swerve.getAutoAimSwerveCommand(AutoConstants.CHASSIS_ROTATION_NOTE_CENTER));

        NamedCommands.registerCommand("Aim from Note Source", Swerve.getAutoAimSwerveCommand(AutoConstants.CHASSIS_ROTATION_NOTE_SOURCE));
		
        NamedCommands.registerCommand("Aim from Note Amp", Swerve.getAutoAimSwerveCommand(AutoConstants.CHASSIS_ROTATION_NOTE_AMP));

        // Populate rotation commands
        for (double rot : AutoConstants.AutoShooterMountRotations)
        {
            NamedCommands.registerCommand("Aim to " + rot + " deg",
                    new RotateShooterMountToPositionCommand(ShooterMount, rot));
            NamedCommands.registerCommand("Shoot then Aim to " + rot + " deg",
                    new SequentialCommandGroup(new ShootCommand(Indexer, Leds),
                            new RotateShooterMountToPositionCommand(ShooterMount, rot)));
        }
        registerAutosAsNamedCommands();
    }

    private void registerAutosAsNamedCommands()
    {
        for (String auto : AutoBuilder.getAllAutoNames())
            NamedCommands.registerCommand("Auto " + auto, getPathPlannerAuto(auto));

    }

    /**
     * Returns a command to follow a path from PathPlanner GUI whilst avoiding obstacles
     * 
     * @param PathName The filename of the path to follow w/o file extension. Must be in the paths
     *                 folder. Ex: Example Human Player Pickup
     * @return A command that will drive the robot along the path
     */
    protected static Command followPath(final String PathName)
    {
        final PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);
        return AutoBuilder.pathfindThenFollowPath(path,
                SwerveConstants.AutoConstants.PathConstraints);
    }

    protected static Command getPathPlannerAuto(final String PathName)
    {
        return new PathPlannerAuto(PathName);

    }

    public abstract Optional<Command> buildAutoCommand();

    protected RobotContainer getRobotContainer()
    {
        return RobotContainer;
    }

}