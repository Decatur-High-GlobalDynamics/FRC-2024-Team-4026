package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.modules.swervedrive.SwerveConstants;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.lib.modules.swervedrive.Commands.AutoAimSwerveCommand;
import frc.lib.modules.swervedrive.Commands.DriveDistanceAuto;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Autonomous
{

    private enum StartingPosition
    {
        Amp, Middle, HumanPlayer
    }

    private enum AutoMode
    {
        DoNothing, Leave, MultiNote
    }

    private static Autonomous instance;

    private final RobotContainer RobotContainer;

    private final ShuffleboardTab Gui;
    private final SendableChooser<StartingPosition> StartingPositionChooser;
    private final SendableChooser<AutoMode> AutoModeChooser;

    private Autonomous(RobotContainer robotContainer)
    {
        instance = this;
        RobotContainer = robotContainer;

        Gui = frc.robot.RobotContainer.getShuffleboardTab();

        // Set up starting position chooser
        final SendableChooser<StartingPosition> StartingPositionChooser = new SendableChooser<>();
        StartingPositionChooser.setDefaultOption("Middle", StartingPosition.Middle);
        StartingPositionChooser.addOption("Amp Side", StartingPosition.Amp);
        StartingPositionChooser.addOption("Middle", StartingPosition.Middle);
        StartingPositionChooser.addOption("Human Player Side", StartingPosition.HumanPlayer);
        Gui.add("Starting Position", StartingPositionChooser);
        this.StartingPositionChooser = StartingPositionChooser;

        // Set up auto mode chooser
        final SendableChooser<AutoMode> AutoModeChooser = new SendableChooser<>();
        AutoModeChooser.setDefaultOption("Do Nothing", AutoMode.DoNothing);
        AutoModeChooser.addOption("Do Nothing", AutoMode.DoNothing);
        AutoModeChooser.addOption("Leave", AutoMode.Leave);
        AutoModeChooser.addOption("Multi Note", AutoMode.MultiNote);
        Gui.add("Auto Mode", AutoModeChooser);
        this.AutoModeChooser = AutoModeChooser;
    }

    /** Creates an instance and adds auto options to Shuffleboard */
    public static void init(RobotContainer robotContainer)
    {
        if (instance == null)
        {
            new Autonomous(robotContainer);
        }
        else if (instance.RobotContainer != robotContainer)
        {
            System.err.println("Cannot reinitialize Autonomous!");
        }
    }

    /**
     * Parses selected options into a single command
     */
    public Optional<Command> buildAutoCommand()
    {
        final StartingPosition StartingPosition = StartingPositionChooser.getSelected();
        final AutoMode AutoMode = AutoModeChooser.getSelected();

        final SwerveDriveSubsystem SwerveDrive = RobotContainer.getSwerveDrive();
        final ShooterSubsystem Shooter = RobotContainer.getShooter();
        final ShooterMountSubsystem ShooterMount = RobotContainer.getShooterMount();
        final VisionSubsystem Vision = RobotContainer.getVision();

        final SequentialCommandGroup AutoMain = new SequentialCommandGroup();
        final ParallelRaceGroup AutoAsync = new ParallelRaceGroup(
                new AutoAimSwerveCommand(SwerveDrive, Vision), AutoMain);

        switch (AutoMode)
        {
        case DoNothing:
            return Optional.empty();

        case Leave:
            AutoMain.addCommands(new DriveDistanceAuto(AutoConstants.LEAVE_DISTANCE,
                    SwerveConstants.AutoConstants.MAX_SPEED, SwerveDrive));
            break;

        case MultiNote:
            String[] pathSequence = new String[0];

            switch (StartingPosition)
            {
            case Amp:
            case Middle:
                pathSequence = new String[]
                {
                        StartingPosition == Autonomous.StartingPosition.Amp
                                ? "Top Start to Top Note"
                                : "Middle Start to Top Note",
                        "Top to Middle Note", "Middle to Bottom Note",
                };
                break;
            case HumanPlayer:
                pathSequence = new String[]
                {
                        "Bottom Start to Bottom Note", "Bottom to Middle Note",
                        "Middle to Top Note",
                };
                break;
            }

            for (String pathName : pathSequence)
            {
                // Add intake and aiming command once we have that!
                AutoMain.addCommands(followPath(pathName));
            }

            break;
        }

        return Optional.ofNullable(AutoAsync);
    }

    /** Calls {@link #buildAutoCommand()} */
    public static Optional<Command> getAutoCommand()
    {
        return instance.buildAutoCommand();
    }

    /** Closes the instance's SendableChoosers to free up resources */
    public static void close()
    {
        instance.StartingPositionChooser.close();
        instance.AutoModeChooser.close();
    }

    /**
     * Returns a command to follow a path from PathPlanner GUI whilst avoiding obstacles
     * 
     * @param pathName The filename of the path to follow w/o file extension. Must be in the paths
     *                 folder. Ex: Example Human Player Pickup
     * @return A command that will drive the robot along the path
     */
    private Command followPath(final String pathName)
    {
        final PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.pathfindThenFollowPath(path,
                SwerveConstants.AutoConstants.PathConstraints);
    }

}
