package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.lib.core.LogitechControllerButtons;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RobotContainer
{

	private static RobotContainer instance;

	private final ShuffleboardTab ShuffleboardTab;

	private final SendableChooser<Command> AutoChooser;

	private final SwerveDriveSubsystem SwerveDrive;
	private final ClimberSubsystem ClimberSubsystem;
	private final ShooterSubsystem ShooterSubsystem;

	private final ShooterMountSubsystem ShooterMountSubsystem;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer()
	{
		instance = this;

		ShuffleboardTab = Shuffleboard.getTab("Tab 1");

		AutoChooser = AutoBuilder.buildAutoChooser();

		// Instantiate subsystems
		SwerveDrive = new SwerveDriveSubsystem();
		ClimberSubsystem = new ClimberSubsystem();
		ShooterSubsystem = new ShooterSubsystem();
		ShooterMountSubsystem = new ShooterMountSubsystem();

		Autonomous.init(this);

		// Configure the button bindings
		configurePrimaryBindings();
		configureSecondaryBindings();
	}

	private void configurePrimaryBindings()
	{
		final Joystick PrimaryController = new Joystick(0);

		SwerveDrive.setDefaultCommand(SwerveDrive.getDefaultCommand(PrimaryController));
	}

	private void configureSecondaryBindings()
	{
		final Joystick secondaryController = new Joystick(1);
		final JoystickButton rightTrigger = new JoystickButton(secondaryController,
				LogitechControllerButtons.triggerRight);

		rightTrigger.whileTrue(new ShooterCommand(ShooterSubsystem));
	}

	public static ShuffleboardTab getShuffleboardTab()
	{
		return instance.ShuffleboardTab;
	}

	public SwerveDriveSubsystem getSwerveDrive()
	{
		return SwerveDrive;
	}

	public ShooterSubsystem getShooter()
	{
		return ShooterSubsystem;
	}

}
