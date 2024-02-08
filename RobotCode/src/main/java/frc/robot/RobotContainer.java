package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.lib.modules.swervedrive.Commands.TeleopAimSwerveCommand;
import frc.lib.core.LogitechControllerButtons;
import frc.robot.commands.ShooterOverrideCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RobotContainer
{

	private static RobotContainer instance;

	private final ShuffleboardTab ShuffleboardTab;

	private final SwerveDriveSubsystem SwerveDrive;
	private final ClimberSubsystem ClimberSubsystem;
	private final ShooterSubsystem ShooterSubsystem;
	private final ShooterMountSubsystem ShooterMountSubsystem;
	private final VisionSubsystem VisionSubsystem;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer()
	{
		instance = this;

		ShuffleboardTab = Shuffleboard.getTab("Tab 1");

		// Instantiate subsystems
		SwerveDrive = new SwerveDriveSubsystem();
		ClimberSubsystem = new ClimberSubsystem();
		ShooterSubsystem = new ShooterSubsystem();
		ShooterMountSubsystem = new ShooterMountSubsystem();
		VisionSubsystem = new VisionSubsystem();

		Autonomous.init(this);

		// Configure the button bindings
		configurePrimaryBindings();
		configureSecondaryBindings();
	}

	private void configurePrimaryBindings()
	{
		final Joystick PrimaryController = new Joystick(0);

		final JoystickButton rightTrigger = new JoystickButton(PrimaryController,
				LogitechControllerButtons.triggerRight);

		SwerveDrive.setDefaultCommand(SwerveDrive.getDefaultCommand(PrimaryController));
		rightTrigger.whileTrue(SwerveDrive.getTeleopAimCommand(PrimaryController, VisionSubsystem));
	}

	private void configureSecondaryBindings()
	{
		final Joystick secondaryController = new Joystick(1);
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

	public ShooterMountSubsystem getShooterMount()
	{
		return ShooterMountSubsystem;
	}

	public VisionSubsystem getVision()
	{
		return VisionSubsystem;
	}

}
