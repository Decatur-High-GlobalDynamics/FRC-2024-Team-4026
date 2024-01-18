package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
// import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.core.LogitechControllerButtons;
import frc.robot.Commands.ShooterCommand;

import frc.robot.Subsystems.ShooterSubsystem;

public class RobotContainer
{

	private final static ShuffleboardTab shuffleboard = Shuffleboard.getTab("Tab 1");

	private final SendableChooser<Command> autoChooser = new SendableChooser<>();

	public static Joystick primaryController;
	public static Joystick secondaryController;
	public static ShooterSubsystem shooting;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer()
	{
		shooting = new ShooterSubsystem();

		addAutonomousOptions();

		// Configure the button bindings
		configurePrimaryBindings();
		configureSecondaryBindings();
	}

	private void configurePrimaryBindings()
	{

	}

	private void configureSecondaryBindings()
	{
		secondaryController = new Joystick(1);
		JoystickButton x = new JoystickButton(secondaryController, LogitechControllerButtons.x);
		JoystickButton rightTrigger = new JoystickButton(secondaryController,
				LogitechControllerButtons.triggerRight);

		rightTrigger.whileTrue(new ShooterCommand(shooting));
	}

	// Add autonomous options to the SendableChooser
	public void addAutonomousOptions()
	{

	}

	public Command getAutonomousCommand()
	{
		return autoChooser.getSelected();
	}

	public static ShuffleboardTab getShuffleboard()
	{
		return shuffleboard;
	}

}
