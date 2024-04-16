package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.modules.leds.Color;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.lib.modules.swervedrive.Commands.ZeroGyroCommand;
import frc.lib.core.Autonomous;
import frc.lib.core.LogitechControllerButtons;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.ClimberSpeedCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.RotateShooterMountToPositionCommand;
import frc.robot.commands.ShooterOverrideCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterMountConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * p The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer
{

	private static RobotContainer instance;

	private final ShuffleboardTab ShuffleboardTab;

	private final SwerveDriveSubsystem SwerveDrive;
	private final ClimberSubsystem ClimberSubsystem;
	private final ShooterSubsystem ShooterSubsystem;
	private final ShooterMountSubsystem ShooterMountSubsystem;
	private final VisionSubsystem VisionSubsystem;
	private final IndexerSubsystem IndexerSubsystem;
	private final IntakeSubsystem IntakeSubsystem;
	private final LedSubsystem LedSubsystem;

	private final Autonomous Autonomous;

	private final Pigeon2 gyro;

	private final PowerDistribution pdh;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer()
	{
		instance = this;

		pdh = new PowerDistribution(26, ModuleType.kRev);
		pdh.setSwitchableChannel(true);

		ShuffleboardTab = Shuffleboard.getTab("Tab 1");

		gyro = new Pigeon2(Ports.PIGEON_GYRO, Constants.CANIVORE_NAME);
		gyro.optimizeBusUtilization();
		gyro.getYaw().setUpdateFrequency(20);

		// Instantiate subsystems
		SwerveDrive = new SwerveDriveSubsystem();
		ClimberSubsystem = new ClimberSubsystem();
		ShooterSubsystem = new ShooterSubsystem();
		ShooterMountSubsystem = new ShooterMountSubsystem();
		VisionSubsystem = new VisionSubsystem(SwerveDrive);
		IndexerSubsystem = new IndexerSubsystem();
		IntakeSubsystem = new IntakeSubsystem();
		LedSubsystem = new LedSubsystem();

		LedSubsystem.setAllPixels(Color.Blue);

		Autonomous = new SideBasedAuto(this);

		// Configure the button bindings
		configurePrimaryBindings();
		configureSecondaryBindings();
	}

	private void configurePrimaryBindings()
	{
		final Joystick PrimaryController = new Joystick(0);

		final JoystickButton LeftTrigger = new JoystickButton(PrimaryController,
				LogitechControllerButtons.triggerLeft);
		final JoystickButton RightTrigger = new JoystickButton(PrimaryController,
				LogitechControllerButtons.triggerRight);
		final JoystickButton LeftBumper = new JoystickButton(PrimaryController,
				LogitechControllerButtons.bumperLeft);
		final JoystickButton YButton = new JoystickButton(PrimaryController,
				LogitechControllerButtons.y);

		// Swerve
		SwerveDrive.setDefaultCommand(SwerveDrive.getDefaultCommand(PrimaryController));

		// Aim to amp
		LeftTrigger.whileTrue(SwerveDrive.getTeleopAimToPositionCommand(
				PrimaryController, -(Math.PI / 2.0)));

		// Aim to speaker subwoofer
		RightTrigger.whileTrue(
				SwerveDrive.getTeleopAimToPositionAllianceRelativeCommand(PrimaryController, 0));
		// RightTrigger.whileTrue(SwerveDrive.getTeleopAimCommand(PrimaryController,
		// ShooterMountSubsystem, IndexerSubsystem))

		// Aim to speaker podium
		LeftBumper.whileTrue(
				SwerveDrive.getTeleopAimToPositionAllianceRelativeCommand(PrimaryController, -0.5));

		// Zero chassis rotation
		YButton.onTrue(new ZeroGyroCommand(SwerveDrive));
	}

	private void configureSecondaryBindings()
	{
		final Joystick SecondaryController = new Joystick(1);

		final JoystickButton LeftTrigger = new JoystickButton(SecondaryController,
				LogitechControllerButtons.triggerLeft);
		final JoystickButton RightTrigger = new JoystickButton(SecondaryController,
				LogitechControllerButtons.triggerRight);
		final JoystickButton LeftBumper = new JoystickButton(SecondaryController,
				LogitechControllerButtons.bumperLeft);
		final JoystickButton AButton = new JoystickButton(SecondaryController,
				LogitechControllerButtons.a);
		final JoystickButton BButton = new JoystickButton(SecondaryController,
				LogitechControllerButtons.b);
		final JoystickButton XButton = new JoystickButton(SecondaryController,
				LogitechControllerButtons.x);
		final JoystickButton YButton = new JoystickButton(SecondaryController, 
				LogitechControllerButtons.y);
		final JoystickButton LeftButton = new JoystickButton(SecondaryController,
				LogitechControllerButtons.left);

		// Climb
		ClimberSubsystem.setDefaultCommand(new ClimberSpeedCommand(ClimberSubsystem,
				() -> (SecondaryController.getY()), () -> (SecondaryController.getThrottle())));

		// Shoot subwoofer
		LeftTrigger.whileTrue(new ShooterOverrideCommand(ShooterSubsystem, IndexerSubsystem,
				LedSubsystem, ShooterConstants.SHOOTER_SPEAKER_VELOCITY, false));
		LeftTrigger.whileTrue(new RotateShooterMountToPositionCommand(ShooterMountSubsystem,
				ShooterMountConstants.SHOOTER_MOUNT_SPEAKER_ANGLE_FIXED_OFFSET));
		// LeftTrigger.whileTrue(new AimShooterCommand(ShooterSubsystem, ShooterMountSubsystem,
		// SwerveDrive));

		// Shoot podium
		LeftBumper.whileTrue(new ShooterOverrideCommand(ShooterSubsystem, IndexerSubsystem,
				LedSubsystem, ShooterConstants.SHOOTER_SPEAKER_VELOCITY, false));
		LeftBumper.whileTrue(new RotateShooterMountToPositionCommand(ShooterMountSubsystem,
				ShooterMountConstants.SHOOTER_MOUNT_PODIUM_ANGLE_FIXED_OFFSET));

		// Passing
		YButton.whileTrue(new ShooterOverrideCommand(ShooterSubsystem, IndexerSubsystem,
				LedSubsystem, ShooterConstants.SHOOTER_PASSING_VELOCITY, false));
		YButton.whileTrue(new RotateShooterMountToPositionCommand(ShooterMountSubsystem,
				ShooterMountConstants.SHOOTER_MOUNT_PASSING_ANGLE_FIXED_OFFSET));

		// Amp
		AButton.whileTrue(new AmpCommand(ShooterMountSubsystem, ShooterSubsystem, IndexerSubsystem,
				LedSubsystem));

		// Outtake
		BButton.whileTrue(
				new IntakeReverseCommand(IntakeSubsystem, IndexerSubsystem, ShooterSubsystem));

		// Intake
		XButton.whileTrue(new IntakeCommand(IntakeSubsystem, IndexerSubsystem,
				ShooterMountSubsystem, ShooterSubsystem, LedSubsystem));

		// Override indexer
		RightTrigger.whileTrue(new IndexerCommand(IndexerSubsystem));
	}

	public static ShuffleboardTab getShuffleboardTab()
	{
		return instance.ShuffleboardTab;
	}

	/**
	 * @return the position of the speaker april tag for our alliance, or empty if the tag is not
	 *         found
	 */
	public static Optional<Pose3d> getSpeakerPose()
	{
		return Constants.AprilTagFieldLayout
				.getTagPose(DriverStation.getAlliance().get() == Alliance.Blue
						? VisionConstants.BLUE_SPEAKER_TAG_ID
						: VisionConstants.RED_SPEAKER_TAG_ID);
	}

	public static Pigeon2 getGyro()
	{
		return instance.gyro;
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

	public IndexerSubsystem getIndexer()
	{
		return IndexerSubsystem;
	}

	public IntakeSubsystem getIntake()
	{
		return IntakeSubsystem;
	}

	public LedSubsystem getLeds()
	{
		return LedSubsystem;
	}

	public Autonomous getAutonomous()
	{

		return Autonomous;
	}

}