package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.modules.leds.TeamColor;
import frc.lib.modules.elevator.ElevatorSubsystem;
import frc.lib.modules.elevator.commands.Elevator.ElevatorSpeedCommand;
import frc.lib.modules.leds.LedSubsystem;
import frc.lib.core.Autonomous;
import frc.lib.core.LogitechControllerButtons;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.IndexerCommand;
import frc.lib.modules.intake.Commands.IntakeCommand;
import frc.lib.modules.intake.Commands.IntakeReverseCommand;
import frc.lib.modules.shootermount.RotateShooterMountToPositionCommand;
import frc.lib.modules.shooter.Commands.DriverShooterCommand;
import frc.lib.modules.shooter.Commands.ShootFromDistance;
import frc.lib.modules.shooter.Commands.ShooterOverrideCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.SwerveConstants;
import frc.lib.modules.shooter.ShooterConstants;
import frc.lib.modules.shootermount.ShooterMountConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.lib.modules.intake.IntakeSubsystem;
import frc.lib.modules.shootermount.ShooterMountSubsystem;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.lib.modules.shooter.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * p The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer
{

	private static RobotContainer instance;

	private final ShuffleboardTab ShuffleboardTab;

	private final ElevatorSubsystem ClimberSubsystem;
	private final ShooterSubsystem ShooterSubsystem;
	private final ShooterMountSubsystem ShooterMountSubsystem;
	private final VisionSubsystem VisionSubsystem;
	private final IndexerSubsystem IndexerSubsystem;
	private final IntakeSubsystem IntakeSubsystem;
	private final LedSubsystem LedSubsystem;
	private final CommandSwerveDrivetrain SwerveSubsystem;

	private final Autonomous Autonomous;

	private final PowerDistribution pdh;

	private final Telemetry SwerveLogger;

	private final SwerveRequest.FieldCentric Drive;
	private final SwerveRequest.FieldCentricFacingAngle DriveFacingAngle;
	private final SwerveRequest.SwerveDriveBrake Brake;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer()
	{
		instance = this;

		pdh = new PowerDistribution(26, ModuleType.kRev);
		pdh.setSwitchableChannel(true);

		ShuffleboardTab = Shuffleboard.getTab("Tab 1");

		// Instantiate subsystems
		SwerveSubsystem = TunerConstants.DriveTrain;
		ClimberSubsystem = new ElevatorSubsystem();
		ShooterSubsystem = new ShooterSubsystem();
		ShooterMountSubsystem = new ShooterMountSubsystem();
		VisionSubsystem = new VisionSubsystem(SwerveSubsystem);
		IndexerSubsystem = new IndexerSubsystem();
		IntakeSubsystem = new IntakeSubsystem();
		LedSubsystem = new LedSubsystem();

		// Swerve drive request
		Drive = new SwerveRequest.FieldCentric().withDeadband(SwerveConstants.MAX_SPEED * 0.05)
				.withRotationalDeadband(SwerveConstants.MAX_ANGULAR_SPEED * 0.05)
				.withDriveRequestType(DriveRequestType.Velocity);
		DriveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
				.withDeadband(SwerveConstants.MAX_SPEED * 0.05)
				.withRotationalDeadband(SwerveConstants.MAX_ANGULAR_SPEED * 0.05)
				.withDriveRequestType(DriveRequestType.Velocity);
		DriveFacingAngle.HeadingController = SwerveConstants.ROTATIONAL_AIMING_PID_CONTROLLER;
		Brake = new SwerveRequest.SwerveDriveBrake();

		LedSubsystem.setAllPixels(TeamColor.Blue);

		SwerveLogger = new Telemetry(SwerveConstants.MAX_SPEED);

		// Start swerve telemetry
		SwerveSubsystem.registerTelemetry(SwerveLogger::telemeterize);

		Autonomous = new SideBasedAuto(this);

		SwerveSubsystem.seedFieldRelative(new Pose2d());

		// Configure the button bindings
		configurePrimaryBindings();
		configureSecondaryBindings();
	}

	private void configurePrimaryBindings()
	{
		final Joystick PrimaryController = new Joystick(0);

		final JoystickButton RightTrigger = new JoystickButton(PrimaryController,
				LogitechControllerButtons.triggerRight);
		final JoystickButton LeftTrigger = new JoystickButton(PrimaryController,
				LogitechControllerButtons.triggerLeft);
		final JoystickButton RightBumper = new JoystickButton(PrimaryController,
				LogitechControllerButtons.bumperRight);
		final JoystickButton LeftBumper = new JoystickButton(PrimaryController,
				LogitechControllerButtons.bumperLeft);
		final JoystickButton AButton = new JoystickButton(PrimaryController,
				LogitechControllerButtons.a);
		final JoystickButton BButton = new JoystickButton(PrimaryController,
				LogitechControllerButtons.b);
		final JoystickButton XButton = new JoystickButton(PrimaryController,
				LogitechControllerButtons.x);
		final JoystickButton YButton = new JoystickButton(PrimaryController,
				LogitechControllerButtons.y);

		// Swerve
		SwerveSubsystem.setDefaultCommand(SwerveSubsystem.applyRequest(() -> Drive
				.withVelocityX(-PrimaryController.getY() * SwerveConstants.MAX_SPEED)
				.withVelocityY(-PrimaryController.getX() * SwerveConstants.MAX_SPEED)
				.withRotationalRate(
						-PrimaryController.getTwist() * SwerveConstants.MAX_ANGULAR_SPEED)));

		// Aim to joystick direction
		// Button.whileTrue(SwerveSubsystem.applyRequest(() -> DriveFacingAngle
		// .withVelocityX(-PrimaryController.getY() * SwerveConstants.MAX_SPEED)
		// .withVelocityY(-PrimaryController.getX() * SwerveConstants.MAX_SPEED)
		// .withTargetDirection(new Rotation2d(-PrimaryController.getTwist(),
		// -PrimaryController.getThrottle()))));

		// Aim to 0
		YButton.whileTrue(SwerveSubsystem.applyRequest(() -> DriveFacingAngle
				.withVelocityX(-PrimaryController.getY() * SwerveConstants.MAX_SPEED)
				.withVelocityY(-PrimaryController.getX() * SwerveConstants.MAX_SPEED)
				.withTargetDirection(new Rotation2d(0))));

		// Aim to amp
		XButton.whileTrue(SwerveSubsystem.applyRequest(() -> DriveFacingAngle
				.withVelocityX(-PrimaryController.getY() * SwerveConstants.MAX_SPEED)
				.withVelocityY(-PrimaryController.getX() * SwerveConstants.MAX_SPEED)
				.withTargetDirection(
						new Rotation2d((-Math.PI / 2) + (isRedAlliance() ? 0 : Math.PI)))));

		// Aim at speaker with odometry
		BButton.whileTrue(SwerveSubsystem.applyRequest(() -> DriveFacingAngle
				.withVelocityX(-PrimaryController.getY() * SwerveConstants.MAX_SPEED)
				.withVelocityY(-PrimaryController.getX() * SwerveConstants.MAX_SPEED)
				.withTargetDirection(getRotationToSpeaker())));

		// Reset the field-centric heading
		AButton.onTrue(SwerveSubsystem.runOnce(() -> SwerveSubsystem.seedFieldRelative()));

		// Intake
		LeftBumper.whileTrue(new IntakeCommand(IntakeSubsystem, IndexerSubsystem,
				ShooterMountSubsystem, ShooterSubsystem, LedSubsystem));

		// Shoot from subwoofer
		RightTrigger.whileTrue(new DriverShooterCommand(ShooterSubsystem, IndexerSubsystem,
				ShooterMountSubsystem, LedSubsystem, ShooterConstants.SHOOTER_SPEAKER_VELOCITY,
				ShooterMountConstants.SHOOTER_MOUNT_SPEAKER_ANGLE_FIXED_OFFSET, false));

		// Shoot from distance with pose estimation
		// RightTrigger.whileTrue(new ShootFromDistance(ShooterSubsystem, IndexerSubsystem,
		// 		ShooterMountSubsystem, LedSubsystem));

		// Amp
		LeftTrigger.whileTrue(new AmpCommand(ShooterMountSubsystem, ShooterSubsystem,
				IndexerSubsystem, LedSubsystem));

		// Pass
		RightBumper.whileTrue(new DriverShooterCommand(ShooterSubsystem, IndexerSubsystem,
				ShooterMountSubsystem, LedSubsystem, ShooterConstants.SHOOTER_PASSING_VELOCITY,
				ShooterMountConstants.SHOOTER_MOUNT_PASSING_ANGLE_FIXED_OFFSET, false));
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

		// Climb
		ClimberSubsystem.setDefaultCommand(new ElevatorSpeedCommand(ClimberSubsystem,
				() -> (SecondaryController.getY()), () -> (SecondaryController.getThrottle())));

		// Shoot subwoofer
		// LeftTrigger.whileTrue(new ShooterOverrideCommand(ShooterSubsystem, IndexerSubsystem,
		// LedSubsystem, ShooterConstants.SHOOTER_SPEAKER_VELOCITY, false));
		// LeftTrigger.whileTrue(new RotateShooterMountToPositionCommand(ShooterMountSubsystem,
		// ShooterMountConstants.SHOOTER_MOUNT_SPEAKER_ANGLE_FIXED_OFFSET));
		// LeftTrigger.whileTrue(new AimShooterCommand(ShooterSubsystem, ShooterMountSubsystem,
		// SwerveDrive));

		// Shoot podium
		// LeftBumper.whileTrue(new ShooterOverrideCommand(ShooterSubsystem, IndexerSubsystem,
		// LedSubsystem, ShooterConstants.SHOOTER_SPEAKER_VELOCITY, false));
		// LeftBumper.whileTrue(new RotateShooterMountToPositionCommand(ShooterMountSubsystem,
		// ShooterMountConstants.SHOOTER_MOUNT_PODIUM_ANGLE_FIXED_OFFSET));

		// Passing
		// YButton.whileTrue(new ShooterOverrideCommand(ShooterSubsystem, IndexerSubsystem,
		// LedSubsystem, ShooterConstants.SHOOTER_PASSING_VELOCITY, false));
		// YButton.whileTrue(new RotateShooterMountToPositionCommand(ShooterMountSubsystem,
		// ShooterMountConstants.SHOOTER_MOUNT_PASSING_ANGLE_FIXED_OFFSET));

		// Amp
		// AButton.whileTrue(new AmpCommand(ShooterMountSubsystem, ShooterSubsystem,
		// IndexerSubsystem,
		// LedSubsystem));

		// Outtake
		BButton.whileTrue(
				new IntakeReverseCommand(IntakeSubsystem, IndexerSubsystem, ShooterSubsystem));

		// Intake
		// XButton.whileTrue(new IntakeCommand(IntakeSubsystem, IndexerSubsystem,
		// ShooterMountSubsystem, ShooterSubsystem, LedSubsystem));

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

	public CommandSwerveDrivetrain getSwerve()
	{
		return SwerveSubsystem;
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

	public boolean isRedAlliance()
	{
		return DriverStation.getAlliance().get() == Alliance.Red;
	}

	public static Rotation2d getRotationToSpeaker() {
		Translation2d robotPose = instance.SwerveSubsystem.getPose().getTranslation();

		Optional<Pose3d> speakerPoseOptional = getSpeakerPose();
		Translation2d speakerPose = !speakerPoseOptional.isEmpty() ? new Translation2d(
				speakerPoseOptional.get().getX(), speakerPoseOptional.get().getY())
				: new Translation2d();

		return new Rotation2d(robotPose.getX() - speakerPose.getX(), 
				robotPose.getY() - speakerPose.getY());
	}

	public static double getDistanceToSpeaker() {
		Translation2d robotPose = instance.SwerveSubsystem.getPose().getTranslation();

		Optional<Pose3d> speakerPoseOptional = getSpeakerPose();
		Translation2d speakerPose = !speakerPoseOptional.isEmpty() ? new Translation2d(
				speakerPoseOptional.get().getX(), speakerPoseOptional.get().getY())
				: new Translation2d();

		return robotPose.getDistance(speakerPose);
	}

}