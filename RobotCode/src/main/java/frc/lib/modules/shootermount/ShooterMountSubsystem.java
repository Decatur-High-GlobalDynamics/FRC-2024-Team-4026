package frc.lib.modules.shootermount;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import frc.lib.modules.shootermount.ShooterMountConstants;

public class ShooterMountSubsystem extends SubsystemBase
{

	private TalonFX shooterMountMotorLeft, shooterMountMotorRight;

	private MotionMagicDutyCycle motorControlRequest;

	private double shooterMountMinAngle;

	/** Target rotation in rotations */
	private double targetRotation;

	/**
	 * Key: distance to speaker in meters, Value: Rotation of shooter mount motors
	 */
	private InterpolatingDoubleTreeMap shooterMountAngleTreeMap;
	/**
	 * Key: distance to speaker in meters, Value: Note velocity in meters per second
	 */
	private InterpolatingDoubleTreeMap noteFlightTimeEstimateTreeMap;

	private Pigeon2 gyro;

	private double offset;

	public ShooterMountSubsystem()
	{
		gyro = new Pigeon2(Ports.SHOOTER_MOUNT_GYRO);

		shooterMountMotorLeft = new TalonFX(Ports.SHOOTER_MOUNT_MOTOR_LEFT);

		shooterMountMotorRight = new TalonFX(Ports.SHOOTER_MOUNT_MOTOR_RIGHT);

		shooterMountMotorRight.setControl(new Follower(shooterMountMotorLeft.getDeviceID(), true));

		// create configurator
		TalonFXConfiguration mainMotorConfigs = new TalonFXConfiguration();

		mainMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
		mainMotorConfigs.CurrentLimits.StatorCurrentLimit = 60;

		mainMotorConfigs.Feedback.FeedbackRemoteSensorID = gyro.getDeviceID();
		mainMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemotePigeon2_Roll;

		// set pid profiles configs
		Slot0Configs pidSlot0Configs = mainMotorConfigs.Slot0;
		pidSlot0Configs.kP = ShooterMountConstants.SHOOTER_MOUNT_KP;
		pidSlot0Configs.kI = ShooterMountConstants.SHOOTER_MOUNT_KI;
		pidSlot0Configs.kD = ShooterMountConstants.SHOOTER_MOUNT_KD;
		pidSlot0Configs.kS = ShooterMountConstants.SHOOTER_MOUNT_KS;
		pidSlot0Configs.kV = ShooterMountConstants.SHOOTER_MOUNT_KV;
		pidSlot0Configs.kA = ShooterMountConstants.SHOOTER_MOUNT_KA;

		// set motionmagic velocity configs
		MotionMagicConfigs motionMagicVelocityConfigs = mainMotorConfigs.MotionMagic;
		motionMagicVelocityConfigs.MotionMagicCruiseVelocity = ShooterMountConstants.SHOOTER_MOUNT_CRUISE_VELOCITY;
		motionMagicVelocityConfigs.MotionMagicAcceleration = ShooterMountConstants.SHOOTER_MOUNT_ACCELERATION;

		// config the main motor
		shooterMountMotorLeft.getConfigurator().apply(mainMotorConfigs);
		shooterMountMotorRight.getConfigurator().apply(mainMotorConfigs);

		shooterMountMinAngle = 1 / 360;
		targetRotation = 1 / 360;

		// Populate tree maps
		shooterMountAngleTreeMap = new InterpolatingDoubleTreeMap();
		noteFlightTimeEstimateTreeMap = new InterpolatingDoubleTreeMap();
		for (int i = 0; i < ShooterMountConstants.SpeakerDistanceTreeMapKeys.length; i++)
		{
			double key = ShooterMountConstants.SpeakerDistanceTreeMapKeys[i];
			shooterMountAngleTreeMap.put(key,
					ShooterMountConstants.ShooterMountAngleTreeMapValues[i]);
			noteFlightTimeEstimateTreeMap.put(key,
					ShooterMountConstants.NoteFlightTimeEstimateTreeMapValues[i]);
		}

		offset = gyro.getRoll().getValueAsDouble() / 360;

		motorControlRequest = new MotionMagicDutyCycle(offset);
		shooterMountMotorLeft.setControl(motorControlRequest.withPosition(offset));

		RobotContainer.getShuffleboardTab().addDouble("Actual Shooter Mount Rotation",
				() -> (shooterMountMotorLeft.getRotorPosition().getValueAsDouble()));
		RobotContainer.getShuffleboardTab().addDouble("Desired Shooter Mount Rotation",
				() -> targetRotation + offset);
		RobotContainer.getShuffleboardTab().addDouble("Shooter Mount Min",
				() -> shooterMountMinAngle);
		RobotContainer.getShuffleboardTab().addDouble("Shooter Gyro",
				() -> (gyro.getRoll().getValueAsDouble() / 360));
	}

	@Override
	public void periodic()
	{
		// If a motor gets reset this tells to continue sending only what it needs to send the the
		// CAN bus
		if (shooterMountMotorLeft.hasResetOccurred() || shooterMountMotorRight.hasResetOccurred())
		{
			shooterMountMotorLeft.optimizeBusUtilization();
			shooterMountMotorRight.optimizeBusUtilization();
			shooterMountMotorLeft.getRotorPosition().setUpdateFrequency(20);
		}
	}

	/**
	 * Set the desired rotation of the shooter mount
	 * 
	 * @param targetRotation the desired rotation in encoder ticks
	 */
	public void setTargetRotation(double targetRotation)
	{
		System.out.println(targetRotation);

		this.targetRotation = Math.max(
				Math.min(targetRotation, ShooterMountConstants.SHOOTER_MOUNT_MAX_ANGLE_OFFSET),
				shooterMountMinAngle);

		// Has the shooter mount compensate for gravity
		double gravityFeedForward = ShooterMountConstants.SHOOTER_MOUNT_KG
				* Math.cos(ShooterMountConstants.SHOOTER_MOUNT_MIN_ANGLE_IN_RADIANS
						+ Math.toRadians(this.targetRotation * 360));

		shooterMountMotorLeft.setControl(motorControlRequest
				.withPosition(this.targetRotation + offset).withFeedForward(gravityFeedForward));
	}

	/**
	 * @return A map where: Key: distance to speaker in meters, Value: Rotation compensation in
	 *         degrees
	 */
	public InterpolatingDoubleTreeMap getShooterMountAngleTreeMap()
	{
		return shooterMountAngleTreeMap;
	}

	/**
	 * @return A map where: Key: distance to speaker in meters, Value: Note velocity in meters per
	 *         second
	 */
	public InterpolatingDoubleTreeMap getNoteFlightTimeEstimateTreeMap()
	{
		return noteFlightTimeEstimateTreeMap;
	}

	public boolean isAtTargetRotation()
	{
		return Math.abs((gyro.getRoll().getValueAsDouble() / 360)
				- (targetRotation + offset)) < ShooterMountConstants.AIMING_DEADBAND;
	}
}