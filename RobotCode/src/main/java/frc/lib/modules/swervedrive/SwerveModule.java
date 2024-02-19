package frc.lib.modules.swervedrive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.core.motors.TeamSparkMAX;
import frc.lib.core.util.CANCoderUtil;
import frc.lib.core.util.CANSparkMaxUtil;
import frc.lib.core.util.CTREModuleState;
import frc.lib.core.util.Conversions;
import frc.lib.core.util.CANCoderUtil.CCUsage;
import frc.lib.core.util.CANSparkMaxUtil.Usage;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;

public class SwerveModule
{

	public final int moduleNumber;

	private Rotation2d angleOffset;
	private Rotation2d lastAngle;

	private CANSparkMax mAngleMotor;
	private TalonFX mDriveMotor;

	private RelativeEncoder integratedAngleEncoder;
	private SparkPIDController angleController;
	private CANCoder angleEncoder;

	SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.DRIVE_KS,
			SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA);

	public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants)
	{
		this.moduleNumber = moduleNumber;
		angleOffset = moduleConstants.AngleOffset;

		/* Angle Encoder Config */
		angleEncoder = new CANCoder(moduleConstants.CANCODER_ID);
		configAngleEncoder();

		/* Angle Motor Config */
		mAngleMotor = new TeamSparkMAX("AngleMotor", moduleConstants.ANGLE_MOTOR_ID);
		integratedAngleEncoder = mAngleMotor.getEncoder();
		angleController = mAngleMotor.getPIDController();
		configAngleMotor();

		/* Drive Motor Config */
		mDriveMotor = new TalonFX(moduleConstants.DRIVE_MOTOR_ID);
		configDriveMotor();

		lastAngle = getState().angle;
	}

	public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
	{
		/*
		 * This is a custom optimize function, since default WPILib optimize assumes continuous
		 * controller which CTRE and Rev onboard is not
		 */
		desiredState = CTREModuleState.optimize(desiredState, getState().angle);
		setAngle(desiredState);
		setSpeed(desiredState, isOpenLoop);
	}

	private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
	{
		// if isOpenLoop is false, we convert to a Falcon unit. If true, we set the motor speed
		// using a PercentOutput of motor power
		if (isOpenLoop)
		{
			double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
			mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
		}
		else
		{
			double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
					SwerveConstants.WHEEL_CIRCUMFERENCE, SwerveConstants.DRIVE_GEAR_RATIO);
			mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
					feedforward.calculate(desiredState.speedMetersPerSecond));
		}
	}

	private void setAngle(SwerveModuleState desiredState)
	{
		Rotation2d angle = (Math
				.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.MAX_SPEED * 0.01))
						? lastAngle
						: desiredState.angle; // Prevent rotating module if speed is less then 1%.
												// Prevents Jittering.

		angleController.setReference(angle.getDegrees(), CANSparkBase.ControlType.kPosition);
		lastAngle = angle;
	}

	private Rotation2d getAngle()
	{
		return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
	}

	public Rotation2d getCanCoder()
	{
		return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
	}

	public void resetToAbsolute()
	{
		double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
		integratedAngleEncoder.setPosition(absolutePosition);
	}

	private void configAngleEncoder()
	{
		angleEncoder.configFactoryDefault();
		CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kAll);
		angleEncoder.configAllSettings(Robot.getCtreConfigs().swerveCanCoderConfig);
	}

	private void configAngleMotor()
	{
		mAngleMotor.restoreFactoryDefaults();
		CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kMinimal);
		mAngleMotor.setSmartCurrentLimit(SwerveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT);
		mAngleMotor.setInverted(SwerveConstants.ANGLE_MOTOR_INVERT);
		mAngleMotor.setIdleMode(SwerveConstants.ANGLE_NEUTRAL_MODE);
		integratedAngleEncoder.setPositionConversionFactor(SwerveConstants.ANGLE_CONVERSION_FACTOR);
		angleController.setP(SwerveConstants.ANGLE_KP);
		angleController.setI(SwerveConstants.ANGLE_KI);
		angleController.setD(SwerveConstants.ANGLE_KD);
		angleController.setFF(SwerveConstants.ANGLE_KF);
		mAngleMotor.enableVoltageCompensation(SwerveConstants.VOLTAGE_COMP_TARGET);
		mAngleMotor.burnFlash(); // writes configurations to flash memory so they save if a PDP
									// breaker trips
		resetToAbsolute();
	}

	private void configDriveMotor()
	{
		mDriveMotor.configFactoryDefault();
		mDriveMotor.configAllSettings(Robot.getCtreConfigs().swerveDriveFXConfig);
		mDriveMotor.setInverted(SwerveConstants.DRIVE_MOTOR_INVERT);
		mDriveMotor.setNeutralMode(SwerveConstants.DRIVE_NEUTRAL_MODE);
		mDriveMotor.setSelectedSensorPosition(0);
	}

	public SwerveModuleState getState()
	{
		return new SwerveModuleState(
				Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(),
						SwerveConstants.WHEEL_CIRCUMFERENCE, SwerveConstants.DRIVE_GEAR_RATIO),
				getAngle());
	}

	public SwerveModulePosition getPosition()
	{
		return new SwerveModulePosition(
				Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(),
						SwerveConstants.WHEEL_CIRCUMFERENCE, SwerveConstants.DRIVE_GEAR_RATIO),
				getAngle());
	}

	/**
	 * @return motor speed in meters per second
	 */
	public double getDriveMotorSpeed()
	{
		// Rotations per second - Multiplied by 10 to get rotations per second from 100ms period
		double rotations = mDriveMotor.getSelectedSensorVelocity()
				/ SwerveConstants.DRIVE_MOTOR_TICKS_PER_ROTATION * 10;

		// Return rotations per second * wheel circumference
		return rotations * SwerveConstants.WHEEL_DIAMETER / Math.PI;
	}
}