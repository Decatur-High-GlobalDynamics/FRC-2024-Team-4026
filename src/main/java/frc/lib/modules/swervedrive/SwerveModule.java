package frc.lib.modules.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.core.ILogSource;
import frc.lib.core.util.CTREModuleState;
import frc.lib.core.util.Conversions;
import frc.lib.core.util.TeamMotorUtil;
import frc.robot.Robot;
import frc.robot.constants.Constants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

public class SwerveModule implements ILogSource
{

	public final int moduleNumber;

	private Rotation2d angleOffset;
	private Rotation2d lastAngle;

	private CANSparkMax mAngleMotor;
	public TalonFX mDriveMotor;

	private RelativeEncoder integratedAngleEncoder;
	private SparkPIDController angleController;
	private CANcoder angleEncoder;

	private DutyCycleOut openLoopDriveRequest;
	private VelocityDutyCycle closedLoopDriveRequest;

	public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants)
	{
		this.moduleNumber = moduleNumber;
		angleOffset = moduleConstants.AngleOffset;

		/* Angle Encoder Config */
		angleEncoder = new CANcoder(moduleConstants.CANCODER_ID, Constants.CANIVORE_NAME);
		configAngleEncoder();

		/* Angle Motor Config */
		mAngleMotor = new CANSparkMax(moduleConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);
		integratedAngleEncoder = mAngleMotor.getEncoder();
		angleController = mAngleMotor.getPIDController();
		configAngleMotor();

		/* Drive Motor Config */
		mDriveMotor = new TalonFX(moduleConstants.DRIVE_MOTOR_ID, Constants.CANIVORE_NAME);
		configDriveMotor();

		lastAngle = getState().angle;

		openLoopDriveRequest = new DutyCycleOut(0);
		closedLoopDriveRequest = new VelocityDutyCycle(0);
	}

	public void periodic()
	{
		if (angleEncoder.hasResetOccurred() || mDriveMotor.hasResetOccurred()
				|| mAngleMotor.getStickyFault(FaultID.kHasReset))
		{
			angleEncoder.optimizeBusUtilization();
			angleEncoder.getAbsolutePosition().setUpdateFrequency(200);

			mDriveMotor.optimizeBusUtilization();
			mDriveMotor.getRotorPosition().setUpdateFrequency(200);
			mDriveMotor.getRotorVelocity().setUpdateFrequency(200);
			mDriveMotor.getDutyCycle().setUpdateFrequency(200);

			mAngleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10000);
			mAngleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 5);
			mAngleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
			mAngleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000);
			mAngleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000);
			mAngleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10000);
			mAngleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10000);
			mAngleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 10000);
		}
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
			openLoopDriveRequest.Output = percentOutput;
			mDriveMotor.setControl(openLoopDriveRequest);
		}
		else
		{
			double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
					SwerveConstants.WHEEL_CIRCUMFERENCE, SwerveConstants.DRIVE_GEAR_RATIO);
			mDriveMotor.setControl(closedLoopDriveRequest.withVelocity(velocity));
		}
	}

	private void setAngle(SwerveModuleState desiredState)
	{
		// Prevent rotating module if speed is less then 0.1%. Prevents Jittering.
		Rotation2d angle = desiredState.angle;
		if (Math.abs(desiredState.speedMetersPerSecond) <= (0.00001))
		{
			angle = lastAngle;
		}

		angleController.setReference(angle.getDegrees(), CANSparkBase.ControlType.kPosition);
		lastAngle = angle;
	}

	public Rotation2d getCanCoder()
	{
		return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
	}

	private Rotation2d getAngle()
	{
		return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
	}

	public void resetToAbsolute()
	{
		double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
		integratedAngleEncoder.setPosition(absolutePosition);
	}

	private void configAngleEncoder()
	{
		angleEncoder.getConfigurator().apply(Robot.getCtreConfigs().getAngleEncoderConfigs());
	}

	private void configAngleMotor()
	{
		mAngleMotor.restoreFactoryDefaults();
		TeamMotorUtil.optimizeCANSparkBusUsage(mAngleMotor);
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
		mDriveMotor.getConfigurator().apply(Robot.getCtreConfigs().getDriveMotorConfigs());
		mDriveMotor.setInverted(SwerveConstants.DRIVE_MOTOR_INVERT);
		mDriveMotor.setNeutralMode(SwerveConstants.DRIVE_NEUTRAL_MODE);
	}

	public SwerveModuleState getState()
	{
		return new SwerveModuleState(
				Conversions.falconToMPS(mDriveMotor.getRotorPosition().getValueAsDouble(),
						SwerveConstants.WHEEL_CIRCUMFERENCE, SwerveConstants.DRIVE_GEAR_RATIO),
				getAngle());
	}

	public SwerveModulePosition getPosition()
	{
		return new SwerveModulePosition(
				Conversions.falconToMeters(mDriveMotor.getRotorPosition().getValueAsDouble(),
						SwerveConstants.WHEEL_CIRCUMFERENCE, SwerveConstants.DRIVE_GEAR_RATIO),
				getAngle());
	}

	/**
	 * @return motor speed in meters per second
	 */
	public double getDriveMotorSpeed()
	{
		// Rotations per second - Multiplied by 10 to get rotations per second from 100ms period
		double rotations = mDriveMotor.getRotorVelocity().getValueAsDouble()
				/ SwerveConstants.DRIVE_MOTOR_TICKS_PER_ROTATION * 10;

		// Return rotations per second * wheel circumference
		return rotations * SwerveConstants.WHEEL_DIAMETER / Math.PI;
	}
}