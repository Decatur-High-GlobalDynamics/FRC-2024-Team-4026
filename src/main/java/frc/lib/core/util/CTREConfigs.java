package frc.lib.core.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.lib.modules.swervedrive.SwerveConstants;

public final class CTREConfigs
{

	final TalonFXConfiguration swerveAngleFXConfig, swerveDriveFXConfig;
	final CANcoderConfiguration swerveCanCoderConfig;

	public CTREConfigs()
	{
		swerveAngleFXConfig = new TalonFXConfiguration();
		swerveDriveFXConfig = new TalonFXConfiguration();
		swerveCanCoderConfig = new CANcoderConfiguration();
		
		
		/* Swerve Angle Motor Configurations */		
		swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT;
		swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT;
		swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT;
		swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = SwerveConstants.ANGLE_PEAK_CURRENT_DURATION;


		swerveAngleFXConfig.Slot0.kP = SwerveConstants.ANGLE_KP;
		swerveAngleFXConfig.Slot0.kI = SwerveConstants.ANGLE_KI;
		swerveAngleFXConfig.Slot0.kD = SwerveConstants.ANGLE_KD;
		swerveAngleFXConfig.Slot0.kV = SwerveConstants.ANGLE_KF;

		/* Swerve Drive Motor Configuration */		
		swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT;
		swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT;
		swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT;
		swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = SwerveConstants.DRIVE_PEAK_CURRENT_DURATION;
		swerveDriveFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		swerveDriveFXConfig.CurrentLimits.StatorCurrentLimit = 60;

		swerveDriveFXConfig.Slot0.kP = SwerveConstants.DRIVE_KP;
		swerveDriveFXConfig.Slot0.kI = SwerveConstants.DRIVE_KI;
		swerveDriveFXConfig.Slot0.kD = SwerveConstants.DRIVE_KD;
		swerveDriveFXConfig.Slot0.kV = SwerveConstants.DRIVE_KV;
		swerveDriveFXConfig.Slot0.kA = SwerveConstants.DRIVE_KA;
		swerveDriveFXConfig.Slot0.kS = SwerveConstants.DRIVE_KS;
		swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.OPEN_LOOP_RAMP;
		swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.CLOSED_LOOP_RAMP;

		/* Swerve CANCoder Configuration */
		swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
		swerveCanCoderConfig.MagnetSensor.SensorDirection = SwerveConstants.CANCODER_DIRECTION;
	}

	public TalonFXConfiguration getDriveMotorConfigs() {
		return swerveDriveFXConfig;
	}

	public CANcoderConfiguration getAngleEncoderConfigs() {
		return swerveCanCoderConfig;
	}
}