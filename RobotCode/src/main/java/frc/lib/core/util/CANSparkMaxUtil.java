package frc.lib.core.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import frc.lib.modules.swervedrive.SwerveConstants;

/** Sets motor usage for a Spark Max motor controller */
@SuppressWarnings("deprecation")
public class CANSparkMaxUtil
{
	public enum Usage
	{
		kAll, kPositionOnly, kVelocityOnly, kMinimal
	};

	/**
	 * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic
	 * status frame period of nonessential frames from 20ms to 500ms.
	 * <p>
	 * See
	 * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
	 * for a description of the status frames.
	 *
	 * @param motor           The motor to adjust the status frame periods on.
	 * @param usage           The status frame feedack to enable. kAll is the default when a
	 *                        CANSparkMax is constructed.
	 * @param enableFollowing Whether to enable motor following.
	 */
	
	public static void setCANSparkMaxBusUsage(CANSparkMax motor, Usage usage,
			boolean enableFollowing)
	{
		if (enableFollowing)
		{
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
		}
		else
		{
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 500);
		}

		if (usage == Usage.kAll)
		{
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1,
					SwerveConstants.CanSparkMaxPeriodicRates.LOW_INTERVAL);
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2,
					SwerveConstants.CanSparkMaxPeriodicRates.LOW_INTERVAL);
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3,
					SwerveConstants.CanSparkMaxPeriodicRates.MID_INTERVAL);
		}
		else if (usage == Usage.kPositionOnly)
		{
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1,
					SwerveConstants.CanSparkMaxPeriodicRates.HIGH_INTERVAL);
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2,
					SwerveConstants.CanSparkMaxPeriodicRates.LOW_INTERVAL);
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3,
					SwerveConstants.CanSparkMaxPeriodicRates.HIGH_INTERVAL);
		}
		else if (usage == Usage.kVelocityOnly)
		{
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1,
					SwerveConstants.CanSparkMaxPeriodicRates.LOW_INTERVAL);
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2,
					SwerveConstants.CanSparkMaxPeriodicRates.HIGH_INTERVAL);
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3,
					SwerveConstants.CanSparkMaxPeriodicRates.HIGH_INTERVAL);
		}
		else if (usage == Usage.kMinimal)
		{
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1,
					SwerveConstants.CanSparkMaxPeriodicRates.HIGH_INTERVAL);
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2,
					SwerveConstants.CanSparkMaxPeriodicRates.HIGH_INTERVAL);
			motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3,
					SwerveConstants.CanSparkMaxPeriodicRates.HIGH_INTERVAL);
		}
	}

	/**
	 * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic
	 * status frame period of nonessential frames from 20ms to 500ms.
	 * <p>
	 * See
	 * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
	 * for a description of the status frames.
	 *
	 * @param motor The motor to adjust the status frame periods on.
	 * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
	 *              constructed.
	 */
	public static void setCANSparkMaxBusUsage(CANSparkMax motor, Usage usage)
	{
		setCANSparkMaxBusUsage(motor, usage, false);
	}
}