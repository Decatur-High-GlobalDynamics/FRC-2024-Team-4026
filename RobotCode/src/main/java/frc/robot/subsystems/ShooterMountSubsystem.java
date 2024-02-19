package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamTalonFX;
import frc.robot.constants.Ports;
import frc.lib.core.ILogSource;
import frc.robot.constants.ShooterMountConstants;

public class ShooterMountSubsystem extends SubsystemBase implements ILogSource
{

	private TeamTalonFX mainMotor, followMotor;

	/** Target rotation in encoder ticks (4096 encoder ticks to 1 degree) */
	private double targetRotation;

	/** Key: distance to speaker in meters, Value: Rotation compensation in degrees */
	private InterpolatingDoubleTreeMap shooterMountAngleTreeMap;
	/** Key: distance to speaker in meters, Value: Note velocity in meters per second */
	private InterpolatingDoubleTreeMap noteVelocityEstimateTreeMap;

	public ShooterMountSubsystem()
	{
		mainMotor = new TeamTalonFX("SHOOTER_MOUNT_MOTOR_LEFT", Ports.SHOOTER_MOUNT_MOTOR_LEFT);
		followMotor = new TeamTalonFX("SHOOTER_MOUNT_MOTOR_RIGHT", Ports.SHOOTER_MOUNT_MOTOR_RIGHT);

		followMotor.follow(mainMotor);
		followMotor.setInverted(true);

		mainMotor.config_kP(0, ShooterMountConstants.SHOOTER_MOUNT_KP);
		mainMotor.config_kI(0, ShooterMountConstants.SHOOTER_MOUNT_KI);
		mainMotor.config_kD(0, ShooterMountConstants.SHOOTER_MOUNT_KD);
		mainMotor.config_kF(0, ShooterMountConstants.SHOOTER_MOUNT_KF);
		mainMotor.configMotionCruiseVelocity(ShooterMountConstants.SHOOTER_MOUNT_CRUISE_VELOCITY);
		mainMotor.configMotionAcceleration(ShooterMountConstants.SHOOTER_MOUNT_ACCELERATION);
		mainMotor.selectProfileSlot(0, 0);

		targetRotation = 0;

		// Populate tree maps
		shooterMountAngleTreeMap = new InterpolatingDoubleTreeMap();
		noteVelocityEstimateTreeMap = new InterpolatingDoubleTreeMap();
		for (int i = 0; i < ShooterMountConstants.SpeakerDistanceTreeMapKeys.length; i++)
		{
			double key = ShooterMountConstants.SpeakerDistanceTreeMapKeys[i];
			shooterMountAngleTreeMap.put(key,
					ShooterMountConstants.GravityCompensationTreeMapValues[i]);
			noteVelocityEstimateTreeMap.put(key,
					ShooterMountConstants.NoteVelocityEstimateTreeMapValues[i]);
		}
	}

	@Override
	public void periodic()
	{
		mainMotor.set(ControlMode.MotionMagic, targetRotation);
	}

	/**
	 * Set the desired rotation of the shooter mount
	 * 
	 * 
	 * @param targetRotation the desired rotation in degrees
	 */
	public void setTargetRotation(double targetRotation)
	{
		this.targetRotation = degreesToTicks(
				Math.max(targetRotation - ShooterMountConstants.SHOOTER_MOUNT_OFFSET_DEGREES, 0));
	}

	private static double degreesToTicks(double degrees)
	{
		return degrees * ShooterMountConstants.TICKS_IN_ONE_DEGREE;
	}

	public boolean withinAimTolerance()
	{
		return (Math.abs(mainMotor.getCurrentEncoderValue()
				- targetRotation) < ShooterMountConstants.AIMING_DEADBAND ? true : false);

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
	public InterpolatingDoubleTreeMap getNoteVelocityEstimateTreeMap()
	{
		return noteVelocityEstimateTreeMap;
	}

}