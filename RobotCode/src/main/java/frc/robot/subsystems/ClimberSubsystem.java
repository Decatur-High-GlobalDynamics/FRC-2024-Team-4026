package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamTalonFX;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Ports;

public class ClimberSubsystem extends SubsystemBase
{
	private Pigeon2 gyro;
	private TeamTalonFX extendMotorLeft, extendMotorRight;
	private double targetPosition, targetPositionLeft, targetPositionRight;
	private ProfiledPIDController pidController;
	private boolean override;
	private double leftPower, rightPower;

	public ClimberSubsystem()
	{
		gyro = new Pigeon2(Ports.PIGEON_GYRO);
		// sets extension of left and right motors to given extension length
		extendMotorLeft = new TeamTalonFX("Subsystems.Climber.ExtendRight",
				Ports.CLIMBER_EXTEND_RIGHT_MOTOR);
		extendMotorRight = new TeamTalonFX("Subsystems.Climber.ExtendLeft",
				Ports.CLIMBER_EXTEND_LEFT_MOTOR);
		extendMotorLeft.setNeutralMode(NeutralMode.Brake);
		extendMotorRight.setNeutralMode(NeutralMode.Brake);
		extendMotorLeft.setInverted(true);
		targetPosition = ClimberConstants.MIN_EXTENSION;
		pidController = new ProfiledPIDController(ClimberConstants.CLIMBER_KP,
				ClimberConstants.CLIMBER_KI, ClimberConstants.CLIMBER_KD,
				new TrapezoidProfile.Constraints(ClimberConstants.CLIMBER_VELOCITY,
						ClimberConstants.CLIMBER_ACCELERATION));
		override = false;
	}

	public void periodic()
	{
		if (!override)
		{
			targetPositionLeft = targetPosition;
			targetPositionRight = targetPosition;

			// left arm
			if (gyro.getRoll() > ClimberConstants.DEADBAND_GYRO)
			{
				targetPositionLeft = extendMotorLeft.getCurrentEncoderValue();
			}

			// right arm
			else if (gyro.getRoll() < -ClimberConstants.DEADBAND_GYRO)
			{
				targetPositionRight = extendMotorRight.getCurrentEncoderValue();
			}
			// setting extension of climber arms
			extendMotorLeft.set(pidController.calculate(extendMotorLeft.getCurrentEncoderValue(),
					targetPositionLeft));
			extendMotorRight.set(pidController.calculate(extendMotorRight.getCurrentEncoderValue(),
					targetPositionRight));
		}
		else
		{
			extendMotorLeft.set(leftPower * ClimberConstants.MAX_OVERRIDE_SPEED);
			extendMotorRight.set(rightPower * ClimberConstants.MAX_OVERRIDE_SPEED);
			targetPosition = extendMotorLeft.getCurrentEncoderValue();
			targetPosition = extendMotorRight.getCurrentEncoderValue();
		}
	}

	public void setPowers(double leftPower, double rightPower, String reason)
	{
		this.leftPower = leftPower;
		this.rightPower = rightPower;

	}

	public void setPosition(double position)
	{
		targetPosition = position;
		targetPositionLeft = position;
		targetPositionRight = position;
	}

	// checks if the power level is too high or low for both motors.
	public boolean motorPowerCheck(double power)
	{
		return (extendMotorLeft.getCurrentEncoderValue() > ClimberConstants.MAX_EXTENSION
				&& power >= 0)
				|| (extendMotorLeft.getCurrentEncoderValue() < ClimberConstants.MIN_EXTENSION
						&& power <= 0);
	}

	public void setOverride(boolean override)
	{
		this.override = override;
	}

}