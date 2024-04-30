package frc.lib.modules.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;

public class ElevatorSubsystem extends SubsystemBase
{

	private TalonFX climberMotorRight;
	private TalonFX climberMotorLeft;
	private double leftTargetPosition, rightTargetPosition;
	private VelocityDutyCycle motorControlRequestLeftVelocity, motorControlRequestRightVelocity;
	private boolean override;
	private MotionMagicDutyCycle motorControlRequestLeft, motorControlRequestRight;

	public double minimumPositionLeft, minimumPositionRight;

	public ElevatorSubsystem()
	{
		// sets extension of left and right motors to given extension length
		climberMotorLeft = new TalonFX(Ports.CLIMBER_MOTOR_LEFT, Constants.CANIVORE_NAME);
		climberMotorRight = new TalonFX(Ports.CLIMBER_MOTOR_RIGHT, Constants.CANIVORE_NAME);

		climberMotorLeft.setNeutralMode(NeutralModeValue.Brake);
		climberMotorRight.setNeutralMode(NeutralModeValue.Brake);

		leftTargetPosition = ElevatorConstants.LEFT_CLIMBER_MINIMUM;
		rightTargetPosition = ElevatorConstants.RIGHT_CLIMBER_MINIMUM;

		// create configurator
		TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

		// set pid profiles configs
		Slot0Configs pidSlot0Configs = motorConfigs.Slot0;
		pidSlot0Configs.kP = ElevatorConstants.CLIMBER_KP;
		pidSlot0Configs.kI = ElevatorConstants.CLIMBER_KI;
		pidSlot0Configs.kD = ElevatorConstants.CLIMBER_KD;
		pidSlot0Configs.kS = ElevatorConstants.CLIMBER_KS;
		pidSlot0Configs.kV = ElevatorConstants.CLIMBER_KV;
		pidSlot0Configs.kA = ElevatorConstants.CLIMBER_KA;

		// set motionmagic velocity configs
		MotionMagicConfigs motionMagicVelocityConfigs = motorConfigs.MotionMagic;
		motionMagicVelocityConfigs.MotionMagicCruiseVelocity = ElevatorConstants.CLIMBER_CRUISE_VELOCITY;
		motionMagicVelocityConfigs.MotionMagicAcceleration = ElevatorConstants.CLIMBER_ACCELERATION;

		// config the main motor
		climberMotorLeft.getConfigurator().apply(motorConfigs);
		climberMotorRight.getConfigurator().apply(motorConfigs);

		motorControlRequestLeftVelocity = new VelocityDutyCycle(0);
		motorControlRequestRightVelocity = new VelocityDutyCycle(0);

		override = false;

		RobotContainer.getShuffleboardTab().addNumber("L Climber Pos",
				() -> climberMotorLeft.getPosition().getValueAsDouble());
		RobotContainer.getShuffleboardTab().addNumber("R Climber Pos",
				() -> climberMotorRight.getPosition().getValueAsDouble());
		RobotContainer.getShuffleboardTab().addBoolean("Climber Override", () -> override);

		minimumPositionLeft = climberMotorLeft.getPosition().getValueAsDouble();
		minimumPositionRight = climberMotorRight.getPosition().getValueAsDouble();
	}

	@Override
	public void periodic()
	{
		climberMotorLeft.optimizeBusUtilization();
		climberMotorRight.optimizeBusUtilization();

		climberMotorLeft.getRotorPosition().setUpdateFrequency(20);
		climberMotorRight.getRotorPosition().setUpdateFrequency(20);
	}

	public void setLeftTargetPosition(double LeftPosition)
	{
		leftTargetPosition = LeftPosition;

		climberMotorLeft.setControl(motorControlRequestLeft.withPosition(leftTargetPosition));
	}

	public void setRightTargetPosition(double RightPosition)
	{
		rightTargetPosition = RightPosition;

		climberMotorRight.setControl(motorControlRequestRight.withPosition(rightTargetPosition));
	}

	public void setLeftVelocity(double velocity)
	{
		climberMotorLeft.setControl(motorControlRequestLeftVelocity.withVelocity(velocity));
	}

	public void setRightVelocity(double velocity)
	{
		climberMotorRight.setControl(motorControlRequestRightVelocity.withVelocity(velocity));
	}

	public void setBoolean(boolean override)
	{
		this.override = override;
	}

	public void setOverride(boolean override)
	{
		this.override = override;
	}
	

}