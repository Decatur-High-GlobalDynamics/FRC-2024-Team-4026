package frc.lib.modules.intake;

import frc.lib.core.util.TeamMotorUtil;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.lib.modules.intake.IntakeConstants;
import frc.robot.constants.Ports;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase
{

	private CANSparkMax intakeDeployMotorLeft, intakeDeployMotorRight, intakeRollerMotor;
	private double desiredRotation, desiredVelocity;
	private SparkPIDController intakeRollerPidController;

	private Encoder encoder;

	private ProfiledPIDController intakeController;
	private ArmFeedforward intakeFeedforward;

	public IntakeSubsystem()
	{
		intakeDeployMotorRight = new CANSparkMax(Ports.INTAKE_DEPLOY_MOTOR_RIGHT,
				MotorType.kBrushless);
		intakeDeployMotorLeft = new CANSparkMax(Ports.INTAKE_DEPLOY_MOTOR_LEFT,
				MotorType.kBrushless);
		intakeRollerMotor = new CANSparkMax(Ports.INTAKE_ROLLER_MOTOR, MotorType.kBrushless);

		encoder = new Encoder(6, 7);
		
		// Configure deployment motors
		intakeDeployMotorLeft.follow(intakeDeployMotorRight, true);
		intakeDeployMotorRight.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
		intakeDeployMotorLeft.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
		intakeDeployMotorRight.setIdleMode(IdleMode.kBrake);
		intakeDeployMotorLeft.setIdleMode(IdleMode.kBrake);

		intakeController = new ProfiledPIDController(IntakeConstants.INTAKE_RETRACT_KP, 
				IntakeConstants.INTAKE_RETRACT_KI, IntakeConstants.INTAKE_RETRACT_KD,
				new TrapezoidProfile.Constraints(IntakeConstants.INTAKE_RETRACT_CRUISE_VELOCITY,
						IntakeConstants.INTAKE_RETRACT_MAX_ACCELERATION));
		// configuring feedforward
		intakeFeedforward = new ArmFeedforward(0, IntakeConstants.INTAKE_RETRACT_KG, IntakeConstants.INTAKE_RETRACT_KV);
		
				// Configure roller motors
		intakeRollerMotor.setInverted(true);
		intakeRollerMotor.setSmartCurrentLimit(30);
		intakeRollerMotor.setIdleMode(IdleMode.kBrake);

		// Configure roller PID
		intakeRollerPidController = intakeRollerMotor.getPIDController();
		intakeRollerPidController.setP(IntakeConstants.INTAKE_ROLLER_KP, 0);
		intakeRollerPidController.setI(IntakeConstants.INTAKE_ROLLER_KI, 0);
		intakeRollerPidController.setD(IntakeConstants.INTAKE_ROLLER_KD, 0);
		intakeRollerPidController.setFF(IntakeConstants.INTAKE_ROLLER_KFF, 0);

		desiredRotation = IntakeConstants.INTAKE_RETRACTED_ROTATION;
		desiredVelocity = IntakeConstants.INTAKE_REST_VELOCITY;
		//shuffleboard is basically logging software for status and stuff
		RobotContainer.getShuffleboardTab().addDouble("Actual Intake Velocity",
				() -> intakeRollerMotor.getEncoder().getVelocity());
		RobotContainer.getShuffleboardTab().addDouble("Desired Intake Velocity",
				() -> desiredVelocity);
		RobotContainer.getShuffleboardTab().addDouble("Actual Intake Rotation",
				() -> encoder.getRaw());
		RobotContainer.getShuffleboardTab().addDouble("Desired Intake Rotation",
				() -> desiredRotation);
	}

	@Override
	public void periodic()
	{
		//sticky faults are set to true when a motor has a fault.
		if (intakeDeployMotorLeft.getStickyFault(FaultID.kHasReset)
				|| intakeDeployMotorRight.getStickyFault(FaultID.kHasReset)
				|| intakeRollerMotor.getStickyFault(FaultID.kHasReset))
		{
			TeamMotorUtil.optimizeCANSparkBusUsage(intakeRollerMotor);
			intakeRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
			intakeRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);

			TeamMotorUtil.optimizeCANSparkBusUsage(intakeDeployMotorLeft);
			intakeDeployMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
			intakeDeployMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);

			TeamMotorUtil.optimizeCANSparkBusUsage(intakeDeployMotorRight);
			intakeDeployMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
			intakeDeployMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
		}

		intakeDeployMotorRight.set(intakeController.calculate(encoder.getRaw(), desiredRotation) + 
				intakeFeedforward.calculate(encoderToRadians(), intakeController.getSetpoint().velocity));
	}

	// 90 degrees -> 180 ticks
	// 0 degrees -> 2128 ticks
	public double encoderToRadians()
	{
		return (Math.PI / 2) - ((2 * Math.PI) * ((desiredRotation - 180) / 8192));
	}

	/** @param desiredRotation Ticks */
	public void setDesiredRotation(double desiredRotation)
	{
		this.desiredRotation = desiredRotation;
	}

	/** @param desiredVelocity Ticks per second */
	public void setDesiredVelocity(double desiredVelocity)
	{
		this.desiredVelocity = desiredVelocity;
		intakeRollerPidController.setReference(desiredVelocity, ControlType.kVelocity, 0);
		// if (desiredVelocity == 0 /*|| intakeDeployEncoderRight.getPosition() < IntakeConstants.INTAKE_SPIN_ROTATION*/);
		// {
		// 	intakeRollerMotor.set(0);
		// }
	}

}
