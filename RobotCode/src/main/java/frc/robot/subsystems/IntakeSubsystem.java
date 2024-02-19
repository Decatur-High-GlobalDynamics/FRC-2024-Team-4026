package frc.robot.subsystems;

import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.Ports;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamSparkMAX;

public class IntakeSubsystem extends SubsystemBase
{

        private TeamSparkMAX intakeDeployMotorLeft, intakeDeployMotorRight, intakeRollerMotorTop,
                        intakeRollerMotorBottom;
        private double desiredRotation, desiredVelocity;
        private SparkPIDController intakeDeployPidController, intakeRollerPidController;
        private TrapezoidProfile intakeDeployProfile;
        private RelativeEncoder intakeDeployEncoder;

        public IntakeSubsystem()
        {
                intakeDeployMotorRight = new TeamSparkMAX("Intake Deploy Motor Right",
                                Ports.INTAKE_DEPLOY_MOTOR_RIGHT);
                intakeDeployMotorLeft = new TeamSparkMAX("Intake Deploy Motor Left",
                                Ports.INTAKE_DEPLOY_MOTOR_LEFT);
                intakeRollerMotorTop = new TeamSparkMAX("Intake Roller Motor Top",
                                Ports.INTAKE_ROLLER_MOTOR_TOP);
                intakeRollerMotorBottom = new TeamSparkMAX("Intake Roller Motor Bottom",
                                Ports.INTAKE_ROLLER_MOTOR_BOTTOM);

                // Configure deployment motors
                intakeDeployMotorLeft.follow(intakeDeployMotorRight, true);
                intakeDeployMotorRight.setSmartCurrentLimit(Constants.MAX_CURRENT);
                intakeDeployMotorLeft.setSmartCurrentLimit(Constants.MAX_CURRENT);
                intakeDeployMotorRight.setIdleMode(IdleMode.kBrake);
                intakeDeployMotorLeft.setIdleMode(IdleMode.kBrake);
                intakeDeployEncoder = intakeDeployMotorRight.getEncoder();

                // Configure deployment PID
                intakeDeployPidController = intakeDeployMotorRight.getPIDController();
                intakeDeployPidController.setP(IntakeConstants.INTAKE_DEPLOYMENT_KP, 0);
                intakeDeployPidController.setI(IntakeConstants.INTAKE_DEPLOYMENT_KI, 0);
                intakeDeployPidController.setD(IntakeConstants.INTAKE_DEPLOYMENT_KD, 0);
                intakeDeployPidController.setFF(IntakeConstants.INTAKE_DEPLOYMENT_KFF, 0);

                // Configure deployment profile
                intakeDeployProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                                IntakeConstants.INTAKE_DEPLOYMENT_CRUISE_VELOCITY,
                                IntakeConstants.INTAKE_DEPLOYMENT_ACCELERATION));

                // Configure roller motors
                intakeRollerMotorBottom.follow(intakeRollerMotorTop, true);
                intakeRollerMotorTop.setSmartCurrentLimit(Constants.MAX_CURRENT);
                intakeRollerMotorBottom.setSmartCurrentLimit(Constants.MAX_CURRENT);
                intakeRollerMotorTop.setIdleMode(IdleMode.kBrake);
                intakeRollerMotorBottom.setIdleMode(IdleMode.kBrake);

                // Configure roller PID
                intakeRollerPidController = intakeRollerMotorTop.getPIDController();
                intakeRollerPidController.setP(IntakeConstants.INTAKE_ROLLER_KP, 0);
                intakeRollerPidController.setI(IntakeConstants.INTAKE_ROLLER_KI, 0);
                intakeRollerPidController.setD(IntakeConstants.INTAKE_ROLLER_KD, 0);
                intakeRollerPidController.setFF(IntakeConstants.INTAKE_ROLLER_KFF, 0);

                desiredRotation = IntakeConstants.INTAKE_RETRACTED_ROTATION;
                desiredVelocity = IntakeConstants.INTAKE_REST_VELOCITY;
        }

        @Override
        public void periodic()
        {
                TrapezoidProfile.State profiledDesiredRotation = intakeDeployProfile.calculate(0.0,
                                new TrapezoidProfile.State(intakeDeployEncoder.getPosition(),
                                                intakeDeployEncoder.getVelocity()),
                                new TrapezoidProfile.State(desiredRotation, 0.0));
                intakeDeployPidController.setReference(profiledDesiredRotation.position,
                                ControlType.kPosition, 0);
                intakeRollerPidController.setReference(desiredVelocity, ControlType.kVelocity, 0);
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
        }

}
