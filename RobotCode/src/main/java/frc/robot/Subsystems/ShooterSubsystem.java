package frc.robot.Subsystems;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamSparkMAX;

public class ShooterSubsystem extends SubsystemBase
{
	// creates objects
	private double shooterMotorPower;
	private double feederMotorPower;

	private PIDController shooterPID;
	private PIDController feederPID;
	public TeamSparkMAX shooterMotorMain, shooterMotorSub, feederMotorMain, feederMotorSub;

	private static double voltage = 12;

	public ShooterSubsystem()
	{
		//sets default shooter motor power to o.25 and feeder to 0
		shooterMotorPower = 0.25;
		feederMotorPower = 0;
		//initializes pid
		shooterPID = new PIDController(Constants.SHOOTER_KP, Constants.SHOOTER_KI,
				Constants.SHOOTER_KD);
		feederPID = new PIDController(Constants.FEEDER_KP, Constants.FEEDER_KI,
				Constants.FEEDER_KD);
		//initializes motor object
		shooterMotorMain = new TeamSparkMAX("Left Shooter Motor Main", Ports.SHOOTER_MOTOR_MAIN);
		shooterMotorSub = new TeamSparkMAX("Right Shooter Motor Main", Ports.SHOOTER_MOTOR_SUB);
		feederMotorMain = new TeamSparkMAX("Left Shooter Motor Sub", Ports.FEEDER_MOTOR_MAIN);
		feederMotorSub = new TeamSparkMAX("Right Shooter Motor Sub", Ports.FEEDER_MOTOR_SUB);
		//the code is able to enable the unable to be able voltage compensation
		shooterMotorMain.enableVoltageCompensation(voltage);
		shooterMotorSub.enableVoltageCompensation(voltage);
		feederMotorMain.enableVoltageCompensation(voltage);
		feederMotorSub.enableVoltageCompensation(voltage);
		//makes sub motors follow main
		feederMotorSub.follow(feederMotorMain);
		shooterMotorSub.follow(shooterMotorMain);
		//  inverts the right side
		shooterMotorSub.setInverted(true);
		feederMotorSub.setInverted(true);
		//  sets neutral mode for the motors
		shooterMotorSub.setIdleMode(IdleMode.kBrake);
		feederMotorSub.setIdleMode(IdleMode.kBrake);
		shooterMotorMain.setIdleMode(IdleMode.kBrake);
		feederMotorMain.setIdleMode(IdleMode.kBrake);
	}
	//gets the shooter power
	public double getShooterMotorPower()
	{
		return shooterMotorMain.get();
	}
	// sets shooter motor power variable based on mathy stuff owen told me to do
	public void setShooterMotorPower(double power, String reason)
	{
		shooterMotorPower = Math.max(Math.min(1, power), -1);

	}
	// sets feeder motor power variable with some mathy stuff owen knows
	public void setFeedMotorPower(double power, String reason)
	{
		feederMotorPower = Math.max(Math.min(1, power), -1);
	}
	//continuously updates shooter speed based on the commands above
	public void periodic()
	{
		double newShooterPower = shooterPID.calculate(shooterMotorSub.get(), shooterMotorPower);
		shooterMotorMain.set(newShooterPower);

		double newFeederPower = feederPID.calculate(feederMotorSub.get(), feederMotorPower);
		feederMotorMain.set(newFeederPower);
	}
}
