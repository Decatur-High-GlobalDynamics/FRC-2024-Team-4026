package frc.lib.modules.indexer;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.Ports;

// constructor
public class IndexerSubsystem extends SubsystemBase
{

	private double desiredIndexerVelocity;

	private SparkPIDController indexerPid;
	private CANSparkFlex indexerMotorRight, indexerMotorLeft;
	// beambreak is what we use to detect if there is a note
	private DigitalInput beamBreak;

	// sets up pid and other motor settings for the indexer motors
	public IndexerSubsystem()
	{
		desiredIndexerVelocity = IndexerConstants.INDEXER_REST_VELOCITY;

		beamBreak = new DigitalInput(Ports.BEAM_BREAK);

		indexerMotorRight = new CANSparkFlex(Ports.INDEXER_MOTOR_RIGHT, MotorType.kBrushless);
		indexerMotorLeft = new CANSparkFlex(Ports.INDEXER_MOTOR_LEFT, MotorType.kBrushless);

		indexerMotorLeft.follow(indexerMotorRight, true);

		indexerMotorRight.enableVoltageCompensation(Constants.MAX_VOLTAGE);
		indexerMotorLeft.enableVoltageCompensation(Constants.MAX_VOLTAGE);
		indexerMotorRight.setIdleMode(IdleMode.kBrake);
		indexerMotorLeft.setIdleMode(IdleMode.kBrake);
		indexerMotorRight.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
		indexerMotorLeft.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);

		indexerPid = indexerMotorRight.getPIDController();

		indexerPid.setP(IndexerConstants.INDEXER_KP);
		indexerPid.setI(IndexerConstants.INDEXER_KI);
		indexerPid.setD(IndexerConstants.INDEXER_KD);
		indexerPid.setFF(IndexerConstants.INDEXER_KF);
	}

	// command to set the indexer speed
	public void setIndexerMotorVelocity(double desiredIndexerVelocity, String reason)
	{
		this.desiredIndexerVelocity = Math.max(
				Math.min(IndexerConstants.INDEXER_MAX_VELOCITY, desiredIndexerVelocity),
				-IndexerConstants.INDEXER_MAX_VELOCITY);
	}

	// does the pid for the indexer
	@Override
	public void periodic()
	{
		indexerPid.setReference(desiredIndexerVelocity, ControlType.kVelocity);
	}

	// detects if there is a note
	public boolean hasNote()
	{
		return beamBreak.get();
	}

}
