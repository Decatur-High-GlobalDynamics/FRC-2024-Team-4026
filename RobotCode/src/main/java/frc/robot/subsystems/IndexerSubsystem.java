package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.util.TeamMotorUtil;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.Ports;

public class IndexerSubsystem extends SubsystemBase
{

	private double desiredIndexerVelocity;

	private SparkPIDController indexerPid;
	private CANSparkFlex indexerMotorRight, indexerMotorLeft;

	private DigitalInput beamBreak;

	public IndexerSubsystem()
	{
		desiredIndexerVelocity = IndexerConstants.INDEXER_REST_VELOCITY;

		beamBreak = new DigitalInput(Ports.BEAM_BREAK);
		indexerMotorRight = new CANSparkFlex(Ports.INDEXER_MOTOR_RIGHT, MotorType.kBrushless);
		indexerMotorLeft = new CANSparkFlex(Ports.INDEXER_MOTOR_LEFT, MotorType.kBrushless);

		indexerMotorRight.setInverted(true);
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

		RobotContainer.getShuffleboardTab().addDouble("Actual Indexer Velocity",
				() -> indexerMotorRight.getEncoder().getVelocity());
		RobotContainer.getShuffleboardTab().addDouble("Desired Indexer Velocity",
				() -> desiredIndexerVelocity);

		RobotContainer.getShuffleboardTab().addBoolean("Beam Break", () -> beamBreak.get());
	}

	@Override
	public void periodic()
	{
		if (indexerMotorLeft.getStickyFault(FaultID.kHasReset)
				|| indexerMotorRight.getStickyFault(FaultID.kHasReset))
		{
			TeamMotorUtil.optimizeCANSparkBusUsage(indexerMotorRight);
			indexerMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
			indexerMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);

			TeamMotorUtil.optimizeCANSparkBusUsage(indexerMotorLeft);
			indexerMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
		}
	}

	public void setIndexerMotorVelocity(double desiredIndexerVelocity)
	{
		this.desiredIndexerVelocity = Math.max(
				Math.min(IndexerConstants.INDEXER_MAX_VELOCITY, desiredIndexerVelocity),
				-IndexerConstants.INDEXER_MAX_VELOCITY);

		indexerPid.setReference(this.desiredIndexerVelocity, ControlType.kVelocity);
	}

	public boolean hasNote()
	{
		// return !beamBreak.get();

		return false; // For testing without beam break, temporary
	}

}
