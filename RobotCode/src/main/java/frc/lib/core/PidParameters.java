package frc.lib.core;

import java.util.Objects;

import javax.annotation.processing.Generated;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.core.motors.*;
import frc.lib.core.util.TeamUtils;

public class PidParameters
{
	private final static double TELEMETRY_UPDATE_INTERVAL_SECS = 0.5;

	private double lastTelemetryUpdate;

	// The k prefix is Hungarian Notation and doesn't match our code standards, but I don't feel
	// like fixing it rn -Renato (Jan 2024)
	private double kP, kI, kD, kF;

	private final double KI_ZONE;

	private double kPeakOutput;

	private double maxAcc;

	private double maxVel;
	private int errorTolerance;

	/** For TalonSRX */
	public PidParameters(final double kP, final double kI, final double kD, final double kF,
			final double kIZone, final double kPeakOutput, final int errorTolerance)
	{
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.KI_ZONE = kIZone;
		this.kPeakOutput = kPeakOutput;
		this.errorTolerance = errorTolerance;
	}

	/** For Spark MAX */
	public PidParameters(final double kP, final double kI, final double kD, final double kF,
			final double kIZone, final double kPeakOutput, final double maxVel, final double maxAcc,
			final int errorTolerance)
	{
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.KI_ZONE = kIZone;
		this.kPeakOutput = kPeakOutput;
		this.errorTolerance = errorTolerance;
		this.maxVel = maxVel;
		this.maxAcc = maxAcc;
	}

	@Override
	public PidParameters clone()
	{
		return new PidParameters(kP, kI, kD, kF, KI_ZONE, kPeakOutput, errorTolerance);
	}

	@Override
	public boolean equals(final Object obj)
	{
		if (obj == null)
			return false;
		if (!(obj instanceof PidParameters))
			return false;

		final PidParameters otherPP = (PidParameters) obj;

		return otherPP.kP == kP && otherPP.kI == kI && otherPP.kD == kD && otherPP.kF == kF
				&& otherPP.KI_ZONE == KI_ZONE && otherPP.kPeakOutput == kPeakOutput
				&& otherPP.errorTolerance == errorTolerance;
	}

	@Override
	public int hashCode()
	{
		return Objects.hash(kP, kI, kD, kF, KI_ZONE, kPeakOutput, errorTolerance);
	}

	/** Use these parameters on a motor */
	public void configureMotorWithPidParameters(final TeamTalonFX motor, final int pidSlotIndex)
	{
		motor.configureWithPidParameters(this, pidSlotIndex);
	}

	public void configureMotorWithPidParameters(final TeamSparkMAX motor, final int pidSlotIndex)
	{
		motor.configureWithPidParameters(this, pidSlotIndex);
	}

	public void periodic(final String prefix, final TeamTalonFX motor, final int pidSlotIndex)
	{
		final double now = TeamUtils.getCurrentTime();

		if ((now - lastTelemetryUpdate) < TELEMETRY_UPDATE_INTERVAL_SECS)
		{
			return;
		}

		lastTelemetryUpdate = now;

		// We update the motor immediately when the the motor is in a PID-controlled mode
		final boolean updateMotor = motor.isRunningPidControlMode();

		final double new_kF = SmartDashboard.getNumber(prefix + ".PID.kF", kF);
		if (new_kF != kF)
		{
			kF = new_kF;
			if (updateMotor)
				motor.config_kF(pidSlotIndex, kF);
		}
		SmartDashboard.putNumber(prefix + ".PID.kF", kF);

		final double new_kP = SmartDashboard.getNumber(prefix + ".PID.kP", kP);
		if (new_kP != kP)
		{
			kP = new_kP;
			if (updateMotor)
				motor.config_kP(pidSlotIndex, kP);
		}
		SmartDashboard.putNumber(prefix + ".PID.kP", kP);

		final double new_kI = SmartDashboard.getNumber(prefix + ".PID.kI", kI);
		if (new_kI != kI)
		{
			kI = new_kI;
			if (updateMotor)
				motor.config_kI(pidSlotIndex, kI);
		}
		SmartDashboard.putNumber(prefix + ".PID.kI", kI);

		final double new_kD = SmartDashboard.getNumber(prefix + ".PID.kD", kD);
		if (new_kD != kD)
		{
			kD = new_kD;
			if (updateMotor)
				motor.config_kD(pidSlotIndex, kD);
		}
		SmartDashboard.putNumber(prefix + ".PID.kD", kD);

		final double new_kPeakOutput = SmartDashboard.getNumber(prefix + ".kPeakOutput",
				kPeakOutput);
		if (new_kPeakOutput != kPeakOutput)
		{
			kPeakOutput = new_kPeakOutput;
			if (updateMotor)
				motor.configPeakOutputForward(kPeakOutput);
			if (updateMotor)
				motor.configPeakOutputReverse(-kPeakOutput);
		}
		SmartDashboard.putNumber(prefix + ".kPeakOutput", kPeakOutput);

		final int new_errorTolerance = (int) SmartDashboard.getNumber(prefix + ".errorTolerance",
				errorTolerance);
		if (new_errorTolerance != errorTolerance)
		{
			errorTolerance = new_errorTolerance;
			if (updateMotor)
				motor.configAllowableClosedloopError(pidSlotIndex, errorTolerance, 30);
		}
		SmartDashboard.putNumber(prefix + ".errorTolerance", errorTolerance);
	}

	public void periodic(final String prefix, final TeamSparkMAX motor, final int pidSlotIndex)
	{
		final double now = TeamUtils.getCurrentTime();

		if ((now - lastTelemetryUpdate) < TELEMETRY_UPDATE_INTERVAL_SECS)
		{
			return;
		}

		lastTelemetryUpdate = now;

		// We update the motor immediately when the the motor is in a PID-controlled mode
		final boolean updateMotor = motor.isRunningPidControlMode();

		final double new_kF = SmartDashboard.getNumber(prefix + ".PID.kF", kF);
		if (new_kF != kF)
		{
			kF = new_kF;
			if (updateMotor)
				motor.getPIDController().setFF(kF, pidSlotIndex);
		}
		SmartDashboard.putNumber(prefix + ".PID.kF", kF);

		final double new_kP = SmartDashboard.getNumber(prefix + ".PID.kP", kP);
		if (new_kP != kP)
		{
			kP = new_kP;
			if (updateMotor)
				motor.getPIDController().setP(kP, pidSlotIndex);
		}
		SmartDashboard.putNumber(prefix + ".PID.kP", kP);

		final double new_kI = SmartDashboard.getNumber(prefix + ".PID.kI", kI);
		if (new_kI != kI)
		{
			kI = new_kI;
			if (updateMotor)
				motor.getPIDController().setI(kI, pidSlotIndex);
		}
		SmartDashboard.putNumber(prefix + ".PID.kI", kI);

		final double new_kD = SmartDashboard.getNumber(prefix + ".PID.kD", kD);
		if (new_kD != kD)
		{
			kD = new_kD;
			if (updateMotor)
				motor.getPIDController().setD(kD, pidSlotIndex);
		}
		SmartDashboard.putNumber(prefix + ".PID.kD", kD);

		final double new_kPeakOutput = SmartDashboard.getNumber(prefix + ".kPeakOutput",
				kPeakOutput);
		if (new_kPeakOutput != kPeakOutput)
		{
			kPeakOutput = new_kPeakOutput;
			if (updateMotor)
				motor.getPIDController().setOutputRange(-kPeakOutput, kPeakOutput);
		}
		SmartDashboard.putNumber(prefix + ".kPeakOutput", kPeakOutput);

		final int new_errorTolerance = (int) SmartDashboard.getNumber(prefix + ".errorTolerance",
				errorTolerance);
		if (new_errorTolerance != errorTolerance)
		{
			errorTolerance = new_errorTolerance;
			if (updateMotor)
				motor.getPIDController().setSmartMotionAllowedClosedLoopError(errorTolerance,
						pidSlotIndex);
		}
		SmartDashboard.putNumber(prefix + ".errorTolerance", errorTolerance);

		final double new_maxAcc = SmartDashboard.getNumber(prefix + ".maxAcc", maxAcc);
		if (new_maxAcc != maxAcc)
		{
			maxAcc = new_maxAcc;
			if (updateMotor)
				motor.getPIDController().setSmartMotionMaxAccel(maxAcc, pidSlotIndex);
		}

		final double new_maxVel = SmartDashboard.getNumber(prefix + ".maxAcc", maxVel);
		if (new_maxVel != maxVel)
		{
			maxVel = new_maxVel;
			if (updateMotor)
				motor.getPIDController().setSmartMotionMaxVelocity(maxVel, pidSlotIndex);
		}

		SmartDashboard.putNumber(prefix + ".kPeakOutput", kPeakOutput);
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public double getKP()
	{
		return kP;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public void setKP(final double kP)
	{
		this.kP = kP;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public double getKI()
	{
		return kI;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public void setKI(final double kI)
	{
		this.kI = kI;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public double getKD()
	{
		return kD;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public void setKD(final double kD)
	{
		this.kD = kD;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public double getKF()
	{
		return kF;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public void setKF(final double kF)
	{
		this.kF = kF;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public int getErrorTolerance()
	{
		return errorTolerance;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public void setErrorTolerance(final int errorTolerance)
	{
		this.errorTolerance = errorTolerance;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public double getKPeakOutput()
	{
		return kPeakOutput;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public void setKPeakOutput(final double kPeakOutput)
	{
		this.kPeakOutput = kPeakOutput;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public double getMaxAcc()
	{
		return maxAcc;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public void setMaxAcc(final double maxAcc)
	{
		this.maxAcc = maxAcc;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public double getMaxVel()
	{
		return maxVel;
	}

	@Generated("Language Support for Java(TM) by Red Hat")
	public void setMaxVel(final double maxVel)
	{
		this.maxVel = maxVel;
	}
}