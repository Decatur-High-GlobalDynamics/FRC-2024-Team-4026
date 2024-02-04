package frc.lib.core.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.core.PidParameters;
import frc.lib.core.util.TeamUtils;

/**
 * Used for NEOs. Provides support for PID control and an encoder.
 */
public class TeamSparkMAX extends CANSparkMax implements IMotor
{

  private static final double TELEMETRY_UPDATE_INTERVAL_SECS = 0.0;
  private double lastTelemetryUpdate = 0;

  private final String SmartDashboardPrefix;

  private double maxSpeed = Double.MAX_VALUE;

  private double smartMotionLoopTarget;

  private final SparkPIDController CanPidController;

  private final RelativeEncoder CanEncoder;

  private final PidParameters PidProfiles[];

  private CANSparkMax.ControlType ctrlType;

  public TeamSparkMAX(final String smartDashboardPrefix, final int deviceID)
  {
    super(deviceID, MotorType.kBrushless); // Neos are brushless

    this.SmartDashboardPrefix = smartDashboardPrefix;

    PidProfiles = new PidParameters[4];
    CanPidController = getPIDController();
    CanEncoder = getEncoder();

    enableVoltageCompensation(12.0);
  }

  public String getSmartDashboardPrefix()
  {
    return SmartDashboardPrefix;
  }

  public SparkPIDController getPidController()
  {
    return CanPidController;
  }

  private static boolean isPidControlMode(final CANSparkMax.ControlType mode)
  {

    // kDutyCycle, kVelocity, kVoltage, kPosition, kSmartMotion, kCurrent, kSmartVelocity

    // Are all possible values. If one of these are not part of PID, add case for them and return
    // false.
    return mode != CANSparkMax.ControlType.kCurrent;
  }

  public double getCurrentEncoderValue()
  {
    // This should be configurable
    return CanEncoder.getPosition();
  }

  public void resetEncoder()
  {
    CanEncoder.setPosition(0.0);
  }

  public boolean isRunningPidControlMode()
  {
    // Dunno if this is safe, but its the easiest way to get around
    // problems with the PidParameters.
    return isPidControlMode(ctrlType);
  }

  public void periodic()
  {
    final double now = TeamUtils.getCurrentTime();
	// Renato told me to leave this alone, though we may wanna change it later.

    if ((now - lastTelemetryUpdate) < TELEMETRY_UPDATE_INTERVAL_SECS)
    {
      return;
    }

    lastTelemetryUpdate = now;

    final double currentSpeed = CanEncoder.getVelocity();

    if (maxSpeed == Double.MAX_VALUE || currentSpeed > maxSpeed)
      maxSpeed = currentSpeed;
  }

  public double getClosedLoopTarget()
  {
    return this.smartMotionLoopTarget;
  }

  public double setClosedLoopTarget(final double value)
  {
    this.smartMotionLoopTarget = value;
    return this.smartMotionLoopTarget;
  }

  public REVLibError setSmartMotionVelocity(final double speed, final String reason)
  {
    setClosedLoopTarget(speed);
    ctrlType = CANSparkMax.ControlType.kSmartVelocity;
    final REVLibError errors = this.CanPidController.setReference(Math.abs(speed),
        CANSparkMax.ControlType.kSmartVelocity);
    return errors;
  }

  public double getVelocityError()
  {
    final double currentSpeed = CanEncoder.getVelocity();
    return getClosedLoopTarget() - currentSpeed;
  }

  public void configureWithPidParameters(final PidParameters pidParameters, final int pidSlotIndex)
  {
    PidProfiles[pidSlotIndex] = pidParameters;

    CanPidController.setFF(pidParameters.getKF(), pidSlotIndex); // Feed-forward
    CanPidController.setP(pidParameters.getKP(), pidSlotIndex);
    CanPidController.setI(pidParameters.getKI(), pidSlotIndex);
    CanPidController.setD(pidParameters.getKD(), pidSlotIndex);
    CanPidController.setOutputRange(-pidParameters.getKPeakOutput(),
        pidParameters.getKPeakOutput());

    CanPidController.setSmartMotionMaxVelocity(pidParameters.getMaxVel(), pidSlotIndex);
    CanPidController.setSmartMotionMinOutputVelocity(0, pidSlotIndex);
    CanPidController.setSmartMotionMaxAccel(pidParameters.getMaxAcc(), pidSlotIndex);
    CanPidController.setSmartMotionAllowedClosedLoopError(pidParameters.getErrorTolerance(),
        pidSlotIndex);
  }

  /**
   * @param power  Between -1 and 1
   * @param reason Unused for now
   * @see #set(double)
   */
  public void set(final double power, final String reason)
  {
    super.set(power);
  }

  @Override
  public void configF(final double F, final int SLOT)
  {
    CanPidController.setFF(F, SLOT);
  }

  @Override
  public void configP(final double P, final int SLOT)
  {
    CanPidController.setP(P, SLOT);
  }

  @Override
  public void configI(final double I, final int SLOT)
  {
    CanPidController.setI(I, SLOT);
  }

  @Override
  public void configD(final double D, final int SLOT)
  {
    CanPidController.setD(D, SLOT);
  }

  @Override
  public void configPeakOutput(final double PEAK_OUTPUT)
  {
    CanPidController.setOutputRange(-PEAK_OUTPUT, PEAK_OUTPUT);
  }

  @Override
  public void setClosedLoopErrorLimit(final double ERROR_TOLERANCE, final int SLOT)
  {
    CanPidController.setSmartMotionAllowedClosedLoopError(ERROR_TOLERANCE, SLOT);
  }

  @Override
  public void onPidPeriodic(final int SLOT)
  {
    final PidParameters PidParams = PidProfiles[SLOT];

    final boolean UPDATE_MOTOR = isRunningPidControlMode();
    final String Prefix = SmartDashboardPrefix + ".PID";
    double maxAcc = PidParams.getMaxAcc(), maxVel = PidParams.getMaxVel(),
        kPeakOutput = PidParams.getKPeakOutput();

    final double new_maxAcc = SmartDashboard.getNumber(Prefix + ".maxAcc", maxAcc);
    if (new_maxAcc != maxAcc)
    {
      maxAcc = new_maxAcc;
      if (UPDATE_MOTOR)
        getPIDController().setSmartMotionMaxAccel(maxAcc, SLOT);
    }

    final double new_maxVel = SmartDashboard.getNumber(Prefix + ".maxAcc", maxVel);
    if (new_maxVel != maxVel)
    {
      maxVel = new_maxVel;
      if (UPDATE_MOTOR)
        getPIDController().setSmartMotionMaxVelocity(maxVel, SLOT);
    }

    SmartDashboard.putNumber(Prefix + ".kPeakOutput", kPeakOutput);
  }

}