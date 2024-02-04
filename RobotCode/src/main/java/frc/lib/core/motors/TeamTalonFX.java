package frc.lib.core.motors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.lib.core.PidParameters;
import frc.lib.core.util.TeamUtils;

/**
 * Used for everything that isn't a NEO. A wrapper class for motors that helps to consistently and
 * easily perform the following functions: 1. Keep current and max speeds. 2. Get and reset encoder
 * values. 3. Lots and lots of SmartDashboard information
 */
public class TeamTalonFX extends WPI_TalonFX implements IMotor
{

  public static double telemetryUpdateInterval_secs = 0.0;

  private double lastTelemetryUpdate = 0;

  private final String smartDashboardPrefix;

  private double maxSpeed = Double.MAX_VALUE;

  private final PidParameters[] pidProfiles;

  public static boolean isPidControlMode(final ControlMode mode)
  {
    switch (mode)
    {
    case Current:
    case Disabled:
    case Follower:
      return false;
    default:
      return true;
    }
  }

  public TeamTalonFX(final String smartDashboardPrefix, final int deviceNumber)
  {
    super(deviceNumber);
    this.smartDashboardPrefix = smartDashboardPrefix;

    // assuming quadencoder
    this.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    enableVoltageCompensation(true);

    pidProfiles = new PidParameters[4];
  }

  public void periodic()
  {
    final double now = TeamUtils.getCurrentTime();
	// Renato told me to leave this alone, though we may wanna change it later.

    if ((now - getLastTelemetryUpdate()) < telemetryUpdateInterval_secs)
    {
      return;
    }

    setLastTelemetryUpdate(now);

    final double currentSpeed = getSelectedSensorVelocity(0);

    if (getMaxSpeed() == Double.MAX_VALUE || currentSpeed > getMaxSpeed())
      setMaxSpeed(currentSpeed);
  }

  public long getCurrentEncoderValue()
  {
    // This should be configurable
    return (long) getSensorCollection().getIntegratedSensorPosition();
  }

  public boolean isRunningPidControlMode()
  {
    return isPidControlMode(getControlMode());
  }

  /**
   * @param power  Between -1 and 1
   * @param reason Unused for now
   * @see #set(double)
   */
  public void set(final double power, final String reason)
  {
    super.set(power);
    // Logs.info("Set power to " + power + " REASON: " + reason);
  }

  public void resetEncoder()
  {
    getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public double getLastTelemetryUpdate()
  {
    return lastTelemetryUpdate;
  }

  public void setLastTelemetryUpdate(final double val)
  {
    lastTelemetryUpdate = val;
  }

  public String getSmartDashboardPrefix()
  {
    return smartDashboardPrefix;
  }

  public double getMaxSpeed()
  {
    return maxSpeed;
  }

  public void setMaxSpeed(final double val)
  {
    maxSpeed = val;
  }

  public PidParameters[] getPidProfiles()
  {
    return pidProfiles;
  }

  /**
   * Public wrapper for protected method (which aren't allowed in interfaces) // Use this for
   * configurations which can be shared between SRX and FX // Otherwise down cast and use
   * configAllSettings(TalonFXConfiguration allConfigs) // if using config settings only available
   * for TalonFX
   */
  public ErrorCode configBaseAllSettings(final BaseTalonConfiguration allConfigs)
  {
    return configAllSettings(allConfigs);
  }

  public double getVelocityError()
  {
    if (getControlMode() != ControlMode.Velocity)
    {
      return 0;
    }

    final double currentSpeed = getSelectedSensorVelocity(0);
    return (getClosedLoopTarget(0) - currentSpeed);
  }

  public void configureWithPidParameters(final PidParameters pidParameters, final int pidSlotIndex)
  {
    getPidProfiles()[pidSlotIndex] = pidParameters;

    config_kF(pidSlotIndex, pidParameters.getKF(), 30);
    config_kP(pidSlotIndex, pidParameters.getKP(), 30);
    config_kI(pidSlotIndex, pidParameters.getKI(), 30);
    config_kD(pidSlotIndex, pidParameters.getKD(), 30);
    configPeakOutputForward(pidParameters.getKPeakOutput(), 30);
    configPeakOutputReverse(-pidParameters.getKPeakOutput(), 30);
    configAllowableClosedloopError(pidSlotIndex, pidParameters.getErrorTolerance(), 30);
  }

  @Override
  public void configF(final double F, final int SLOT)
  {
    config_kF(SLOT, F, 30);
  }

  @Override
  public void configP(final double P, final int SLOT)
  {
    config_kP(SLOT, P, 30);
  }

  @Override
  public void configI(final double I, final int SLOT)
  {
    config_kI(SLOT, I, 30);
  }

  @Override
  public void configD(final double D, final int SLOT)
  {
    config_kD(SLOT, D, 30);
  }

  @Override
  public void configPeakOutput(final double PEAK_OUTPUT)
  {
    configPeakOutputForward(PEAK_OUTPUT, 30);
    configPeakOutputReverse(-PEAK_OUTPUT, 30);
  }

  @Override
  public void setClosedLoopErrorLimit(final double ERROR_TOLERANCE, final int SLOT)
  {
    configAllowableClosedloopError(SLOT, (int) ERROR_TOLERANCE, 30);
  }

}