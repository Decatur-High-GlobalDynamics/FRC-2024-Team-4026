package frc.lib.core.motors;

import frc.lib.core.PidParameters;

public interface IMotor
{
    public void configureWithPidParameters(PidParameters pidParams, int slot);

    public boolean isRunningPidControlMode();

    public void configF(final double F, final int SLOT);

    public void configP(final double P, final int SLOT);

    public void configI(final double I, final int SLOT);

    public void configD(final double D, final int SLOT);

    public void configPeakOutput(final double PEAK_OUTPUT);

    public void setClosedLoopErrorLimit(final double ERROR_TOLERANCE, final int SLOT);

    public default void onPidPeriodic(final int SLOT)
    {}

}
