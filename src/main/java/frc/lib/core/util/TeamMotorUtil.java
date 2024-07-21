package frc.lib.core.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class TeamMotorUtil {

    private static final int MAX_PERIODIC_FRAME_PERIOD = 65535;

    public static void optimizeCANSparkBusUsage(CANSparkBase motor) {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAX_PERIODIC_FRAME_PERIOD);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAX_PERIODIC_FRAME_PERIOD);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAX_PERIODIC_FRAME_PERIOD);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, MAX_PERIODIC_FRAME_PERIOD);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, MAX_PERIODIC_FRAME_PERIOD);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, MAX_PERIODIC_FRAME_PERIOD);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, MAX_PERIODIC_FRAME_PERIOD);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, MAX_PERIODIC_FRAME_PERIOD);
    }
}