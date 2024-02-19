package frc.robot.constants;

public class IntakeConstants
{
    /** Ticks per second */
    public static final double INTAKE_REST_VELOCITY = 0;
    /** Ticks per second */
    public static final double INTAKE_DEPLOYED_VELOCITY = 1;

    /** Ticks */
    public static final double INTAKE_RETRACTED_ROTATION = 0;
    /** Ticks */
    public static final double INTAKE_DEPLOYED_ROTATION = 10;

    public static final double INTAKE_DEPLOYMENT_KP = 0.1;
    public static final double INTAKE_DEPLOYMENT_KI = 0;
    public static final double INTAKE_DEPLOYMENT_KD = 0.1;
    public static final double INTAKE_DEPLOYMENT_KFF = 0;

    /** Cruise velocity in ticks */
    public static final double INTAKE_DEPLOYMENT_CRUISE_VELOCITY = 10;
    /** Acceleration in ticks per second */
    public static final double INTAKE_DEPLOYMENT_ACCELERATION = 10;

    public static final double INTAKE_ROLLER_KP = 0.001;
    public static final double INTAKE_ROLLER_KI = 0;
    public static final double INTAKE_ROLLER_KD = 0;
    public static final double INTAKE_ROLLER_KFF = 0.01;
}
