package frc.robot.constants;

import com.pathplanner.lib.path.PathConstraints;

import frc.robot.generated.TunerConstants;

public final class SwerveConstants
{

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps; 

    public static final double MAX_ACCELERATION = 5; // needs testing
    
    public static final double MAX_ANGULAR_SPEED = 1.5 * Math.PI; 

    public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // needs testing but I'm pretty sure this is good

    public static final double ANGLE_KP = 60;
    public static final double ANGLE_KI = 0;
    public static final double ANGLE_KD = 0;
    public static final double ANGLE_KS = 0;
    public static final double ANGLE_KV = 1.5;
    public static final double ANGLE_KA = 0;

    public static final double DRIVE_KP = 0; // 3;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0;
    public static final double DRIVE_KS = 0.12;
    public static final double DRIVE_KV = 0.116;
    public static final double DRIVE_KA = 0;

    public static final double DRIVE_BASE_RADIUS_METERS = 0.33;

    public static final PathConstraints PathConstraints = new PathConstraints(MAX_SPEED,
            MAX_ACCELERATION, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION);

}
