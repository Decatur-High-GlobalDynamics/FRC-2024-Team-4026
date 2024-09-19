package frc.robot.constants;

import com.pathplanner.lib.path.PathConstraints;

import frc.robot.generated.TunerConstants;

public final class SwerveConstants
{

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps; 

    public static final double MAX_ACCELERATION = 5; // needs testing
    
    public static final double MAX_ANGULAR_SPEED = 1.5 * Math.PI; 

    public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // needs testing but I'm pretty sure this is good

    public static final PathConstraints PathConstraints = new PathConstraints(MAX_SPEED,
            MAX_ACCELERATION, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION);

}
