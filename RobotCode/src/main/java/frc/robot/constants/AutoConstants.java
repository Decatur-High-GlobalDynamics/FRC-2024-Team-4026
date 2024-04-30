package frc.robot.constants;

import frc.lib.core.modules.shootermount.RotateShooterMountToPositionCommand;

public final class AutoConstants
{

    /**
     * The shooter mount rotations (in degrees) to be registered as named commands for PathPlanner.
     * Passed to {@link RotateShooterMountToPositionCommand}.
     */
    public static final double[] AutoShooterMountRotations =
    {
            20, 40, 60
    };

	public static final double CHASSIS_ROTATION_NOTE_CENTER = 0;
	public static final double CHASSIS_ROTATION_NOTE_SOURCE = -0.5;
	public static final double CHASSIS_ROTATION_NOTE_AMP = 0.5;

}
