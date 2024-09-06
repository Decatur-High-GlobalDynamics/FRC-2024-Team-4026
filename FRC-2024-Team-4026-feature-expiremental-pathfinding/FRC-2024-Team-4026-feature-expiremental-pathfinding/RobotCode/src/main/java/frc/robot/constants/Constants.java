package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class Constants
{

	public static final int MAX_VOLTAGE = 12;
	public static final int NEO_550_MAX_CURRENT = 25;
	public static final int NEO_MAX_CURRENT = 50;

	public static final AprilTagFieldLayout AprilTagFieldLayout = AprilTagFields.k2024Crescendo
			.loadAprilTagLayoutField();

	public static final String CANIVORE_NAME = "CANivore 0";

}
