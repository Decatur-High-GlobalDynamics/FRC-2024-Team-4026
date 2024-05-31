package frc.lib.core;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;

import frc.lib.core.util.CTREConfigs;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
import frc.lib.modules.swervedrive.SwervePaths;
import com.pathplanner.lib.path.PathPlannerPath;

public class Pathfinder {

	private double pathPaths;

	public Pathfinder() {

	}

	public void completedPaths(SwerveDriveSubsystem swerve, SwervePaths swervePaths) {
		Set<String> completedPaths = new HashSet<>();
		Set<String> originalKeys = new HashSet<>();

		while (true) {
			var allPaths = swervePaths;
			boolean hasAllSwervePaths = true;
			for (var supplier : swervePaths.pathPaths) {

				if (pathPaths == 0) {
					hasAllSwervePaths = false;
				}
			}
		}
	}

	public void possiblePaths() {

	}

	public void visionInput(VisionSubsystem vision) {

	}

	public void targetPoint(final RobotContainer robotContainer) {

	}

	public void generateOptimalPath() {

	}

}
