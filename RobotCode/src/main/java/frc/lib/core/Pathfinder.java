package frc.lib.core;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import frc.lib.core.util.CTREConfigs;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
import frc.lib.modules.swervedrive.SwervePaths;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathSegment;

public class Pathfinder
{

	private double pathPaths;

	public Pathfinder()
	{

	}

	public void completedPaths(SwerveDriveSubsystem swerve, SwervePaths swervePaths)
	{
		Set<String> completedPaths = new HashSet<>();
		Set<String> originalKeys = new HashSet<>();

		for (String name : originalKeys)
		{
			if (completedPaths.contains(name))
			{
				completedPaths.remove(name);
			}
			else
			{
				completedPaths.add(name);
			}
		}

		while (true)
		{
			var allSwervePaths = swervePaths;
			boolean hasAllSwervePaths = true;
			for (var supplier : swervePaths.pathPaths)
			{

				if (pathPaths == 0)
				{
					hasAllSwervePaths = false;
				}
			}
		}
	}

	public void possiblePaths()
	{
		Map<String, List<PathSegment>> pathQueue = new HashMap<>();

	}

	public void visionInput(VisionSubsystem vision)
	{
		var apriltag = vision;
	}

	public void targetPoint(final RobotContainer robotContainer)
	{

	}

	public void generateOptimalPath()
	{

	}

}
