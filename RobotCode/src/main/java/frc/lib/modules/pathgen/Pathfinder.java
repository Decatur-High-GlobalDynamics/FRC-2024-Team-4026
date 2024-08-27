package frc.lib.modules.pathgen;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.math.RoundingMode;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import frc.lib.core.Autonomous;
import frc.lib.core.util.CTREConfigs;
import frc.lib.modules.pathgen.Constants.timedRobotState;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
import frc.lib.modules.swervedrive.SwervePaths;
import frc.lib.modules.pathgen.Constants.RobotModel;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Pathfinder<PathSegment>
{
	private double pathPaths;

	public Pathfinder()
	{

	}


	RobotModel model = RobotModel.setMass(0).setMoi(0).setRobotLength().setRobotWidth()
			.setWheelRadius(Units.inchesToMeters(1.95225)).setMaxWheelTorque(0.0);
	// SwerveConstants.moduleLimitsFree.@maxDriveVelocity(Units.inchesToMeter(1.95225)*
	// 0.75.build());
	

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

		try
		{
			Files.list(Path.of("src", "main", "deploy", "pathplanner")).forEach((path) ->
			{
				String filename = path.getFileName().toString();
				if (!filename.endsWith(".path"))
					return;
				String[] components = filename.split("\\.");
				if (components.length == 2 && !completedPaths.contains(components[0]))
				{
					path.toFile().delete();
					System.out.println(components[0] + " -paths are kill");
				}
			});
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
		System.out.println("-path is not kill");

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

	public void possiblePaths(SwervePaths swervePaths)
	{
		Map<String, List<PathSegment>> pathQueue = new HashMap<>();

		for (String intakeName : List.of("1", "2", "3"))
		{

			final double minDistance = 0.15;

			Map<String, List<PathSegment>> returnPaths = new HashMap<>();
			var intakeTrajectory = new SwervePaths();
			int index = -1;
			int validIndex = -1;
			Translation2d lastTranslation = new Translation2d();

			;
		}

	}

	public void visionInput(VisionSubsystem vision)
	{
		var apriltag = vision;
	}

	public void targetPoint(final RobotContainer robotContainer, VisionSubsystem vision,
			Autonomous auto)
	{
		var apriltag = vision;
		var Note = vision;
	}

	public static <PathSegment> PathSegment ContinuationPoint(Path path, SwerveDriveSubsystem swerve){
		Pose2d endPose2d = swerve.getPose();
		return PathSegment.build().setStartPose(path.getStartingPose()).setEndPose(endPose2d).build();
	}

	public static <PathSegment> PathSegment AddTranslationPoint(){
		
	}

	public void generateOptimalPath(SwervePaths swervePaths, Map<String, String> pathQueue)
	{
		String generateSwervePath = System.getenv("generateSwervePath");

		for (Map.Entry<String, String> entry : pathQueue.entrySet())
		{
			// Pathpath continues
			Path path;
			double startingTime = System.currentTimeMillis();

			if (generateSwervePath == null)
			{
				path =
				path.buildPaths()
			}
			else
			{
				// targetPoint(newBuilder().setModel(model))
				// .addAllSegments(entry.getValue()).build();

			}

		}

	}

	private static <PathSegment> String getHashCodes(RobotModel model, List<PathSegment> segments)
        {
            StringBuilder hashString = new StringBuilder();
            DecimalFormat format = new DecimalFormat("#.000000");
            format.setRoundingMode(RoundingMode.HALF_DOWN);
            hashString.append(format.format(model.getMass()));
            hashString.append(format.format(model.getMoi()));
            hashString.append(format.format(model.getRobotLength()));
            hashString.append(format.format(model.getRobotWidth()));
            hashString.append(format.format(model.getWheelRadius()));
            hashString.append(format.format(model.getMaxWheelTorque()));
            hashString.append(format.format(model.getMaxWheelOmega()));

			//for (PathSegment segment : segments){
			//	for(Waypoint waypoint : segment.getWaypoints()){
			//		hashString.append(format.format(waypoint.getX()));
			//hashString.append(format.format(waypoint.getY()));
			//	}
			//
		//}
        }

}
