package frc.lib.modules.pathgen;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.modules.pathgen.Constants.PathConstants;
import frc.lib.modules.pathgen.Constants.Paths;
import frc.lib.modules.pathgen.Constants.RobotState;
import frc.lib.modules.pathgen.Constants.timedRobotState;
import frc.lib.modules.pathgen.Constants.FieldConstants;
import frc.lib.modules.pathgen.Constants.FieldConstants.Speaker;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Timer;

// public class Path
// {

// 	public Path(String name, PathConstants pathConstants)
// 	{
// 		 Path deployment = disablePaths() ? Path.of("src", "main", "deploy") : Filesystem.getDeployDirectory().toPath();
// 		 File file = Path.of(deployment.toString(), "pathplanner", name + ".path").toFile();
// 		 try
// 		 {
// 		 InputStream stream = new FileInputStream(file);
// 		 }
// 		 finally
// 		 {
		
// 		 }
// 	}

// 	public double getTimeLength()
// 	{
// 		if (1 == 1)
// 		{
// 			return 0;
// 		}
// 		else
// 		{
// 			return 0;
// 		}
// 	}

// 	public Pose2d getStartingPose()
// 	{
// 		return new Pose2d();

// 	}

// 	public Pose2d[] getPathPoses()
// 	{
// 		Pose2d[] poses = new Pose2d[1];
// 		for (int i = 0; i < poses.length; i++)
// 		{
// 			poses[i] = new Pose2d();
// 		}
// 		return poses;
// 	}

// 	public RobotState[] getRobotState()
// 	{
// 		RobotState[] state = new RobotState[1];
// 		for (int i = 0; i < state.length; i++)
// 		{
// 			state[i] = new RobotState();
// 		}
// 		return state;
// 	}

// 	// pseudo code
// 	public RobotState getRobotStartState()
// 	{
// 		if (this == null)
// 		{
// 			return new RobotState();
// 		}
// 		else
// 		{
// 			return new RobotState();
// 		}
// 	}

// 	// pseudo code
// 	public RobotState getRobotEndState()
// 	{
// 		if (this == null)
// 		{
// 			return new RobotState();
// 		}
// 		else
// 		{
// 			return new RobotState();
// 		}
// 	}

	

// 	 public RobotState sample()
// 	{
// 		 timedRobotState before = null;
// 		 timedRobotState after = null;
// 		 for (timedRobotState state : state.getStates())
// 		 {
// 		 if (state.getTimeLength == 0)
// 		 {
// 		 return state.getStates();
// 		 }

// 		 if (state.getTimeLength <= 0)
// 		 {
// 		 before = state;
// 		 }
// 		 else
// 		 {
// 		 after = state;
// 		 break;
// 		 }
// 		 }
// 	}




// 	public static boolean shouldFlip()
// 	{

// 		return DriverStation.getAlliance().isPresent()
// 				&& DriverStation.getAlliance().get() == Alliance.Red;

// 	}

// 	public static double apply(double xCoordinate)
// 	{
// 		if (shouldFlip())
// 		{
// 			return FieldConstants.fieldLength - xCoordinate;
// 		}
// 		else
// 		{
// 			return xCoordinate;
// 		}
// 	}

// 	public static Translation2d apply(Translation2d translation)
// 	{
// 		if (shouldFlip())
// 		{
// 			return new Translation2d(apply(translation.getX()), translation.getY());
// 		}
// 		else
// 		{
// 			return translation;
// 		}
// 	}

// 	public static Rotation2d apply(Rotation2d rotation)
// 	{
// 		if (shouldFlip())
// 		{
// 			return new Rotation2d(-rotation.getCos(), rotation.getSin());
// 		}
// 		else
// 		{
// 			return rotation;
// 		}
// 	}

// 	public static Pose2d apply(Pose2d pose)
// 	{
// 		if (shouldFlip())
// 		{
// 			return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
// 		}
// 		else
// 		{
// 			return pose;
// 		}
// 	}

// 	public static Translation3d apply(Translation3d translation3d)
// 	{
// 		if (shouldFlip())
// 		{
// 			return new Translation3d(apply(translation3d.getX()), translation3d.getY(),
// 					translation3d.getZ());
// 		}
// 		else
// 		{
// 			return translation3d;
// 		}
// 	}

// 	public static boolean disablePaths()
// 	{

// 		return true;
// 	}

	
	
	

// 	public Object buildPaths(){
// 		return Paths.build();
// 	}

// }
