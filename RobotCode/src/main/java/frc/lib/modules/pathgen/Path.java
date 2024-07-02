package frc.lib.modules.pathgen;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import frc.lib.modules.pathgen.PathConstants;

public class Path
{

	public Path(String name, PathConstants pathConstants)
	{
		Path deployment = disablePaths() ? Path.of("src", "main", "deploy")
				: Filesystem.getDeployDirectory().toPath();
		File file = Path.of(deployment.toString(), "pathplanner", name + ".path").toFile();
		try
		{
			InputStream stream = new FileInputStream(file);
		}
		finally
		{

		}
	}

	public double getTimeLength()
	{
		if (1 == 1)
		{
			return 0;
		}
		else
		{
			return 0;
		}
	}

	public Pose2d getStartingPose()
	{
		return new Pose2d();

	}

	public Pose2d[] getPathPoses()
	{
		Pose2d[] poses = new Pose2d[1];
		for (int i = 0; i < poses.length; i++)
		{
			poses[i] = new Pose2d();
		}
		return poses;
	}

	public static boolean disablePaths()
	{
		return true;
	}

}
