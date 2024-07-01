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
		// Path deployment = PathConstants.disablePaths() ? Path.of("src", "main", "deploy") :
		// Filesystem.getDeployDirectory().toPath();
		// File file = Path.of(deployment.toString(), "pathplanner", name + ".path").toFile();
		// try{
		// InputStream stream = new FileInputStream(file);
		// }
	}

	// public double getTimeLength(){
	// if(){
	//
	// }
	// else{
	// return 0;
	// }
	// }

	public Pose2d getStartingPose()
	{
		return new Pose2d();

	}

	public static void disablePaths()
	{

	}
}
