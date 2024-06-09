package frc.lib.modules.swervedrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.function.Function;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathSegment;

public class SwervePaths
{

	// perfectly fine name
	public static final List<Function<Set<String>, Map<String, List<PathSegment>>>> pathPaths = new ArrayList<>();
	{

	}
}