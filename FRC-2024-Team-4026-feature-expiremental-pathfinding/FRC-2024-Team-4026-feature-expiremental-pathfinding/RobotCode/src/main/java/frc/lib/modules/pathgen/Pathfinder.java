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
import java.util.Optional;
import java.util.Set;

import frc.lib.core.Autonomous;
import frc.lib.core.util.CTREConfigs;
import frc.lib.modules.pathgen.Constants.timedRobotState;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
import frc.lib.modules.swervedrive.SwervePaths;
import frc.lib.modules.pathgen.Constants.RobotModel;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathSegment;
import frc.lib.modules.pathgen.Setpoint;
import frc.lib.modules.pathgen.ModuleLimits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Pathfinder{
	private double pathPaths;

	private final SwerveDriveKinematics kinematics;
	private final Translation2d[] moduleLocations;

	public Pathfinder()
	{

	}


	RobotModel model = RobotModel.setMass(0).setMoi(0).setRobotLength().setRobotWidth()
			.setWheelRadius(Units.inchesToMeters(1.95225)).setMaxWheelTorque(0.0);
	// SwerveConstants.moduleLimitsFree.@maxDriveVelocity(Units.inchesToMeter(1.95225)*
	// 0.75.build());

	private double unwrapAngle(double ref, double angle) {
		double diff = angle - ref;
		if (diff > Math.PI) {
		  return angle - 2.0 * Math.PI;
		} else if (diff < -Math.PI) {
		  return angle + 2.0 * Math.PI;
		} else {
		  return angle;
		}
	  }
	
	private double findRoot(
      Function2d func,
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      int iterations_left) {
    if (iterations_left < 0 || epsilonEquals(f_0, f_1)) {
      return 1.0;
    }
    var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
    var x_guess = (x_1 - x_0) * s_guess + x_0;
    var y_guess = (y_1 - y_0) * s_guess + y_0;
    var f_guess = func.f(x_guess, y_guess);
    if (Math.signum(f_0) == Math.signum(f_guess)) {
      // 0 and guess on same side of root, so use upper bracket.
      return s_guess
          + (1.0 - s_guess)
              * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
    } else {
      // Use lower bracket.
      return s_guess
          * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
    }
  }

  protected double findSteeringMaxS(
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      double max_deviation,
      int max_iterations) {
    f_1 = unwrapAngle(f_0, f_1);
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_deviation) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_deviation;
    Function2d func =
        (x, y) -> {
          return unwrapAngle(f_0, Math.atan2(y, x)) - offset;
        };
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
  }

  protected double findDriveMaxS(
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      double max_vel_step,
      int max_iterations) {
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_vel_step) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_vel_step;
    Function2d func =
        (x, y) -> {
          return Math.hypot(x, y) - offset;
        };
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
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

	//probably defunct
	public void visionInput(VisionSubsystem vision)
	{
		var apriltag = vision;
	}

	
	private boolean flipHeading(Rotation2d prevToGoal) {
		return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
	}

	private interface Function2d {
		double f(double x, double y);
	  }

	//Generates to target point for a path to generated to
	public Setpoint TargetPoint(final RobotContainer robotContainer, VisionSubsystem vision,
			Autonomous auto, final ModuleLimits limits, double dt){
		final Translation2d[] modules = moduleLocations;

		Setpoint previousSetpoint;

		SwerveModuleState[] desiredModuleStates = kinematics.toSwerveModuleStates(desiredState);

		if(limits.maxDriveVelocity > 0.0) {
			SwerveDriveKinematics.desaturateWheelSpeeds(desiredSwerveModuleStates, limits.maxDriveVelocity());
			//add driving thingy
			desiredState = kinematics.toChassisSpeeds(desiredModuleStates);
			}
		

		if (desiredState.toTwist2d()){
				//add driving thingy, but false
				for(int i = 0; i < modules.length; ++i){
					desiredModuleStates[i].angle = previousSetpoint.moduleStates()[i].angle;
					desiredModuleStates[i].speedMetersPerSecond = 0.0;
		}
	}
		double[] prev_vx = new double[modules.length];
   		double[] prev_vy = new double[modules.length];
    	Rotation2d[] prev_heading = new Rotation2d[modules.length];
    	double[] desired_vx = new double[modules.length];
    	double[] desired_vy = new double[modules.length];
    	Rotation2d[] desired_heading = new Rotation2d[modules.length];
		boolean all_modules_should_flip = true;
		for(int i = 0; i < modules.length; i++){
			prev_vx[i] = previousSetpoint.moduleStates()[i].angle.getCos()*previousSetpoint.moduleStates()[i].speedMetersPerSecond;
			prev_vy[i] = previousSetpoint.moduleStates()[i].angle.getSin()*previousSetpoint.moduleStates()[i].speedMetersPerSecond;
			prev_heading[i] = prevSetpoint.moduleStates()[i].angle;
			if (prevSetpoint.moduleStates()[i].speedMetersPerSecond < 0.0) {
				prev_heading[i] = prev_heading[i].rotateBy(Rotation2d.fromRadians(Math.PI));
			  }

			desired_vx[i] = desiredModuleStates[i].angle.getCos() * desiredModuleStates[i].speedMetersPerSecond;
    		desired_vy[i] = desiredModuleStates[i].angle.getSin() * desiredModuleStates[i].speedMetersPerSecond;
			desired_heading[i] = desiredModuleStates[i].angle;

			if(desiredModuleStates[i].speedMetersPerSecond < 0.0){
				desired_heading[i] = desired_heading[i].rotateBy(Rotation2d.fromRadians(Math.PI));
			}

			if(all_modules_should_flip){
				double required_rotation_rad = Math.abs(prev_heading[i].unaryMinus().rotateBy(desired_heading[i]).getRadians());
				if(required_rotation_rad < Math.PI / 2.0){
					all_modules_should_flip = false;
				}
			}
			
			if (all_modules_should_flip && !prevSetpoint.chassisSpeeds().toTwist2d().epsilonEquals(new Twist2d()) && !desiredState.toTwist2d().epsilonEquals(new Twist2d())) {
				return generateSetpoint(limits, prevSetpoint, new ChassisSpeeds(), dt);
			}
		}
			
			//Does all the Deltas between the start and end point
			double dx = desiredState.vxMetersPerSecond - prevSetpoint.chassisSpeeds().vxMetersPerSecond;
			double dy = desiredState.vyMetersPerSecond - prevSetpoint.chassisSpeeds().vyMetersPerSecond;
			double dtheta = desiredState.omegaRadiansPerSecond - prevSetpoint.chassisSpeeds().omegaRadiansPerSecond;


			double min_s = 1.0;

			//When an individual module stops
			List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(modules.length);

			final double max_theta_step = dt * limits.maxSteeringVelocity();

   			for (i = 0; i < modules.length; ++i) {
				//add drive
				overrideSteering.add(Optional.empty());
				if(epsilonEquals(previousSetpoint.moduleStates()[i].speedMetersPerSecond, 0.0)){
					if(epsilonEquals(desiredModuleStates[i].speedMetersPerSecond, 0.0)){
						overrideSteering.set(i, Optional.of(previousSetpoint.moduleStates()[i].angle));
						continue;
					}
				}
			

			var necessaryRotation =
				previousSetpoint.moduleStates()[i].angle.unaryMinus().rotateBy(desiredModuleStates[i].angle);

			if(flipHeading(necessaryRotation)){
				necessaryRotation = necessaryRotation.rotateBy(Rotation2d.fromRadians(Math.PI));
			}

			final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / max_theta_step;

			if(numStepsNeeded <= 0.0){
				overrideSteering.set(i, Optional.of(desiredModuleStates[i].angle));
				continue;
			}
			else{
				overrideSteering.set(i, Optional.of(previousSetpoint.moduleStates)[i].angle.rotateBy(Rotation2d.fromRadians(Math.signum(necessaryRotation.getRadians))*max_theta_step));
				min_s = 0.0;
				continue;
			}
		}
		if(min_s == 0.0){
			continue;
		}

		final int kMaxIterations = 8;

		double s =
		findSteeringMaxS(
			prev_vx[i],
			prev_vy[i],
			prev_heading[i].getRadians(),
			desired_vx[i],
			desired_vy[i],
			desired_heading[i].getRadians(),
			max_theta_step,
			kMaxIterations);
		min_s = Math.min(min_s, s);
		final double max_vel_step = dt * limits.maxDriveAcceleration();
		for (i = 0; i < modules.length; ++i) {
		  if (min_s == 0.0){
			break;
		  }
		}
		double vx_min_s = min_s == 1.0 ? desired_vx[i] : (desired_vx[i] - prev_vx[i]) * min_s + prev_vx[i];
		double vy_min_s = min_s == 1.0 ? desired_vy[i] : (desired_vy[i] - prev_vy[i]) * min_s + prev_vy[i];

		final int kMaxIterations2 = 10;
		double ds =
			min_s
				* findDriveMaxS(
					prev_vx[i],
					prev_vy[i],
					Math.hypot(prev_vx[i], prev_vy[i]),
					vx_min_s,
					vy_min_s,
					Math.hypot(vx_min_s, vy_min_s),
					max_vel_step,
					kMaxIterations2);
		min_s = Math.min(min_s, ds);

		ChassisSpeeds retSpeeds =
        new ChassisSpeeds(
            prevSetpoint.chassisSpeeds().vxMetersPerSecond + min_s * dx,
            prevSetpoint.chassisSpeeds().vyMetersPerSecond + min_s * dy,
            prevSetpoint.chassisSpeeds().omegaRadiansPerSecond + min_s * dtheta);
    	var retStates = kinematics.toSwerveModuleStates(retSpeeds);

		for(i = 0; i < modules.length; ++i){
			final var maybeOverride = overrideSteering.get(i);
			if(maybeOverride.isPresent()){
				var override = maybeOverride.get();
    			if (flipHeading(retStates[i].angle.unaryMinus().rotateBy(override))) {
         	 		retStates[i].speedMetersPerSecond *= -1.0;
       		 }
			}
			retStates[i].angle = override;
		}
		final var deltaRotation =
          prevSetpoint.moduleStates()[i].angle.unaryMinus().rotateBy(retStates[i].angle);

		  if (flipHeading(deltaRotation)) {
			retStates[i].angle = retStates[i].angle.rotateBy(Rotation2d.fromRadians(Math.PI));
			retStates[i].speedMetersPerSecond *= -1.0;
		  }
		  return new Setpoint(retSpeeds, retStates);
	 	 }

		


	
			

	public static PathSegment ContinuationPoint(Path path, SwerveDriveSubsystem swerve){
		Pose2d endPose2d = swerve.getPose();
		return PathSegment.build().setStartPose(path.getStartingPose()).setEndPose(endPose2d).build();
	}

	public static PathSegment AddTranslationPoint(){
		
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
				path.buildPaths();
			}
			else
			{
				 targetPoint(newBuilder().setModel(model)).addAllSegments(entry.getValue()).build();

			}

		}

	}

	private static String getHashCodes(RobotModel model, List<PathSegment> segments)
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

			for (PathSegment segment : segments){
				for(Waypoint waypoint : segment.getWaypoints()){
					hashString.append(format.format(waypoint.getX()));
			hashString.append(format.format(waypoint.getY()));

			if (waypoint.hasHeadingConstraint()) {
				hashString.append(format.format(waypoint.getHeadingConstraint()));
			  }
	  
			  if (waypoint.hasSamples()) {
				hashString.append(waypoint.getSamples());
			  }
	  
			  switch (waypoint.getVelocityConstraintCase()) {
				case ZERO_VELOCITY -> {
				  hashString.append(format.format(0));
				}
				case VEHICLE_VELOCITY -> {
				  hashString.append(format.format(waypoint.getVehicleVelocity().getVx()));
				  hashString.append(format.format(waypoint.getVehicleVelocity().getVy()));
				  hashString.append(format.format(waypoint.getVehicleVelocity().getOmega()));
				}
				case VELOCITYCONSTRAINT_NOT_SET -> {}
			  }
				}
			
		}
		if (segment.hasMaxVelocity()) {
			hashString.append(format.format(segment.getMaxVelocity()));
		  }
		  if (segment.hasMaxOmega()) {
			hashString.append(format.format(segment.getMaxOmega()));
		  }
	
		  hashString.append(segment.getStraightLine());
		
        }
}
	
