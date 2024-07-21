package frc.lib.modules.pathgen;

import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.modules.pathgen.Constants.FieldConstants;

public class RobotState
{
	final double X = 1;
	final double Y = 2;
	final double THETA = 3;
	final double V = 4;
	final double W = 5;
	final double OMEGA = 6;

	 public record OdometryObservation(
      SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {}

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

  public record FlywheelSpeeds(double leftSpeed, double rightSpeed) {
    public static FlywheelSpeeds interpolate(FlywheelSpeeds t1, FlywheelSpeeds t2, double v) {
      double leftSpeed = MathUtil.interpolate(t1.leftSpeed(), t2.leftSpeed(), v);
      double rightSpeed = MathUtil.interpolate(t1.rightSpeed(), t2.rightSpeed(), v);
      return new FlywheelSpeeds(leftSpeed, rightSpeed);
    }
  }

  public record AimingParameters(
      Rotation2d driveHeading,
      Rotation2d armAngle,
      double effectiveDistance,
      FlywheelSpeeds flywheelSpeeds) {}

  public record DemoFollowParameters(
      Pose2d targetPose, Rotation2d targetHeading, Rotation2d armAngle) {}

  public record DemoShotParameters(Rotation2d armAngle, FlywheelSpeeds flywheelSpeeds) {}

  private static final double poseBufferSizeSeconds = 2.0;

  private static final double armAngleCoefficient = 57.254371165197;
  private static final double armAngleExponent = -0.593140189605718;

  
  // Super poop
  private static final InterpolatingDoubleTreeMap superPoopArmAngleMap =
      new InterpolatingDoubleTreeMap();

  static {
    superPoopArmAngleMap.put(Units.feetToMeters(33.52713263758169), 35.0);
    superPoopArmAngleMap.put(Units.feetToMeters(28.31299227120627), 39.0);
    superPoopArmAngleMap.put(Units.feetToMeters(25.587026383435525), 48.0);
  }

  private static final InterpolatingTreeMap<Double, FlywheelSpeeds> superPoopFlywheelSpeedsMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), FlywheelSpeeds::interpolate);


  private static final double autoFarShotCompensationDegrees = 0.0; // 0.6 at NECMP

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Pose Estimation Members
  private static Pose2d odometryPose = new Pose2d();
  private static Pose2d estimatedPose = new Pose2d();
  private final static TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
  private Pose2d trajectorySetpoint = new Pose2d();
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  // Odometry

  private SwerveDriveWheelPositions lastWheelPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });
  private Rotation2d lastGyroAngle = new Rotation2d();
  private Twist2d robotVelocity = new Twist2d();
  private Twist2d trajectoryVelocity = new Twist2d();

  /** Cached latest aiming parameters. Calculated in {@code getAimingParameters()} */
  private AimingParameters latestParameters = null;

  private AimingParameters latestSuperPoopParameters = null;
  // Demo parameters
  private Pose3d demoTagPose = null;
  private DemoFollowParameters latestDemoParamters = null;

  
  private DemoShotParameters demoShotParameters =
      new DemoShotParameters(Rotation2d.fromDegrees(0.0), new FlywheelSpeeds(0.0, 0.0));

  private BooleanSupplier lookaheadDisable = () -> false;

  
  /** Add odometry observation */
  public void addOdometryObservation(OdometryObservation observation) {
    latestParameters = null;
    latestSuperPoopParameters = null;
   
  }

  public void addVisionObservation(VisionObservation observation) {
    latestParameters = null;
    latestSuperPoopParameters = null;
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
          > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // sample --> odometryPose transform and backwards of that
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    // get old estimate by applying odometryToSample Transform
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }
    // difference between estimate and vision pose
    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
    // scale transform by visionK
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate current estimate by applying scaled transform to old estimate
    // then replaying odometry data
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }

  public void addVelocityData(Twist2d robotVelocity) {
    latestParameters = null;
    this.robotVelocity = robotVelocity;
  }

  public void addTrajectoryVelocityData(Twist2d robotVelocity) {
    latestParameters = null;
    trajectoryVelocity = robotVelocity;
  }

  public AimingParameters getAimingParameters() {
    if (latestParameters != null) {
      // Cache previously calculated aiming parameters. Cache is invalidated whenever new
      // observations are added.
      return latestParameters;
    }

    Transform2d fieldToTarget =
        Path.apply(FieldConstants.Speaker.centerSpeakerOpening)
            .toTranslation2d()
            .toTransform2d()
            .plus();
    Pose2d fieldToPredictedVehicle;
    Pose2d fieldToPredictedVehicleFixed =
        new Pose2d(fieldToPredictedVehicle.getTranslation(), new Rotation2d());

  
    
  }

  private static final Translation2d superPoopTarget =
      FieldConstants.Subwoofer.centerFace
          .getTranslation()
          .interpolate(FieldConstants.ampCenter, 0.5);


  public void setDemoTagPose(Pose3d demoTagPose) {
    this.demoTagPose = demoTagPose;
    latestDemoParamters = null;
  }



  public Optional<DemoFollowParameters> getDemoTagParameters() {
    if (latestDemoParamters != null) {
      // Use cached demo parameters.
      return Optional.of(latestDemoParamters);
    }
    // Return empty optional if no demo tag pose.
    if (demoTagPose == null) return Optional.empty();

    // Calculate target pose.
    Pose2d targetPose =
        demoTagPose
            .toPose2d()
            .transformBy(
                new Transform2d(
                    new Translation2d());

    // Calculate heading without movement.
    Translation2d demoTagFixed = demoTagPose.getTranslation().toTranslation2d();
    Translation2d robotToDemoTagFixed = demoTagFixed.minus(getEstimatedPose().getTranslation());
    Rotation2d targetHeading = robotToDemoTagFixed.getAngle();

    

  

  public boolean inShootingZone() {
    Pose2d robot = Path.apply(getEstimatedPose());
    if (robot.getY() <= FieldConstants.Stage.ampLeg.getY()) {
      return robot.getX() <= FieldConstants.wingX;
    } else {
      return robot.getX() <= FieldConstants.fieldLength / 2.0 + 0.5;
    }
  }

  public boolean inCloseShootingZone() {
    return getEstimatedPose()
            .getTranslation()
            .getDistance(
                Path.apply(
                    FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()))
					//remember to put real value
        < Units.feetToMeters(2);
  }

  /**
   * Reset estimated pose and odometry pose to pose <br>
   * Clear pose buffer
   */
public static void resetPose(Pose2d initialPose) {
	estimatedPose = initialPose;
	odometryPose = initialPose;
	poseBuffer.clear();
}

public Twist2d fieldVelocity() {
    Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(estimatedPose.getRotation());
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  }

  
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  /**
   * Predicts what our pose will be in the future. Allows separate translation and rotation
   * lookaheads to account for varying latencies in the different measurements.
   *
   * @param translationLookaheadS The lookahead time for the translation of the robot
   * @param rotationLookaheadS The lookahead time for the rotation of the robot
   * @return The predicted pose.
   */
  public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
    Twist2d velocity = DriverStation.isAutonomousEnabled() ? trajectoryVelocity : robotVelocity;
    return getEstimatedPose()
        .transformBy(
            new Transform2d(
                velocity.dx * translationLookaheadS,
                velocity.dy * translationLookaheadS,
                Rotation2d.fromRadians(velocity.dtheta * rotationLookaheadS)));
  }

 
  public Pose2d getOdometryPose() {
    return odometryPose;
  }
}
