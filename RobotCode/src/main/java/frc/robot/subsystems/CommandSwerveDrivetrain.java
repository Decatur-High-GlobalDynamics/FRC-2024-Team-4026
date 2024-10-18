package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(180); // 180
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(0); // 0
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private SwerveRequest.ApplyChassisSpeeds robotRelativeDrive;
    private SwerveRequest.FieldCentricFacingAngle DriveFacingAngle;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        ConfigureSubsystem();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        ConfigureSubsystem();
    }

    public void ConfigureSubsystem() {
        if (Utils.isSimulation()) {
            startSimThread();
        }

        robotRelativeDrive = new SwerveRequest.ApplyChassisSpeeds()
                .withDriveRequestType(DriveRequestType.Velocity);

        DriveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(SwerveConstants.MAX_SPEED * 0.05)
                .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_SPEED * 0.05)
                .withDriveRequestType(DriveRequestType.Velocity);
        DriveFacingAngle.HeadingController = SwerveConstants.ROTATIONAL_AIMING_PID_CONTROLLER;

        ConfigureAutoBuilder();

        RobotContainer.getShuffleboardTab().addDouble("Gyro", () -> getPose().getRotation().getDegrees());

        RobotContainer.getShuffleboardTab().addDouble("Module Angle State",
                () -> getModule(0).getCurrentState().angle.getDegrees());
        RobotContainer.getShuffleboardTab().addDouble("Module Angle Target",
                () -> getModule(0).getTargetState().angle.getDegrees());
        RobotContainer.getShuffleboardTab().addDouble("Module Speed State 0",
                () -> getModule(0).getCurrentState().speedMetersPerSecond);
        // RobotContainer.getShuffleboardTab().addDouble("Module Speed State 1", () ->
        // getModule(1).getCurrentState().speedMetersPerSecond);
        // RobotContainer.getShuffleboardTab().addDouble("Module Speed State 2", () ->
        // getModule(2).getCurrentState().speedMetersPerSecond);
        // RobotContainer.getShuffleboardTab().addDouble("Module Speed State 3", () ->
        // getModule(3).getCurrentState().speedMetersPerSecond);
        RobotContainer.getShuffleboardTab().addDouble("Module Speed Target",
                () -> getModule(0).getTargetState().speedMetersPerSecond);
    }

    public void ConfigureAutoBuilder() {
        resetPose(new Pose2d());

        HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5, 0, 0),
                new PIDConstants(5, 0, 0),
                SwerveConstants.MAX_SPEED,
                SwerveConstants.DRIVE_BASE_RADIUS_METERS,
                new ReplanningConfig());

        BooleanSupplier isRedAlliance = () -> {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == Alliance.Red;
        };

        AutoBuilder.configureHolonomic(this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                pathFollowerConfig,
                isRedAlliance,
                this);
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public void resetPose(Pose2d location) {
        this.seedFieldRelative(location);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.m_kinematics.toChassisSpeeds(this.getState().ModuleStates);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.setControl(robotRelativeDrive.withSpeeds(speeds));
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoAimSwerveCommand(double angle) {
        return this.applyRequest(() -> DriveFacingAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(new Rotation2d(RobotContainer.isRedAlliance() ? (Math.PI - angle) : angle)));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /*
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state
         */
        /*
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match
         */
        /*
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled
         */
        /*
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
}
