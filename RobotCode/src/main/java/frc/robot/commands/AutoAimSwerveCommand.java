package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAimSwerveCommand extends Command {

    private final CommandSwerveDrivetrain Swerve;
    private double angle;

    private final SwerveRequest.FieldCentricFacingAngle DriveFacingAngle;
    
    public AutoAimSwerveCommand(CommandSwerveDrivetrain Swerve, double angle) {
        this.Swerve = Swerve;
        this.angle = RobotContainer.isRedAlliance() ? (Math.PI - angle) : angle;

        DriveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.Velocity);
        DriveFacingAngle.HeadingController = SwerveConstants.ROTATIONAL_AIMING_PID_CONTROLLER;

        addRequirements(Swerve);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Swerve.applyRequest(() -> DriveFacingAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(new Rotation2d(angle)));
    }

    @Override
    public void end(boolean isFinished) {
        
    }

    @Override
    public boolean isFinished() {
        return Swerve.getPose().getRotation().getRadians() < angle + 0.1
                && Swerve.getPose().getRotation().getRadians() > angle - 0.1;
    }

}
