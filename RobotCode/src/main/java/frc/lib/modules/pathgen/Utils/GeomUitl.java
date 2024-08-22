package frc.lib.modules.pathgen.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GeomUitl
{
    public GeomUitl()
    {

    }

    public static Transform2d toTransform2d(Pose2d pose)
    {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    public static Transform2d toTransform2d(double x, double y, Rotation2d angle)
    {
        return new Transform2d(x, y, angle);
    }

    public static Transform2d toTransform2d(Rotation2d rotation)
    {
        return new Transform2d(new Translation2d(), rotation);
    }

    public static Pose2d inverse(Pose2d pose)
    {
        return new Pose2d(pose.getTranslation().rotateBy(pose.getRotation().unaryMinus()),
                pose.getRotation().unaryMinus());
    }

    public static Pose2d toPose2d(Transform2d transform)
    {
        return new Pose2d(transform.getTranslation(), transform.getRotation());

    }

    public static Pose2d toPose2d(Translation2d translation)
    {
        return new Pose2d(translation, new Rotation2d());
    }

    public static Pose2d toPose2d(Rotation2d rotation)
    {
        return new Pose2d(new Translation2d(), rotation);
    }

    public static Pose3d toTransform3d(Pose3d pose)
    {
        return new Pose3d(pose.getTranslation(), pose.getRotation());
    }

    public static Pose3d toPose3d(Transform3d transform)
    {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    public static Twist2d toTwist2d(ChassisSpeeds speeds)
    {
        return new Twist2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond);
    }

    public static Twist2d multiply(Twist2d twist, double factor)
    {
        return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
    }

    public static Pose2d withTranslation()
    {
        return new Pose2d(new Translation2d(), new Rotation2d());
    }

    public static Pose2d withRotation()
    {
        return new Pose2d(new Translation2d(), new Rotation2d());
    }

}
