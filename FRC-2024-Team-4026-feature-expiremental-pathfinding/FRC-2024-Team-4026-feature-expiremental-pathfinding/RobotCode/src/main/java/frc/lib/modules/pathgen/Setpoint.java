import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public record Setpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates) {
}
