import org.junit.jupiter.api.Test;

import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTest {
    @Test
    void testShooter()
    {
        ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

        shooterSubsystem.setShooterMotorPower(1, "test");
        System.out.println("result: " + shooterSubsystem.getShooterMotorPower());
    }
}
