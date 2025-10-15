package frc.robot.commands.swervedrive;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static frc.robot.commands.swervedrive.SwerveDriveCommand.MAX_DRIVE_SPEED_ADJUSTMENT;
import static frc.robot.commands.swervedrive.SwerveDriveCommand.MAX_TURN_SPEED_ADJUSTMENT;
import static org.mockito.ArgumentMatchers.doubleThat;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import frc.robot.Constants.Controller;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class SwerveDriveCommandTest {
    @Mock
    Drivetrain drivetrain;

    @Mock
    ControllerDelegate delegate;

    @Test
    void passesScaledSpeedsToDrivetrain() {
        // Arrange: stub drivetrain BEFORE command construction
        double maxSpeed = 3.6;
        double maxTurn = Math.PI;
        when(drivetrain.getMaxSpeed()).thenReturn(maxSpeed);
        when(drivetrain.getMaxTurnAngularSpeed()).thenReturn(maxTurn);

        // Inputs (pick values safely above deadband so we don't get zeroed)
        double leftX = 0.50; // strafe right
        double leftY = -0.40; // forward (remember: code uses -leftY for x)
        double rightX = 0.0;

        when(delegate.getLeftX()).thenReturn(leftX);
        when(delegate.getLeftY()).thenReturn(leftY);
        when(delegate.getRightX()).thenReturn(rightX);
        when(delegate.getAcceleration()).thenReturn(1.0);
        when(delegate.getElevatorDecelerateRatio()).thenReturn(1.0);
        when(delegate.isHalfSpeed()).thenReturn(false);
        when(delegate.getDriver()).thenReturn(ControllerDelegate.Driver.ASA);

        SwerveDriveCommand command = new SwerveDriveCommand(delegate, drivetrain);

        command.execute();

        double x = applyDeadband(-leftY, Controller.deadzone);
        double y = applyDeadband(-leftX, Controller.deadzone);
        double rot = applyDeadband(-rightX, Controller.deadzone);

        // From your command constants:
        double driveMax = MAX_DRIVE_SPEED_ADJUSTMENT * maxSpeed;
        double turnMax = MAX_TURN_SPEED_ADJUSTMENT * maxTurn;

        double expectedX = x * driveMax * 1.0;
        double expectedY = y * driveMax * 1.0;
        double expectedRot = rot * turnMax * 1.0;

        // Assert (with tolerance)
        verify(drivetrain)
                .drive(
                        doubleThat(a -> Math.abs(a - expectedX) < 1e-9),
                        doubleThat(a -> Math.abs(a - expectedY) < 1e-9),
                        doubleThat(a -> Math.abs(a - expectedRot) < 1e-9));
    }
}
