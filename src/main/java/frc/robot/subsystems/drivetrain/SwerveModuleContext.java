package frc.robot.subsystems.drivetrain;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Robot;
import frc.robot.support.PIDSettings;
import frc.robot.support.sparkmax.TeamSparkMax;
import frc.robot.support.sparkmax.TeamSparkMaxImpl;
import frc.robot.support.sparkmax.TeamSparkMaxSimImpl;
import lombok.Builder;
import lombok.Getter;

@Builder
public class SwerveModuleContext {

    public static SwerveModuleContext defaults() {
        return SwerveModuleContext.builder().build();
    }

    // Drive motor CAN ID
    private final int driveMotorId;

    // Turning motor CAN ID
    private final int turningMotorId;

    // Name of the SwerveModule
    @Getter
    private String name;

    // DIO input for the drive encoder channel B
    @Getter
    private final int turnEncoderPWMChannel;

    // Offset from 0 to 1 for the home position of the encoder
    @Getter
    private final double turnOffset;

    @Getter
    @Builder.Default
    private PIDSettings driveMotorPIDSettings = new PIDSettings(1, 0, 0);

    @Getter
    @Builder.Default
    // Used to scale the normalized angular error into motor power for the turning motor
    private final double rotationalProportionalGain = 1.6;

    @Getter(lazy = true)
    private final TeamSparkMax driveMotor = createMotor(driveMotorId, MotorType.kBrushless);

    @Getter(lazy = true)
    private final TeamSparkMax turningMotor = createMotor(turningMotorId, MotorType.kBrushless);

    // 🆕 Simulation parameters
    @Getter
    @Builder.Default
    private double driveGearRatio = 6.75;

    @Getter
    @Builder.Default
    private double turnGearRatio = 150.0;

    @Getter
    @Builder.Default
    private double driveInertia = 0.025; // kg·m²

    @Getter
    @Builder.Default
    private double turnInertia = 0.004; // kg·m²

    /**
     * Private static utility to conditionally construct a TeamSparkMaxImpl or TeamSparkMaxSimImpl instance with the
     * given canId and motor type, based on the state of {@link Robot#isReal()}
     *
     * @param canId
     *            The id of the motor to instantiate
     * @param type
     *            The type of the motor to instantiate
     * @return The instantiated TeamSparkMax instance
     */
    private static TeamSparkMax createMotor(int canId, MotorType type) {
        return (Robot.isReal()) ? new TeamSparkMaxImpl(canId, type) : new TeamSparkMaxSimImpl(canId, type);
    }
}
