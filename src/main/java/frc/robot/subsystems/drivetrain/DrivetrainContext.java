package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.support.PIDSettings;
import java.io.IOException;
import java.util.Optional;
import lombok.Builder;
import lombok.Data;
import org.json.simple.parser.ParseException;

@Data
@Builder
public class DrivetrainContext {

    public static DrivetrainContext defaults() {
        return DrivetrainContext.builder().build();
    }

    @Builder.Default
    private double maxSpeed = Units.feetToMeters(12.5); // WP this seemed to work don't know why // 3.68;

    @Builder.Default
    private PIDSettings lateralMovementPIDSettings = new PIDSettings(3, 0, 0);

    @Builder.Default
    private PIDSettings rotationPIDSettings = new PIDSettings(3, 0, 0);

    @Builder.Default
    private SwerveModuleContext flSwerveContext = SwerveModuleContext.builder()
            .name("Swerve Module/Front Left")
            .driveMotorId(Constants.Port.FRONT_LEFT_DRIVE_CHANNEL)
            .turningMotorId(Constants.Port.FRONT_LEFT_STEER_CHANNEL)
            .turnEncoderPWMChannel(Constants.Port.FRONT_LEFT_TURN_ENCODER_DIO_CHANNEL)
            .turnOffset(Constants.RobotVersion2025.flTurnEncoderOffset)
            .build();

    @Builder.Default
    private SwerveModuleContext frSwerveContext = SwerveModuleContext.builder()
            .name("Swerve Module/Front Right")
            .driveMotorId(Constants.Port.FRONT_RIGHT_DRIVE_CHANNEL)
            .turningMotorId(Constants.Port.FRONT_RIGHT_STEER_CHANNEL)
            .turnEncoderPWMChannel(Constants.Port.FRONT_RIGHT_TURN_ENCODER_DIO_CHANNEL)
            .turnOffset(Constants.RobotVersion2025.frTurnEncoderOffset)
            .build();

    @Builder.Default
    private SwerveModuleContext rlSwerveContext = SwerveModuleContext.builder()
            .name("Swerve Module/Back Left")
            .driveMotorId(Constants.Port.REAR_LEFT_DRIVE_CHANNEL)
            .turningMotorId(Constants.Port.REAR_LEFT_TURN_CHANNEL)
            .turnEncoderPWMChannel(Constants.Port.REAR_LEFT_TURN_ENCODER_DIO_CHANNEL)
            .turnOffset(Constants.RobotVersion2025.rlTurnEncoderOffset)
            .build();

    @Builder.Default
    private SwerveModuleContext rrSwerveContext = SwerveModuleContext.builder()
            .name("Swerve Module/Back Right")
            .driveMotorId(Constants.Port.REAR_RIGHT_DRIVE_CHANNEL)
            .turningMotorId(Constants.Port.REAR_RIGHT_STEER_CHANNEL)
            .turnEncoderPWMChannel(Constants.Port.REAR_RIGHT_TURN_ENCODER_DIO_CHANNEL)
            .turnOffset(Constants.RobotVersion2025.rrTurnEncoderOffset)
            .build();

    @Builder.Default
    private double flTurnOffset = Constants.RobotVersion2025.flTurnEncoderOffset;

    @Builder.Default
    private double frTurnOffset = Constants.RobotVersion2025.frTurnEncoderOffset;

    @Builder.Default
    private double rlTurnOffset = Constants.RobotVersion2025.rlTurnEncoderOffset;

    @Builder.Default
    private double rrTurnOffset = Constants.RobotVersion2025.rrTurnEncoderOffset;

    @Builder.Default
    private Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    @Builder.Default
    private Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));

    @Builder.Default
    private SwerveModuleState fullStopAt135Degrees = new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4)));

    @Builder.Default
    private SwerveModuleState fullStopAt45Degrees = new SwerveModuleState(0, new Rotation2d((Math.PI / 4)));

    @Builder.Default
    private boolean limelightMegaTag2AlgorithmEnabled = true;

    @Builder.Default
    private int angularVelocityThreshold = 720;

    @Builder.Default
    private double rawFiducialAmbiguityThreshold = 0.7;

    @Builder.Default
    private double rawFiducialDistanceToCameraThreshold = 3.0;

    public double getMaxTurnAngularSpeed() {
        return this.maxSpeed / Constants.Drive.SMBackLeftLocation.getNorm(); // 1/2
    }

    public Optional<RobotConfig> getRobotConfig() {
        try {
            return Optional.of(RobotConfig.fromGUISettings());
        } catch (IOException | ParseException e) {
            // TODO: Correct logging
            System.err.println("Unable to obtain RobotConfig from GUI Settings!");
            return Optional.empty();
        }
    }
}
