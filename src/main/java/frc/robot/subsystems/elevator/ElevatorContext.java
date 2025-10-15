package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.support.DIOChannel;
import frc.robot.support.PIDSettings;
import frc.robot.support.sparkmax.TeamSparkMax;
import frc.robot.support.sparkmax.TeamSparkMaxImpl;
import frc.robot.support.sparkmax.TeamSparkMaxSimImpl;
import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class ElevatorContext {

    public static ElevatorContext defaults() {
        return ElevatorContext.builder()
                .elevatorMotor(
                        (Robot.isReal()
                                ? new TeamSparkMaxImpl(Constants.Port.ELEVATOR_DRIVE_CHANNEL, MotorType.kBrushless)
                                : new TeamSparkMaxSimImpl(Constants.Port.ELEVATOR_DRIVE_CHANNEL, MotorType.kBrushless)))
                .elevatorFollowerMotor(
                        (Robot.isReal()
                                ? new TeamSparkMaxImpl(
                                        Constants.Port.ELEVATOR_FOLLOWER_DRIVE_CHANNEL, MotorType.kBrushless)
                                : new TeamSparkMaxSimImpl(
                                        Constants.Port.ELEVATOR_FOLLOWER_DRIVE_CHANNEL, MotorType.kBrushless)))
                .build();
    }

    @Builder.Default
    private PIDSettings elevatorControllerPIDSettings = new PIDSettings(0.2, 0, 0);

    @Builder.Default
    private PIDSettings feedbackSensorPIDSettings = new PIDSettings(1.0, 0, 0);

    @Builder.Default
    private double elevatorControllerTolerance = .16;

    @Builder.Default
    private int elevatorMotorChannel = Constants.Port.ELEVATOR_DRIVE_CHANNEL;

    @Builder.Default
    private int elevatorFollowerMotorChannel = Constants.Port.ELEVATOR_FOLLOWER_DRIVE_CHANNEL;

    @Builder.Default
    private double downPosition = 0;

    @Builder.Default
    private double level2Position = 5.8;

    @Builder.Default
    private double troughPosition = 2.5;

    @Builder.Default
    private double algaeGroundPosition = 6.2;

    @Builder.Default
    private double algaeLevel3Position = 10.2;

    @Builder.Default
    private double level3Position = 13;

    @Builder.Default
    private double algaeLevel4Position = 17.6;

    @Builder.Default
    private double level4Position = 24.5;

    @Builder.Default
    private final double advanceIncrement = .15;

    @Builder.Default
    private double reverseIncrement = -.05;

    @Builder.Default
    private final int countsPerRevolution = 8192;

    @Builder.Default
    private final double elevatorCeiling = 23.7;

    @Builder.Default
    private final double elevatorDecelerationOffset = 5.6;

    @Builder.Default
    private final double elevatorSpeedMotorPositionAdjustment = .7;

    @Builder.Default
    private final double elevatorSpeedBoostFactor = 10;

    @Builder.Default
    // 1.6 * Math.PI = Distance per
    private final double elevatorPositionConversionFactor = 1.6 * Math.PI;

    @Builder.Default
    private final double elevatorVelocityConversionFactor = 1;

    @Builder.Default
    private final int elevatorLimitSwitchTopChannel = DIOChannel.SIX.getChannel();

    @Builder.Default
    private final int elevatorLimitSwitchBottomChannel = DIOChannel.SEVEN.getChannel();

    @Builder.Default
    private TrapezoidProfile.Constraints elevatorConstraints =
            new TrapezoidProfile.Constraints(Units.feetToMeters(140), Units.feetToMeters(125));

    private TeamSparkMax elevatorMotor;

    private TeamSparkMax elevatorFollowerMotor;
}
