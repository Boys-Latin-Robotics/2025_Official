package frc.robot.subsystems.algae;

import static edu.wpi.first.math.util.Units.feetToMeters;

import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.support.PIDSettings;
import frc.robot.support.sparkmax.TeamSparkMax;
import frc.robot.support.sparkmax.TeamSparkMaxImpl;
import frc.robot.support.sparkmax.TeamSparkMaxSimImpl;
import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class AlgaeMechanismContext {

    /**
     * Instantiates an AlgaeMechanismContext instance with default values.
     *
     * @return Default AlgaeMechanismContext
     */
    public static AlgaeMechanismContext defaults() {
        return AlgaeMechanismContext.builder()
                .algaeMotor(
                        (Robot.isReal())
                                ? new TeamSparkMaxImpl(Constants.Algae.m_AlgaeMtrC, SparkLowLevel.MotorType.kBrushless)
                                : new TeamSparkMaxSimImpl(
                                        Constants.Algae.m_AlgaeMtrC, SparkLowLevel.MotorType.kBrushless))
                .build();
    }

    @Builder.Default
    private final Constraints algaeControllerConstraints = new Constraints(feetToMeters(10), feetToMeters(8));

    @Builder.Default
    private final PIDSettings algaeControllerPIDSettings = new PIDSettings(0.75, 0, 0);

    @Builder.Default
    private final PIDSettings algaeMotorPIDSettings = new PIDSettings(1, 0, 0);

    @Builder.Default
    private final double algaeUpPosition = -1.03;

    @Builder.Default
    private final double algaeDownPosition = -0.14;

    @Builder.Default
    private final double algaeMiddlePosition = -0.75;

    @Builder.Default
    private final double algaeGroundPosition = -1.165;

    @Builder.Default
    private final double algaeControllerTolerance = 0.035;

    @Builder.Default
    private final double advanceIncrement = 0.1;

    @Builder.Default
    private final double reverseIncrement = -0.1;

    @Builder.Default
    private final double gearRatio = 75;

    @Builder.Default
    private final int countsPerRevolution = 8192;

    @Builder.Default
    private final int algaeSensorForwardIRValue = 2400;

    @Builder.Default
    private final double algaeVelocityConversionFactor = 1.0;

    // TODO: This channel was never represented within our frc.robot.Constants class. Consider moving the definition
    // of this channel to frc.robot.Constants once initial refactor is complete
    @Builder.Default
    private final int algaeSensorChannel = 2;

    private TeamSparkMax algaeMotor;

    public double getAlgaePositionConversionFactor() {
        // revolutions -> radians
        return (2 * Math.PI) / this.gearRatio;
    }
}
