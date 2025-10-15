package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkLowLevel;
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
public class ClimbMechanismContext {

    /**
     * Instantiates an ClimbMechanismContext instance with default values.
     *
     * @return Default ClimbMechanismContext
     */
    public static ClimbMechanismContext defaults() {
        return ClimbMechanismContext.builder()
                .climbMotor(
                        (Robot.isReal())
                                ? new TeamSparkMaxImpl(
                                        Constants.Port.CLIMB_DRIVE_CHANNEL, SparkLowLevel.MotorType.kBrushless)
                                : new TeamSparkMaxSimImpl(
                                        Constants.Port.CLIMB_DRIVE_CHANNEL, SparkLowLevel.MotorType.kBrushless))
                .build();
    }

    @Builder.Default
    private int climbMotorChannel = Constants.Port.CLIMB_DRIVE_CHANNEL;

    @Builder.Default
    private int climbMagSwitchChannel = Constants.Port.climbMagSwitchDIOC;

    @Builder.Default
    private PIDSettings climbControllerPIDSettings = new PIDSettings(1, 0, 0);

    @Builder.Default
    private double climbPositionConversionFactor = 1;

    @Builder.Default
    private double climbVelocityConversionFactor = 1;

    @Builder.Default
    private final double advanceIncrement = .75;

    @Builder.Default
    private double reverseIncrement = -.75;

    @Builder.Default
    // TODO: Look up the datasheet for the exact encoder you installed on the climb mechanism.
    //  If we're not using a quadrature encoder at all, then don’t pick kAlternateOrExternalEncoder
    //  Instead use the primary encoder or the appropriate sensor type.
    private int climbAltEncoderCPR = (int) Constants.Conversion.MagEncoderCountsPerRev;

    private TeamSparkMax climbMotor;
}
