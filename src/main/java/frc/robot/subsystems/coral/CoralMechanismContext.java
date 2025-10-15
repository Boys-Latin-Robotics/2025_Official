package frc.robot.subsystems.coral;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class CoralMechanismContext {

    public static CoralMechanismContext defaults() {
        return CoralMechanismContext.builder().build();
    }

    @Builder.Default
    private final double advanceIncrement = .60;

    @Builder.Default
    private double reverseIncrement = -.60;

    @Builder.Default
    private final double advanceRightMotorToTroughIncrement = .95;

    @Builder.Default
    private final double advanceLeftMotorToTroughIncrement = -.25;

    // TODO: These rear and front channels were never represented within our frc.robot.Constants class. Consider moving
    // the definition
    // of this channel to frc.robot.Constants once initial refactor is complete
    @Builder.Default
    private int rearSensorChannel = 0;

    @Builder.Default
    private int frontSensorChannel = 1;

    @Builder.Default
    private int frontSensorLoadedThreshold = 2200;

    @Builder.Default
    private int rearSensorLoadedThreshold = 1600;
}
