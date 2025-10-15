package frc.robot.subsystems.algae;

import frc.robot.Constants;
import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class AlgaeIntakeContext {

    public static AlgaeIntakeContext defaults() {
        return AlgaeIntakeContext.builder().build();
    }

    @Builder.Default
    private final int algaeIntakeMotorChannel = Constants.Algae.intakeMotorChannel;

    @Builder.Default
    private final double advanceIncrement = .75;

    @Builder.Default
    private double reverseIncrement = -.85;
}
