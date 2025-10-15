package frc.robot.subsystems.coral;

import lombok.Builder;
import lombok.Data;

@Builder
@Data
public class ServoContext {

    public static ServoContext defaults() {
        return ServoContext.builder().build();
    }

    @Builder.Default
    private int channel = 9;

    @Builder.Default
    private final double forwardPosition = .80;

    @Builder.Default
    private double backwardPosition = 0;

    @Builder.Default
    private double middlePosition = .50;
}
