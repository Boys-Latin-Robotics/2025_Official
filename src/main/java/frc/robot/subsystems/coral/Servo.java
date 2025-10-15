package frc.robot.subsystems.coral;

import static java.util.Objects.requireNonNull;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// PMW(Pulse Width Modulation)
public class Servo extends SubsystemBase {

    private final ServoContext context;

    private final PWM servo;

    public Servo() {
        this(ServoContext.defaults());
    }

    public Servo(final ServoContext context) {
        requireNonNull(context, "ServoContext cannot be null");
        this.context = context;
        this.servo = new PWM(this.context.getChannel());
    }

    public void forwardPosition() {
        servo.setPosition(this.context.getForwardPosition());
    }

    public void backwardPosition() {
        servo.setPosition(this.context.getBackwardPosition());
    }

    public void middlePosition() {
        servo.setPosition(this.context.getMiddlePosition());
    }

    public Command getForwardPositionCommand() {
        return this.runEnd(this::forwardPosition, this::middlePosition);
    }

    public Command getBackwardCommand() {
        return this.runEnd(this::backwardPosition, this::middlePosition);
    }
}
