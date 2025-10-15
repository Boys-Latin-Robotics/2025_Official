package frc.robot.subsystems.algae;

import static com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;
import static java.util.Objects.requireNonNull;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A {@link Subsystem} implementation responsible for the operation of the Algae intake mechanism
 */
public class AlgaeIntake extends SubsystemBase {

    private final AlgaeIntakeContext context;

    private final TalonSRX algaeIntakeMotor;

    /**
     * Instantiates a new AlgaeIntake with default {@link AlgaeIntakeContext}
     */
    public AlgaeIntake() {
        this(AlgaeIntakeContext.defaults());
    }

    /**
     * Instantiates a new AlgaeIntake subsystem with the specified settings
     *
     * @param context
     *            The AlgaeIntakeSettings to apply to this instance
     */
    public AlgaeIntake(final AlgaeIntakeContext context) {
        requireNonNull(context, "AlgaeIntakeContext cannot be null");
        this.context = context;
        this.algaeIntakeMotor = new TalonSRX(this.context.getAlgaeIntakeMotorChannel());
    }

    /**
     * Advances the intake motor by the specified speed advancement increment
     */
    private void advanceIntake() {
        this.algaeIntakeMotor.set(PercentOutput, this.context.getAdvanceIncrement());
    }

    /**
     * Reverses the intake motor by the specified speed reversing increment
     */
    private void reverseIntake() {
        this.algaeIntakeMotor.set(PercentOutput, this.context.getReverseIncrement());
    }

    /**
     * Stops movement of the intake motor
     */
    private void stopIntake() {
        this.algaeIntakeMotor.set(PercentOutput, 0);
    }

    /**
     * Obtains a "runEnd" Command which will advance the intake motor on each iteration. Interruption of the Command
     * will trigger stopping of the intake motor.
     *
     * @return The Command
     */
    public Command getAdvanceIntakeCommand() {
        return this.runEnd(this::advanceIntake, this::stopIntake);
    }

    /**
     * Obtains a "runEnd" Command which will reverse the intake motor on each iteration. Interruption of the Command
     * will trigger stopping of the intake motor.
     *
     * @return The Command
     */
    public Command getReverseIntakeCommand() {
        return this.runEnd(this::reverseIntake, this::stopIntake);
    }

    /**
     * Obtains a "runOnce" Command which will stop the intake motor.
     *
     * @return The Command
     */
    public Command getStopIntakeCommand() {
        return this.runOnce(this::stopIntake);
    }

    /**
     * Obtains a "runOnce" Command which will advance the intake motor by the configured %.
     *
     * @return Command
     */
    public Command getAdvanceIntakeOnceCommand() {
        return this.runOnce(this::advanceIntake);
    }

    /**
     * Obtains a "runOnce" Command which will reverse the intake motor by the configured %.
     *
     * @return Command
     */
    public Command getReverseIntakeOnceCommand() {
        return this.runOnce(this::reverseIntake);
    }
}
