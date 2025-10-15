package frc.robot.subsystems.coral;

import static com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;
import static java.util.Objects.requireNonNull;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * A {@link Subsystem} implementation responsible for the operation of the coral mechanism
 */
public class CoralMechanism extends SubsystemBase {

    private final CoralMechanismContext context;

    private final TalonSRX coralMotorRight;

    private final TalonSRX coralMotorLeft;

    private final AnalogInput rearSensor;

    private final AnalogInput frontSensor;

    /**
     * Instantiates a new CoralMechanism with default {@link CoralMechanismContext}
     */
    public CoralMechanism() {
        this(CoralMechanismContext.defaults());
    }

    /**
     * Instantiates a new CoralMechanism subsystem with the specified settings
     *
     * @param context
     *            The CoralMechanismSettings to apply to this instance
     */
    public CoralMechanism(final CoralMechanismContext context) {
        requireNonNull(context, "CoralMechanismContext cannot be null");
        this.context = context;
        this.coralMotorRight = new TalonSRX(Constants.Port.RIGHT_CORAL_DRIVE_CHANNEL);
        this.coralMotorLeft = new TalonSRX(Constants.Port.LEFT_CORAL_DRIVE_CHANNEL);
        this.rearSensor = new AnalogInput(this.context.getRearSensorChannel());
        this.frontSensor = new AnalogInput(this.context.getFrontSensorChannel());
    }

    /**
     * Advances the coral motors by the specified speed advancement increments
     */
    private void advanceCoralMotors() {
        this.coralMotorRight.set(PercentOutput, this.context.getAdvanceIncrement());
        this.coralMotorLeft.set(PercentOutput, -this.context.getAdvanceIncrement());
    }

    /**
     * Reverses the coral motors by the specified speed reversing increments
     */
    private void reverseCoralMotors() {
        this.coralMotorRight.set(PercentOutput, this.context.getReverseIncrement());
        this.coralMotorLeft.set(PercentOutput, -this.context.getReverseIncrement());
    }

    /**
     * Advances the coral motors to the trough by the specified speed advancement increments
     */
    private void advanceCoralMotorsToTrough() {
        this.coralMotorRight.set(PercentOutput, this.context.getAdvanceRightMotorToTroughIncrement());
        this.coralMotorLeft.set(PercentOutput, this.context.getAdvanceLeftMotorToTroughIncrement());
    }

    /**
     * Stops the coral motors
     */
    private void stopCoralMotors() {
        this.coralMotorRight.set(PercentOutput, 0);
        this.coralMotorLeft.set(PercentOutput, 0);
    }

    /**
     * Obtains the reading value from the front sensor
     *
     * @return The value
     */
    private int getFrontSensorReading() {
        return this.frontSensor.getValue();
    }

    /**
     * Obtains the reading value from the rear sensor
     *
     * @return The value
     */
    private int getRearSensorReading() {
        return this.rearSensor.getValue();
    }

    /**
     * Obtains a "runEnd" Command which will advance the coral motors on each iteration. Interruption of the Command
     * will trigger stopping of the coral motors.
     *
     * @return The Command
     */
    public Command getAdvanceCoralCommand() {
        return this.runEnd(this::advanceCoralMotors, this::stopCoralMotors);
    }

    /**
     * Obtains a "runEnd" Command which will reverse the coral motors on each iteration. Interruption of the Command
     * will trigger stopping of the coral motors.
     *
     * @return The Command
     */
    // TODO: Remove if unused
    public Command getReverseCoralCommand() {
        return this.runEnd(this::reverseCoralMotors, this::stopCoralMotors);
    }

    /**
     * Obtains a "runEnd" Command which will advance the coral motors to the trough on each iteration. Interruption of
     * the Command will trigger stopping of the coral motors.
     *
     * @return The Command
     */
    public Command getCoralTroughCommand() {
        return this.runEnd(this::advanceCoralMotorsToTrough, this::stopCoralMotors);
    }

    /**
     * Obtains a "runEnd" Command which will advance the coral motors on each iteration and until the coral has been
     * fully loaded per {@link CoralMechanism#isCoralLoaded()}.
     *
     * @return The Command
     */
    public Command getCoralIntakeAutoCommand() {
        return this.runEnd(this::advanceCoralMotors, this::stopCoralMotors).onlyWhile(() -> !isCoralLoaded());
    }

    /**
     * Obtains a "runOnce" command that will stop the coral motors
     *
     * @return The Command
     */
    public Command getStopCoralCommand() {
        return this.runOnce(this::stopCoralMotors);
    }

    /**
     * Indicates that a coral has been fully loaded into the mechanism, as indicated by the front and rear sensors
     * detecting coral at their corresponding thresholds.
     *
     * @return Indication that a coral has been fully loaded
     */
    public boolean isCoralLoaded() {
        return (this.getFrontSensorReading() > this.context.getFrontSensorLoadedThreshold()
                && this.getRearSensorReading() > this.context.getRearSensorLoadedThreshold());
    }

    /**
     * {@link Sendable#initSendable(SendableBuilder)} implementation
     *
     * @param builder
     *            The sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Coral/IRf", this::getFrontSensorReading, null);
        builder.addDoubleProperty("Coral/IRb", this::getRearSensorReading, null);
        builder.addBooleanProperty("Coral/CoralLoaded", this::isCoralLoaded, null);
    }
}
