package frc.robot.subsystems.algae;

import static java.util.Objects.requireNonNull;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.support.PIDSettings;
import frc.robot.support.sparkmax.TeamSparkMax;

/**
 * A {@link Subsystem} implementation responsible for the operation of the Algae mechanism
 */
public class AlgaeMechanism extends SubsystemBase {

    // TODO: This global flag should be migrated to AlgaeMechanismSettings or another unit rather than just being hung
    // off this class as a static member. Presently toggled from our frc.robot.Robot implementation
    public static boolean AUTO_RUNNING;

    private final AlgaeMechanismContext context;

    private final TeamSparkMax algaeMotor;

    private final ProfiledPIDController algaeController;

    private final AnalogInput algaeSensor;

    private double algaeSpeed;

    private double nextAlgaePosition;

    /**
     * Instantiates a new AlgaeMechanism with default {@link AlgaeMechanismContext}
     */
    public AlgaeMechanism() {
        this(AlgaeMechanismContext.defaults());
    }

    /**
     * Instantiates a new AlgaeMechanism with the specified settings
     *
     * @param context
     *            The {@link AlgaeMechanismContext}
     */
    public AlgaeMechanism(final AlgaeMechanismContext context) {
        requireNonNull(context, "AlgaeMechanismContext cannot be null");
        this.context = context;
        this.algaeMotor = this.context.getAlgaeMotor();
        this.algaeMotor.configure(
                this.assembleAlgaeMotorConfig(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        PIDSettings pidSettings = this.context.getAlgaeControllerPIDSettings();
        this.algaeController = new ProfiledPIDController(
                pidSettings.p(), pidSettings.i(), pidSettings.d(), this.context.getAlgaeControllerConstraints());
        this.algaeController.setTolerance(this.context.getAlgaeControllerTolerance());
        this.algaeSensor = new AnalogInput(this.context.getAlgaeSensorChannel());
        resetAlgaeMotorPosition();
    }

    /**
     * Prepares and configures a {@link SparkMaxConfig} to be applied to this mechanism's algae motor
     *
     * @return The SparkMaxConfig
     */
    private SparkMaxConfig assembleAlgaeMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        PIDSettings pidSettings = this.context.getAlgaeMotorPIDSettings();
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(pidSettings.p(), pidSettings.i(), pidSettings.d());
        config.inverted(false).idleMode(IdleMode.kCoast);
        config.alternateEncoder // TODO MAKE SURE TO USE RIGHT TYPE OF ENCODER WHEN DOING CONFIGS!
                .positionConversionFactor(this.context.getAlgaePositionConversionFactor())
                .velocityConversionFactor(this.context.getAlgaeVelocityConversionFactor())
                .countsPerRevolution(this.context.getCountsPerRevolution());
        return config;
    }

    /**
     * Updates this subsystem's {@link edu.wpi.first.math.controller.PIDController} by setting its goal to the specified
     * position, and subsequently applies the calculated speed to the algae motor. When the specified position is
     * reached, the algae motor's speed is set to 0. This method is intended to be invoked by this subsystem's
     * {@link Subsystem#periodic()} implementation.
     *
     * @param position
     *            The desired position
     */
    private void updateAlgaeControllerPosition(double position) {
        this.algaeController.setGoal(position);
        this.algaeSpeed = this.algaeController.calculate(this.getCurrentAlgaePosition());
        if (this.algaeController.atGoal()) {
            this.algaeSpeed = 0;
        }
        // NOTE: Why is this value inverted?
        this.moveAlgaeMotor(-algaeSpeed);
    }

    /**
     * Resets the algae motor's {@link RelativeEncoder} position to 0
     */
    private void resetAlgaeMotorPosition() {
        this.algaeMotor.getAlternateEncoder().setPosition(0);
    }

    /**
     * Advances the algae motor by the specified speed advancement increment
     */
    private void advanceAlgaeMotor() {
        this.moveAlgaeMotor(this.context.getAdvanceIncrement());
    }

    /**
     * Reverses the algae motor by the specified speed reversing increment
     */
    private void reverseAlgaeMotor() {
        this.moveAlgaeMotor(this.context.getReverseIncrement());
    }

    /**
     * Moves the algae motor per the specified speed
     *
     * @param speed
     *            The speed to move the algae motor by
     */
    private void moveAlgaeMotor(double speed) {
        this.algaeMotor.set(speed);
    }

    /**
     * Provides indication of whether the algae IR sensor as reached or exceeded the specified threshold
     *
     * @return Indication
     */
    private boolean isAlgaeSensorIntakeForwardIR() {
        return this.algaeSensor.getValue() >= this.context.getAlgaeSensorForwardIRValue();
    }

    /**
     * Stops the progress of the algae motor
     */
    public void stopAlgaeMotor() {
        this.moveAlgaeMotor(0);
    }

    /**
     * Obtains the algae motor's {@link RelativeEncoder} position
     *
     * @return The position
     */
    public double getCurrentAlgaePosition() {
        return this.algaeMotor.getAlternateEncoder().getPosition();
    }

    /**
     * Sets the next position for the algae mechanism's PID controller to achieve
     *
     * @param nextAlgaePosition
     *            The desired position
     */
    public void setNextAlgaePosition(double nextAlgaePosition) {
        this.nextAlgaePosition = nextAlgaePosition;
    }

    /**
     * Resets the algae mechanism's PID controller
     */
    public void resetPIDController() {
        this.algaeController.reset(this.getCurrentAlgaePosition());
    }

    /**
     * Provides indication of whether the algae mechanism's PID controller has achieved its currently assigned goal
     *
     * @return Indication that the goal has been achieved
     */
    public boolean isAlgaePIDControllerAtGoal() {
        return this.algaeController.atGoal();
    }

    /**
     * {@link Subsystem#periodic()} implementation which will continuously update the position of the algae mechanism's
     * PID controller when the Robot is being teleoperated, according to the currently assigned "next" position
     * according to {@link AlgaeMechanism#setNextAlgaePosition(double)}. This implementation will also ensure that if
     * the algae IR sensor recognizes that algae is present within the mechanism, the next position for the algae motor
     * to achieve with be the value provided by {@link AlgaeMechanismContext#getAlgaeMiddlePosition()}
     */
    @Override
    public void periodic() {
        if (!AUTO_RUNNING) {
            if (isAlgaeSensorIntakeForwardIR()) {
                this.nextAlgaePosition = this.context.getAlgaeMiddlePosition();
            }
            this.updateAlgaeControllerPosition(this.nextAlgaePosition);
        }
    }

    /**
     * Obtains a "runEnd" Command which will advance the algae mechanism motor on each iteration. Interruption of the
     * Command will trigger stopping of the algae mechanism motor.
     *
     * @return The Command
     */
    // TODO: Remove if unused
    public Command getAdvanceAlgaeCommand() {
        return this.runEnd(this::advanceAlgaeMotor, this::stopAlgaeMotor);
    }

    /**
     * Obtains a "runEnd" Command which will reverse the algae mechanism motor on each iteration. Interruption of the
     * Command will trigger stopping of the algae mechanism motor.
     *
     * @return The Command
     */
    // TODO: Remove if unused
    public Command getReverseAlgaeCommand() {
        return this.runEnd(this::reverseAlgaeMotor, this::stopAlgaeMotor);
    }

    /**
     * Obtains a "runOnce" Command which will stop the algae mechanism motor.
     *
     * @return The Command
     */
    // TODO: Remove if unused
    public Command getStopAlgaeCommand() {
        return this.runOnce(this::stopAlgaeMotor);
    }

    /**
     * Obtains a "runOnce" Command which will reset the algae mechanism motor position to 0.
     *
     * @return The Command
     */
    public Command getResetAlgaeCommand() {
        return this.runOnce(this::resetAlgaeMotorPosition);
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
        builder.addDoubleProperty(this.getName() + "/Algae/Position/EncoderPos", this::getCurrentAlgaePosition, null);
        builder.addDoubleProperty(
                this.getName() + "/Algae/Position/Goal", () -> this.algaeController.getGoal().position, null);
        builder.addBooleanProperty(this.getName() + "/Algae/Position/atGoal", this::isAlgaePIDControllerAtGoal, null);
        builder.addDoubleProperty(this.getName() + "/Algae/Position/Speed", () -> this.algaeSpeed, null);
        builder.addBooleanProperty(this.getName() + "/Intake/IRGood", this::isAlgaeSensorIntakeForwardIR, null);
        builder.addDoubleProperty(this.getName() + "/Intake/IRValue", this.algaeSensor::getValue, null);
        builder.addBooleanProperty(this.getName() + "/Algae/AutoRunning", () -> AUTO_RUNNING, null);
    }
}
