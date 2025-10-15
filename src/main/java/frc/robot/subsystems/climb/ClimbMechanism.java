package frc.robot.subsystems.climb;

import static edu.wpi.first.wpilibj.RobotBase.isSimulation;
import static java.util.Objects.requireNonNull;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.support.PIDSettings;
import frc.robot.support.sparkmax.TeamSparkMax;

/**
 * A {@link Subsystem} implementation responsible for the operation of the climbing mechanism
 */
public class ClimbMechanism extends SubsystemBase {

    private final TeamSparkMax climbMotor;

    private final DigitalInput climbMagSwitch;

    private final ClimbMechanismContext context;

    /**
     * Instantiates a new AlgaeIntake with default {@link ClimbMechanismContext}
     */
    public ClimbMechanism() {
        this(ClimbMechanismContext.defaults());
    }

    /**
     * Instantiates a new ClimbMechanism subsystem with the specified settings
     *
     * @param context The ClimbMechanismSettings to apply to this instance
     */
    public ClimbMechanism(final ClimbMechanismContext context) {
        requireNonNull(context, "ClimbMechanismContext cannot be null");
        this.context = context;
        this.climbMotor = this.context.getClimbMotor();
        this.climbMotor.configure(
                this.assembleClimbMotorConfig(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.climbMagSwitch = new DigitalInput(this.context.getClimbMagSwitchChannel());
    }

    /**
     * Prepares and configures a {@link SparkMaxConfig} to be applied to this mechanism's climb motor
     *
     * @return The SparkMaxConfig
     */
    private SparkMaxConfig assembleClimbMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true).idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(this.context.getClimbPositionConversionFactor())
                .velocityConversionFactor(this.context.getClimbVelocityConversionFactor());
        PIDSettings pidSettings = this.context.getClimbControllerPIDSettings();

        if (isSimulation()) {
            //  Eliminate data-port dependency in sim
            config.closedLoop
                    .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                    .pid(pidSettings.p(), pidSettings.i(), pidSettings.d());
        } else {
            // Use the alternate encoder in real
            AlternateEncoderConfig altConfig = new AlternateEncoderConfig()
                    .setSparkMaxDataPortConfig()
                    .countsPerRevolution(context.getClimbAltEncoderCPR());

            config.apply(altConfig);
            config.closedLoop
                    .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAlternateOrExternalEncoder)
                    .pid(pidSettings.p(), pidSettings.i(), pidSettings.d());
        }
        return config;
    }

    /**
     * Advances the climb motor by the specified speed advancement increment
     */
    private void advanceClimbMotor() {
        this.climbMotor.set(this.context.getAdvanceIncrement());
    }

    /**
     * Reverses the climb motor by the specified speed reversing increment
     */
    private void reverseClimbMotor() {
        this.climbMotor.set(this.context.getReverseIncrement());
    }

    /**
     * Stops the progress of the climb motor
     */
    private void stopClimbMotor() {
        this.climbMotor.set(0);
    }

    /**
     * Indicates that the climb mechanism is in the "down" position
     *
     * @return Indication
     */
    private boolean isClimbDown() {
        return !this.climbMagSwitch.get();
    }

    /**
     * Obtains a "runEnd" Command which will advance the climb mechanism motor on each iteration. Interruption of the
     * Command will trigger stopping of the climb mechanism motor.
     *
     * @return The Command
     */
    // TODO: Remove if unused
    public Command getAdvanceClimbCommand() {
        return this.runEnd(this::advanceClimbMotor, this::stopClimbMotor);
    }

    /**
     * Obtains a "runEnd" Command which will reverse the climb mechanism motor on each iteration. Interruption of the
     * Command will trigger stopping of the climb mechanism motor.
     *
     * @return The Command
     */
    // TODO: Remove if unused
    public Command getReverseClimbCommand() {
        return this.runEnd(this::reverseClimbMotor, this::stopClimbMotor);
    }

    /**
     * Obtains a "runEnd" Command which will reverse the climb mechanism motor until it is fully down per
     * {@link ClimbMechanism#isClimbDown()} and which point it will stop. Interruption of the Command will trigger
     * stopping of the climb mechanism motor.
     *
     * @return The Command
     */
    public Command getFullyReverseClimbCommand() {
        return this.runEnd(this::reverseClimbMotor, this::stopClimbMotor)
                .until(this::isClimbDown)
                .finallyDo(this::stopClimbMotor);
    }

    /**
     * Obtains a "runOnce" Command which will stop the climb mechanism motor.
     *
     * @return The Command
     */
    public Command getStopClimbCommand() {
        return this.runOnce(this::stopClimbMotor);
    }

    /**
     * Obtains the climb motor's {@link RelativeEncoder} position
     *
     * @return The position
     */
    private double getCurrentClimbPosition() {
        return this.climbMotor.getAlternateEncoder().getPosition();
    }

    /**
     * {@link Sendable#initSendable(SendableBuilder)} implementation
     *
     * @param builder The sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Climb/Position", this::getCurrentClimbPosition, null);
    }
}
