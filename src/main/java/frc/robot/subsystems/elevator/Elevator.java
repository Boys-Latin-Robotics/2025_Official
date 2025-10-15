package frc.robot.subsystems.elevator;

import static java.util.Objects.requireNonNull;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.support.PIDSettings;
import frc.robot.support.sparkmax.TeamSparkMax;

public class Elevator extends SubsystemBase {

    private final ElevatorContext context;

    private double elevatorSpeed;

    private double elevatorPosition;

    // A motor to rotate up and down
    private final TeamSparkMax elevatorMotor;
    private final TeamSparkMax elevatorFollowerMotor;
    private final ProfiledPIDController elevatorController;
    private final DigitalInput elevatorLimitSwitchTop;
    private final DigitalInput elevatorLimitSwitchBottom;

    public Elevator() {
        this(ElevatorContext.defaults());
    }

    public Elevator(final ElevatorContext context) {
        requireNonNull(context, "ElevatorContext cannot be null");

        this.context = context;

        this.elevatorLimitSwitchTop = new DigitalInput(this.context.getElevatorLimitSwitchTopChannel());

        this.elevatorLimitSwitchBottom = new DigitalInput(this.context.getElevatorLimitSwitchBottomChannel());

        this.elevatorMotor = this.context.getElevatorMotor();

        this.elevatorMotor.configure(
                this.assembleElevatorMotorConfig(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        this.elevatorFollowerMotor = this.context.getElevatorFollowerMotor();
        this.elevatorFollowerMotor.configure(
                this.assembleElevatorFollowerMotorConfig(),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        PIDSettings pidSettings = this.context.getElevatorControllerPIDSettings();
        this.elevatorController = new ProfiledPIDController(
                pidSettings.p(), pidSettings.i(), pidSettings.d(), this.context.getElevatorConstraints());
        this.elevatorController.setTolerance(this.context.getElevatorControllerTolerance());

        this.resetPosition();
    }

    private void advanceElevatorMotorDown() {
        this.elevatorMotor.set(this.context.getReverseIncrement());
    }

    private SparkMaxConfig assembleElevatorMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false).idleMode(IdleMode.kBrake);
        PIDSettings pidSettings = this.context.getFeedbackSensorPIDSettings();
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .pid(pidSettings.p(), pidSettings.i(), pidSettings.d());
        config.alternateEncoder
                .positionConversionFactor(this.context.getElevatorPositionConversionFactor())
                .velocityConversionFactor(this.context.getElevatorVelocityConversionFactor())
                .countsPerRevolution(this.context.getCountsPerRevolution());
        return config;
    }

    private SparkMaxConfig assembleElevatorFollowerMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.follow(Constants.Port.ELEVATOR_DRIVE_CHANNEL, true);
        return config;
    }

    private void setElevatorMotorUp() {
        if (!elevatorLimitSwitchTop.get()) {
            this.elevatorMotor.set(this.context.getAdvanceIncrement());
        } else {
            this.elevatorMotor.set(0);
        }
    }

    private void resetPosition() {
        this.elevatorMotor.getAlternateEncoder().setPosition(0);
    }

    private boolean isElevatorLimitSwitchTop() {
        return elevatorLimitSwitchTop.get();
    }

    private boolean isElevatorLimitSwitchBottom() {
        return elevatorLimitSwitchBottom.get();
    }

    private void setElevatorMotorMoveVoltage(double voltage) {
        this.elevatorMotor.setVoltage(voltage);
    }

    private double getElevatorMotorEncoderPosition() {
        return this.elevatorMotor.getAlternateEncoder().getPosition();
    }

    private void updatePIDController(double position) {
        this.elevatorController.setGoal(position);
        this.elevatorSpeed = ((this.context.getElevatorSpeedBoostFactor())
                        * this.elevatorController.calculate(this.getElevatorMotorEncoderPosition())
                + this.context.getElevatorSpeedMotorPositionAdjustment());

        if ((this.isElevatorLimitSwitchTop() && this.elevatorSpeed > 0)
                || (this.isElevatorLimitSwitchBottom() && this.elevatorSpeed < 0)) {
            this.elevatorSpeed = 0;
        }
        this.setElevatorMotorMoveVoltage(this.elevatorSpeed);
    }

    private boolean isAtPIDGoal() {
        return this.elevatorController.atGoal();
    }

    public void stopElevatorMotor() {
        this.elevatorMotor.set(0);
    }

    public double getElevatorDecelerateRatio() {
        return 1
                - ((this.getElevatorMotorEncoderPosition())
                        / (this.context.getLevel4Position() + this.context.getElevatorDecelerationOffset()));
    }

    public boolean isElevatorAtPosition() {
        return this.getElevatorMotorEncoderPosition() > this.context.getElevatorCeiling();
    }

    public Command getResetPositionCommand() {
        return this.runOnce(this::resetPosition);
    }

    public Command getElevatorUpLimitCommand() {
        return this.runEnd(this::setElevatorMotorUp, this::stopElevatorMotor).until(this::isElevatorLimitSwitchTop);
    }

    public Command getElevatorDownLimitCommand() {
        return this.runEnd(this::advanceElevatorMotorDown, this::stopElevatorMotor)
                .until(this::isElevatorLimitSwitchBottom);
    }

    public Command getElevatorUpCommand() {
        return this.runEnd(this::setElevatorMotorUp, this::stopElevatorMotor);
    }

    public void initElevatorPID() {
        this.elevatorController.reset(this.getElevatorMotorEncoderPosition());
    }

    public void setElevatorPIDPosition(double position) {
        this.elevatorPosition = position;
    }

    @Override
    public void periodic() {
        updatePIDController(this.elevatorPosition);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(
                getName() + "ElevatorCommand/Command/elevatorSpeedInVolts", () -> this.elevatorSpeed, null);
        builder.addDoubleProperty(
                getName() + "ElevatorCommand/Command/elevatorDesirePIDPos", () -> this.elevatorPosition, null);
        builder.addDoubleProperty("Elevator/Position", this::getElevatorMotorEncoderPosition, null);
        builder.addBooleanProperty("Elevator/LimitSwitchTop", this::isElevatorLimitSwitchTop, null);
        builder.addBooleanProperty("Elevator/LimitSwitchBottom", this::isElevatorLimitSwitchBottom, null);
        builder.addBooleanProperty("Elevator/AtPos", this::isElevatorAtPosition, null);
        builder.addBooleanProperty("Elevator/AtPIDGoal", this::isAtPIDGoal, null);
        builder.addDoubleProperty("Elevator/DecelerateRatio", this::getElevatorDecelerateRatio, null);
    }
}
