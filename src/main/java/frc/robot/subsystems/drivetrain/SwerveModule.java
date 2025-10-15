// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.Conversion.NeoMaxSpeedRPM;
import static frc.robot.Constants.Conversion.TurnGearRatio;
import static java.lang.String.format;
import static java.util.Objects.requireNonNull;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.support.PIDSettings;
import frc.robot.support.sparkmax.TeamSparkMax;

/**
 * This is the code to run a single swerve module. SwerveModules have a turning motor, a drive motor, and associated
 * turning and drive encoders. It is called by the Drivetrain subsystem
 */
public class SwerveModule extends SubsystemBase {

    private static final double TOTAL_ROTATIONAL_RANGE = 2 * Math.PI;

    private static final double POSITION_CONVERSION_FACTOR =
            (Constants.Conversion.kWheelDiameterM * Math.PI) / Constants.Conversion.DriveGearRatio;

    private static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;

    public static final double DRIVE_MAX_SPEED = Units.feetToMeters(12.5);

    // meters per second or 12.1 ft/s (max speed of SDS Mk3 with Neo motor)
    // TODO KMaxSpeed needs to go with enum
    private static final double MAX_ANGULAR_SPEED =
            Units.rotationsPerMinuteToRadiansPerSecond(NeoMaxSpeedRPM / TurnGearRatio); // 1/2

    private static final double MAX_ANGULAR_VELOCITY = MAX_ANGULAR_SPEED;

    // radians per second squared
    private static final double MODULE_MAX_ANGULAR_ACCELERATION = TOTAL_ROTATIONAL_RANGE;

    private static final int TURNING_MOTOR_ASSUMED_FREQUENCY = 242;

    private final SwerveModuleContext context;

    private final TeamSparkMax driveMotor;

    private final TeamSparkMax turningMotor;

    private final DutyCycleEncoder turningMotorEncoder;

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController turningController = new ProfiledPIDController(
            1, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MODULE_MAX_ANGULAR_ACCELERATION));

    // Retain our last desired state to support simulation
    private SwerveModuleState lastDesiredState = new SwerveModuleState();

    private double lastDrivePercent = 0.0;
    private double lastTurnPercent = 0.0;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     */
    public SwerveModule(final SwerveModuleContext context) {
        requireNonNull(context, "SwerveModuleContext cannot be null");

        this.context = context;

        this.setName(this.context.getName());

        this.driveMotor = this.context.getDriveMotor();

        // PWM encoder from CTRE mag encoders
        this.turningMotor = this.context.getTurningMotor();

        this.turningMotorEncoder = new DutyCycleEncoder(
                this.context.getTurnEncoderPWMChannel(), TOTAL_ROTATIONAL_RANGE, this.context.getTurnOffset());

        this.turningMotorEncoder.setAssumedFrequency(TURNING_MOTOR_ASSUMED_FREQUENCY);

        this.driveMotor.configure(
                this.assembleDriveMotorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Limit the PID Controller's input range between -pi and pi and set the input to be continuous.
        this.turningController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Prepares and configures a {@link SparkMaxConfig} to be applied to this swerve module's drive motor
     *
     * @return The SparkMaxConfig
     */
    private SparkMaxConfig assembleDriveMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true).idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        PIDSettings pidSettings = this.context.getDriveMotorPIDSettings();
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(pidSettings.p(), pidSettings.i(), pidSettings.d());

        return config;
    }

    /**
     * Returns the current state of the module. <pi> This takes a current velocity for each different drive encoder and
     * a current angle.
     *
     * @return The current state of each Swerve Module. --> The speed and angle of a Module
     */
    public SwerveModuleState getModuleState() {
        // the getVelocity() function normally returns RPM but is scaled in the
        // SwerveModule constructor to return actual wheel speed

        return new SwerveModuleState(
                this.driveMotor.getEncoder().getVelocity(), Rotation2d.fromRadians(this.turningMotorEncoder.get()));
    }

    /**
     * Sets the desired state for the module. This implementation will also retain the desired state internally in
     * support for simulation. See {@link SwerveModule#getLastDesiredState}
     *
     * <p>
     * This means the speed it should be going and the angle it should be going.
     *
     * @param desiredState
     *            Desired state with speed and angle.
     */
    public void setDesiredState(final SwerveModuleState desiredState) {

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(getModulePosition().angle);

        final double signedAngleDifference =
                closestAngleCalculator(this.getModulePosition().angle.getRadians(), desiredState.angle.getRadians());

        // proportion error control
        double rotateMotorPercentPower = signedAngleDifference / TOTAL_ROTATIONAL_RANGE;

        double turnMotorPercentPower = this.context.getRotationalProportionalGain() * rotateMotorPercentPower;
        this.turningMotor.set(turnMotorPercentPower);

        double driveMotorPercentPower = desiredState.speedMetersPerSecond / DRIVE_MAX_SPEED;
        this.driveMotor.set(driveMotorPercentPower);

        this.refreshSmartDashboard(
                driveMotorPercentPower,
                turnMotorPercentPower,
                signedAngleDifference,
                desiredState.angle.getRadians(),
                this.getModulePosition().angle.getRadians());

        this.lastDesiredState = desiredState;

        this.lastDrivePercent = driveMotorPercentPower;
        this.lastTurnPercent = turnMotorPercentPower;
    }

    public double getLastDrivePercent() {
        return lastDrivePercent;
    }

    public double getLastTurnPercent() {
        return lastTurnPercent;
    }

    /**
     * This gets a current Position (Distance per rotation in meters) for each different drive encoder and a current
     * angle from the Duty Cycle encoder.
     *
     * @return The current Position of each Swerve Module
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                this.driveMotor.getEncoder().getPosition(), Rotation2d.fromRadians(this.turningMotorEncoder.get()));
    }

    /**
     * Obtains the last desired state that has been assigned
     *
     * @return SwerveModuleState The last desired state
     */
    public SwerveModuleState getLastDesiredState() {
        return lastDesiredState;
    }

    /**
     * Sends the specified values to the {@link SmartDashboard} under predefined keys
     *
     * @param driveMotorPercentPower
     * @param turnMotorPercentPower
     * @param signedAngleDifference
     * @param desiredAngle
     * @param currentAngle
     */
    private void refreshSmartDashboard(
            double driveMotorPercentPower,
            double turnMotorPercentPower,
            double signedAngleDifference,
            double desiredAngle,
            double currentAngle) {

        String nameTag = format("DriveTrain/%s", this.getName());
        int driveMotorId = this.driveMotor.getDeviceId();
        int turningMotorId = this.turningMotor.getDeviceId();

        // TODO: Improve and cleanup these names
        SmartDashboard.putNumber(
                format("%s/Drive Encoder/ID:%s/DrivePercent", nameTag, driveMotorId), driveMotorPercentPower);

        SmartDashboard.putNumber(
                format("%s/Turn Encoder/ID:%s/DrivePercent", nameTag, turningMotorId), turnMotorPercentPower);

        SmartDashboard.putNumber(
                format("%s/Turn Encoder/SignedAngleDiff:%s/Angle", nameTag, turningMotorId), signedAngleDifference);

        SmartDashboard.putNumber(
                format("%s/Turn Encoder/DesiredState:%s/DesiredAngle", nameTag, turningMotorId), desiredAngle);

        SmartDashboard.putNumber(
                format("%s/Turn Encoder/CurrentState:%s/DesiredAngle", nameTag, turningMotorId), currentAngle);
    }

    /**
     * Calculates the closest angle and direction between two points on a circle.
     *
     * @param currentAngle
     *            <ul>
     *            <li>where you currently are
     *            </ul>
     * @param desiredAngle
     *            <ul>
     *            <li>where you want to end up
     *            </ul>
     * @return
     *         <ul>
     *         <li>signed double of the angle (rad) between the two points
     *         </ul>
     */
    private double closestAngleCalculator(double currentAngle, double desiredAngle) {
        double signedDiff = 0.0;
        // find the positive raw distance between the angles
        double rawDiff = currentAngle > desiredAngle ? currentAngle - desiredAngle : desiredAngle - currentAngle;
        double modDiff = rawDiff % TOTAL_ROTATIONAL_RANGE; // constrain the difference to a full circle
        if (modDiff > Math.PI) { // if the angle is greater than half a rotation, go backwards
            signedDiff = (TOTAL_ROTATIONAL_RANGE - modDiff); // full circle minus the angle
            if (desiredAngle > currentAngle)
                signedDiff = signedDiff * -1; // get the direction that was lost calculating raw diff
        } else {
            signedDiff = modDiff;
            if (currentAngle > desiredAngle) signedDiff = signedDiff * -1;
        }
        return signedDiff;
    }

    /**
     * Tells the drive and turning motor to stop
     */
    public void stopMotors() {
        this.driveMotor.set(0);
        this.turningMotor.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.publishConstInteger("TurnMotor/ID", this.turningMotor.getDeviceId());
        builder.publishConstInteger("DriveMotor/ID", this.driveMotor.getDeviceId());
        builder.addDoubleProperty(
                "TurnMotor/Angle", () -> Units.radiansToDegrees(this.turningMotorEncoder.get()), null);
        builder.addDoubleProperty("DriveMotor/Pos", this.driveMotor::getPosition, null);
        builder.addDoubleProperty("DriveMotor/Vel", this.driveMotor::getVelocity, null);
        builder.addDoubleProperty("TurnMotor/Encoder/AbsolutePosition", this.turningMotorEncoder::get, null);
        builder.addDoubleProperty(
                "TurnMotor/Encoder/TurningEncoderPosition", this.turningMotor.getEncoder()::getPosition, null);
        builder.setSafeState(this::stopMotors);
    }
}
