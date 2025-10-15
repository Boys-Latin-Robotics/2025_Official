package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Builder;

/**
 * ControllerDelegate encapsulates a series of {@link java.util.function.Supplier} which produce values generated from
 * driver controls, such as {@link CommandXboxController}. ControllerDelegate also maintains {@link Driver} to identify
 * the currently operating Driver. In this way supplied values can be interpreted according to driver preference.
 */
@Builder
public class ControllerDelegate {
    private final DoubleSupplier leftXSupplier;
    private final DoubleSupplier leftYSupplier;
    private final DoubleSupplier rightXSupplier;
    private final DoubleSupplier rightYSupplier;
    private final DoubleSupplier accelerationSupplier;
    private final DoubleSupplier elevatorDecelerateRatioSupplier;
    private final BooleanSupplier runHalfSpeedConditionSupplier;
    private final Driver driver;

    public ControllerDelegate(
            final DoubleSupplier leftXSupplier,
            final DoubleSupplier leftYSupplier,
            final DoubleSupplier rightXSupplier,
            final DoubleSupplier rightYSupplier,
            final DoubleSupplier accelerationSupplier,
            final DoubleSupplier elevatorDecelerateRatioSupplier,
            final BooleanSupplier runHalfSpeedConditionSupplier,
            final Driver driver) {
        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;
        this.rightXSupplier = rightXSupplier;
        this.rightYSupplier = rightYSupplier;
        this.accelerationSupplier = accelerationSupplier;
        this.elevatorDecelerateRatioSupplier = elevatorDecelerateRatioSupplier;
        this.runHalfSpeedConditionSupplier = runHalfSpeedConditionSupplier;
        this.driver = driver;
    }

    public void logRawAxes() {
        for (int i = 0; i < 10; i++) {
            double value = edu.wpi.first.wpilibj.DriverStation.getStickAxis(0, i);
            System.out.println("Axis[" + i + "]: " + value);
        }
    }

    public double getLeftX() {
        return this.leftXSupplier.getAsDouble();
    }

    public double getLeftY() {
        return this.leftYSupplier.getAsDouble();
    }

    public double getRightX() {
        return this.rightXSupplier.getAsDouble();
    }

    public double getRightY() {
        return this.rightYSupplier.getAsDouble();
    }

    public double getAcceleration() {
        return this.accelerationSupplier.getAsDouble();
    }

    public Driver getDriver() {
        return driver;
    }

    public double getElevatorDecelerateRatio() {
        return this.elevatorDecelerateRatioSupplier.getAsDouble();
    }

    public boolean isHalfSpeed() {
        return this.runHalfSpeedConditionSupplier.getAsBoolean();
    }

    /**
     * An enumeration of available drivers and their associated label
     */
    public enum Driver {
        ASA(Constants.DriverLabels.ASA),
        BEN(Constants.DriverLabels.BEN);

        private final String label;

        Driver(final String label) {
            this.label = label;
        }

        public String getLabel() {
            return label;
        }
    }
}
