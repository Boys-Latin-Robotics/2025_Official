package frc.robot.support.sparkmax;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

public class TeamSparkMaxImpl implements TeamSparkMax {

    protected final SparkMax sparkMax;

    public TeamSparkMaxImpl(int channel, SparkLowLevel.MotorType type) {
        this.sparkMax = new SparkMax(channel, type);
    }

    @Override
    public REVLibError configure(
            SparkBaseConfig config, SparkBase.ResetMode resetMode, SparkBase.PersistMode persistMode) {
        return this.sparkMax.configure(config, resetMode, persistMode);
    }

    @Override
    public void set(double speed) {
        this.sparkMax.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        this.sparkMax.setVoltage(voltage);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.sparkMax.setInverted(inverted);
    }

    @Override
    public void stopMotor() {
        this.sparkMax.stopMotor();
    }

    @Override
    public int getDeviceId() {
        return this.sparkMax.getDeviceId();
    }

    @Override
    public RelativeEncoder getEncoder() {
        return this.sparkMax.getEncoder();
    }

    @Override
    public RelativeEncoder getAlternateEncoder() {
        return this.sparkMax.getAlternateEncoder();
    }

    @Override
    public double getAppliedOutput() {
        return this.sparkMax.getAppliedOutput();
    }

    @Override
    public double getPosition() {
        return this.sparkMax.getEncoder().getPosition();
    }

    @Override
    public double getVelocity() {
        return this.sparkMax.getEncoder().getVelocity();
    }
}
