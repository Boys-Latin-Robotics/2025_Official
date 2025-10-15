package frc.robot.support.sparkmax;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;

public interface TeamSparkMax {

    REVLibError configure(SparkBaseConfig config, SparkBase.ResetMode resetMode, SparkBase.PersistMode persistMode);

    int getDeviceId();

    RelativeEncoder getEncoder();

    RelativeEncoder getAlternateEncoder();

    double getAppliedOutput();

    double getPosition();

    double getVelocity();

    void set(double speed);

    void setVoltage(double voltage);

    void setInverted(boolean inverted);

    void stopMotor();
}
