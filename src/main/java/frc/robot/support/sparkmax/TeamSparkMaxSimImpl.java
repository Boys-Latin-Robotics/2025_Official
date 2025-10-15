package frc.robot.support.sparkmax;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.system.plant.DCMotor;

public class TeamSparkMaxSimImpl extends TeamSparkMaxImpl {

    private final SparkMaxSim sparkMaxSim;

    public TeamSparkMaxSimImpl(int channel, SparkLowLevel.MotorType type) {
        super(channel, type);
        this.sparkMaxSim = new SparkMaxSim(this.sparkMax, DCMotor.getNEO(1));
    }

    @Override
    public double getAppliedOutput() {
        return this.sparkMaxSim.getAppliedOutput();
    }

    @Override
    public double getPosition() {
        return this.sparkMaxSim.getRelativeEncoderSim().getPosition();
    }

    @Override
    public double getVelocity() {
        return this.sparkMaxSim.getRelativeEncoderSim().getVelocity();
    }
}
