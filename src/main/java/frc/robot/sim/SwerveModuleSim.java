package frc.robot.sim;

import static edu.wpi.first.math.system.plant.LinearSystemId.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveModuleContext;

public class SwerveModuleSim {
    private final DCMotorSim driveMotorSim;
    private final DCMotorSim turnMotorSim;

    private Rotation2d turnAngle = new Rotation2d();

    private double driveDistanceMeters = 0.0;

    public SwerveModuleSim(final SwerveModuleContext swerveModuleContext) {
        // Approximate drive motor as velocity system
        LinearSystem<N2, N1, N2> drivePlant = createDCMotorSystem(
                DCMotor.getNEO(1), swerveModuleContext.getDriveInertia(), swerveModuleContext.getDriveGearRatio());

        LinearSystem<N2, N1, N2> turnPlant = createDCMotorSystem(
                DCMotor.getNEO(1), swerveModuleContext.getTurnInertia(), swerveModuleContext.getTurnGearRatio());

        this.driveMotorSim = new DCMotorSim(drivePlant, DCMotor.getNEO(1), new double[] {0.0, 0.0});
        this.turnMotorSim = new DCMotorSim(turnPlant, DCMotor.getNEO(1), new double[] {0.0, 0.0});
    }

    public void update(double dtSeconds) {
        driveMotorSim.update(dtSeconds);
        turnMotorSim.update(dtSeconds);

        // Approximate azimuth as integration of turn motor position
        double wheelSpeed = getWheelSpeedMetersPerSecond();
        this.driveDistanceMeters += wheelSpeed * dtSeconds;

        double turnRotations = turnMotorSim.getAngularPositionRotations();
        this.turnAngle = Rotation2d.fromRotations(turnRotations);
    }

    public void setDriveVoltage(double volts) {
        this.driveMotorSim.setInputVoltage(volts);
    }

    public void setTurnVoltage(double volts) {
        this.turnMotorSim.setInputVoltage(volts);
    }

    public double getWheelSpeedMetersPerSecond() {
        // motor angular velocity (rad/sec) to wheel linear speed
        double wheelRadPerSec = driveMotorSim.getAngularVelocityRadPerSec();
        return wheelRadPerSec * Constants.Conversion.wheelRadius;
    }

    public Rotation2d getTurnAngle() {
        return this.turnAngle;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.driveDistanceMeters, this.turnAngle);
    }
}
