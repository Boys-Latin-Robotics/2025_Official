package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.algae.AlgaeMechanism;
import frc.robot.support.Telemetry;
import frc.robot.support.limelight.LimelightUtil;
// some imports no longer needed but leaving them here untill final version

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer = new RobotContainer();
    String codeVersion = "0.0";
    private PowerDistribution PDH = new PowerDistribution(20, PowerDistribution.ModuleType.kRev);
    public static boolean navxCalibrated = false;
    private boolean isConnected = false;

    @Override
    public void close() {
        super.close();

        // TODO: Evaluate port forwarding teardown
        LimelightUtil.stopPortForwarding();

        Telemetry.shutdown();
    }

    // commit
    @Override
    public void robotInit() {
        m_robotContainer.getLimelightBack().SetTagIDToTrack(-1);
        m_robotContainer.getLimelightFrl().SetTagIDToTrack(-1);
        m_robotContainer.getLimelightFrr().SetTagIDToTrack(-1);
        m_robotContainer.getLedStrand().changeLed(128, 0, 0);
        try {
            try (UsbCamera cam = CameraServer.startAutomaticCapture()) {
                cam.setResolution(100, 100);
                cam.setFPS(60);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        SmartDashboard.putString("Code Version", codeVersion);

        // TODO: Evaluate port forwarding setup
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putData(PDH);
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {
        AlgaeMechanism.AUTO_RUNNING = true;
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.getDriveTrain().setFieldRelativeEnable(false);
        System.out.println(m_robotContainer.getDriveTrain().isFieldRelativeEnable());
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        AlgaeMechanism.AUTO_RUNNING = true;
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        // m_robotContainer.m_DriveTrain.ZeroGyro().schedule();
        AlgaeMechanism.AUTO_RUNNING = false;
        m_robotContainer.getDriveTrain().setFieldRelativeEnable(true);
        m_robotContainer.getLimelightBack().SetTagIDToTrack(-1);
        m_robotContainer.getLimelightFrl().SetTagIDToTrack(-1);
        m_robotContainer.getLimelightFrr().SetTagIDToTrack(-1);
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
