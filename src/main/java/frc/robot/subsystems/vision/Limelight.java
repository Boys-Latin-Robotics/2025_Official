package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Limelights;
import frc.robot.support.limelight.LimelightHelpers;

public class Limelight extends SubsystemBase {
    private final DoubleArraySubscriber aprilTagPoseTopic;
    private final IntegerPublisher priorityTagIdPub;
    private final String limelightName;
    private AprilTag currentAprilTag = new AprilTag(-1, new Pose3d());
    private double priorityID;

    public Limelight() {
        this("limelight");
    }

    public Limelight(final String limelightName) {
        this.limelightName = limelightName;
        NetworkTable table = NetworkTableInstance.getDefault().getTable(this.limelightName);
        this.aprilTagPoseTopic =
                table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] {0, 0, 0, 0, 0, 0});
        this.priorityTagIdPub = table.getIntegerTopic("priorityid").publish();
        this.priorityID = table.getEntry("priorityID").getDouble(-1); // Default to -1 if no value is found
    }

    public void SetTagIDToTrack(int tagID) {
        this.priorityTagIdPub.accept(tagID);
    }

    @Override
    public void periodic() {
        this.currentAprilTag = getCurrentAprilTag();
        SmartDashboard.putNumber("AprilTag/tagID", this.currentAprilTag.ID);
        SmartDashboard.putNumber("AprilTag/pose/X", this.currentAprilTag.pose.getX());
        SmartDashboard.putNumber("AprilTag/pose/Y", this.currentAprilTag.pose.getY());
        SmartDashboard.putNumber("AprilTag/pose/Z", this.currentAprilTag.pose.getZ());
        SmartDashboard.putNumber(
                "AprilTag/pose/rotX",
                Math.toDegrees(this.currentAprilTag.pose.getRotation().getX()));
        SmartDashboard.putNumber(
                "AprilTag/pose/rotY",
                Math.toDegrees(this.currentAprilTag.pose.getRotation().getY()));
        SmartDashboard.putNumber(
                "AprilTag/pose/rotZ",
                Math.toDegrees(this.currentAprilTag.pose.getRotation().getZ()));
        SmartDashboard.putNumber(
                "AprilTag/pose/measureRotZ",
                this.currentAprilTag.pose.getRotation().getMeasureZ().magnitude());
        SmartDashboard.putNumber(
                "AprilTag/pose/GetAngle",
                this.currentAprilTag.pose.getRotation().getAngle());
        SmartDashboard.putNumber(
                "AprilTag/pose/GetAxis",
                this.currentAprilTag.pose.getRotation().getAxis().get(2));
        // SmartDashboard.putNumber("AprilTag/pose/RAWz", aprilTagPoseTopic.getAtomic().value[5]);
        SmartDashboard.putNumber(
                "AprilTag/pose/RAWZFromMethod", this.getAprilRotation2d().getDegrees());
    }

    /**
     * This function gets the april tag the camera is viewing.
     *
     * @return The AprilTag the camera is looking at plus a Pose3d - x, y, z, ROTx, ROTy, ROTz.
     */
    public AprilTag getCurrentAprilTag() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(this.limelightName);

        NetworkTableEntry tid = table.getEntry("tid");
        int aprilTagId = (int) tid.getInteger(-1);
        TimestampedDoubleArray poseArray = this.aprilTagPoseTopic.getAtomic(); // (x, y, z, rotx, roty, rotz)

        if (poseArray.value.length < 6) return new AprilTag(-1, new Pose3d());

        Translation3d poseTranslation = new Translation3d(
                poseArray.value[0], // x
                poseArray.value[1], // y
                poseArray.value[2] // z
                );

        Rotation3d poseOrientation = new Rotation3d(
                poseArray.value[3], // roll = rotx
                poseArray.value[4], // pitch = roty
                poseArray.value[5] // yaw = rotz
                );

        Pose3d aprilTagPose = new Pose3d(poseTranslation, poseOrientation); // creating pose3d based off of our
        // translation3d and rot3d and tid
        return new AprilTag(aprilTagId, aprilTagPose);
    }

    /**
     * This function gets the raw Rotation3d values
     *
     * @return The Rotation2d of aprilTag
     */
    public Rotation2d getAprilRotation2d() {
        TimestampedDoubleArray poseArray = this.aprilTagPoseTopic.getAtomic(); // (x, y, z, rotx, roty, rotz)
        return new Rotation2d(Math.toRadians(poseArray.value[5]));
    }

    public Command setLimelightUsageField() {
        return Commands.runOnce(
                () -> LimelightHelpers.setCameraPose_RobotSpace(Limelights.LIMELIGHT_FRONT_LEFT, 0, 0, 0, 0, -90, 0));
    }

    public Command setLimelightUsageRobot() {
        return Commands.runOnce(
                () -> LimelightHelpers.setCameraPose_RobotSpace(Limelights.LIMELIGHT_FRONT_LEFT, .17, 0, .2, 0, 0, 0));
    }

    public Command getPriorityIDCommand(int idBlue, int idRed) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return this.runOnce(() -> this.SetTagIDToTrack(idRed));

        } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            return this.runOnce(() -> this.SetTagIDToTrack(idBlue));
        } else {
            return this.runOnce(() -> this.SetTagIDToTrack(-1));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("AprilTag/tagID", () -> this.currentAprilTag.ID, null);
        builder.addDoubleProperty("AprilTag/pose/X", this.currentAprilTag.pose::getX, null);
        builder.addDoubleProperty("AprilTag/pose/Y", this.currentAprilTag.pose::getY, null);
        builder.addDoubleProperty("AprilTag/pose/Z", this.currentAprilTag.pose::getZ, null);
        builder.addDoubleProperty("AprilTag/pose/rotX", this.currentAprilTag.pose.getRotation()::getX, null);
        builder.addDoubleProperty("AprilTag/pose/rotY", this.currentAprilTag.pose.getRotation()::getY, null);
        builder.addDoubleProperty("AprilTag/pose/rotZ", this.currentAprilTag.pose.getRotation()::getZ, null);
        builder.addDoubleProperty("AprilTag/PriorityID", () -> this.priorityID, null);
    }
}
