package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.*;
import frc.robot.support.DIOChannel;
import frc.robot.support.RobotVersion;
import java.util.Arrays;
import java.util.List;

public final class Constants {

    public static final class Limelights {
        public static final String LIMELIGHT_FRONT_LEFT = "limelight-front-left";
        public static final String LIMELIGHT_FRONT_RIGHT = "limelight-front-right";
        public static final String LIMELIGHT_BACK = "limelight-back";
    }

    public static final class DriverLabels {
        public static final String ASA = "Asa";
        public static final String BEN = "Ben";
    }

    public static class Drive {
        // (x, y) position of each module relative to the robot center (center of rotation)
        public static final Translation2d SMFrontRightLocation = new Translation2d(0.285, -0.285);
        public static final Translation2d SMFrontLeftLocation = new Translation2d(0.285, 0.285);
        public static final Translation2d SMBackLeftLocation = new Translation2d(-0.285, 0.285);
        public static final Translation2d SMBackRightLocation = new Translation2d(-0.285, -0.285);

        // we do conversion in limelight. would normally tell robot where the camera
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    }

    public static class Conversion {
        public static final double driveEncoderCtsperRev = 6.8;
        public static final double kWheelDiameterM = Inches.of(4).in(Meters);
        public static final double wheelRadius = kWheelDiameterM / 2.0;
        public static final double kWheelCircumference = Math.PI * kWheelDiameterM;
        public static final double NeoEncoderCountsPerRev = 42;
        public static final double NeoRevPerEncoderCounts = 1 / NeoEncoderCountsPerRev;
        public static final double NeoMaxSpeedRPM = 5820;
        public static final double MagEncoderCountsPerRev = 4096;
        public static final double MagRevPerEncoderCounts = 1 / MagEncoderCountsPerRev;
        public static final double DriveGearRatio = 8.14;
        public static final double TurnGearRatio = 12.8;
        public static final double driveEncoderConversion = DriveGearRatio * kWheelCircumference;
    }

    public static class Controller {
        public static final int DRIVER_CONTROLLER_CHANNEL = 0;
        public static final int MANIPULATION_CONTROLLER_CHANNEL = 1;
        public static final int DEBUG_CONTROLLER_CHANNEL = 2;
        public static final int buttonA = 1;
        public static final int buttonB = 2;
        public static final int buttonX = 3;
        public static final int buttonY = 4;
        public static final int buttonLeft = 5;
        public static final int buttonRight = 6;
        public static final int buttonOptions = 7;
        public static final int buttonStart = 8;
        public static final int buttonLS = 9;
        public static final int buttonRS = 10;
        public static final double deadzone = 0.17;
        public static final double RTdeadzone = .01;
    }

    public static class AprilTagID {
        public static final int PracticeSpeakerCenter = 1;
        public static final int BlueSpeakerCenter = 7;
        public static final int RedSpeakerCenter = 4;

        public static final int BlueStageCenter = 14;
        public static final int RedStageCenter = 13;

        public static final int BlueStageLeft = 15;
        public static final int RedStageRight = 12;

        public static final int RedStageLeft = 11;
        public static final int BlueStageRight = 16;

        public static final Pose2d BlueSpeakerCenterPose = new Pose2d(); // TODO
        public static final Pose2d RedSpeakerCenterPose = new Pose2d(); // TODO

        public static final PathConstraints pathConstraints = new PathConstraints(2, 3, 360, 540);
    }

    public static class Port {
        public static final int REAR_LEFT_TURN_CHANNEL = 1;
        public static final int REAR_LEFT_DRIVE_CHANNEL = 2;
        public static final int FRONT_LEFT_DRIVE_CHANNEL = 3;
        public static final int FRONT_LEFT_STEER_CHANNEL = 4;
        public static final int FRONT_RIGHT_STEER_CHANNEL = 5;
        public static final int FRONT_RIGHT_DRIVE_CHANNEL = 6;
        public static final int REAR_RIGHT_DRIVE_CHANNEL = 7;
        public static final int REAR_RIGHT_STEER_CHANNEL = 8;
        public static final int ELEVATOR_DRIVE_CHANNEL = 11;
        public static final int CLIMB_DRIVE_CHANNEL = 12;
        public static final int RIGHT_CORAL_DRIVE_CHANNEL = 13;
        public static final int LEFT_CORAL_DRIVE_CHANNEL = 14;
        public static final int ELEVATOR_FOLLOWER_DRIVE_CHANNEL = 15;
        public static final int REAR_LEFT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.ZERO.getChannel();
        public static final int FRONT_LEFT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.ONE.getChannel();
        public static final int FRONT_RIGHT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.TWO.getChannel();
        public static final int REAR_RIGHT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.THREE.getChannel();
        public static final int climbMagSwitchDIOC = DIOChannel.FOUR.getChannel();
        ;
        public static final int hangerLeftMagSwitchDIOC = DIOChannel.SEVEN.getChannel();
        public static final int hangerRightMagSwitchDIOC = DIOChannel.EIGHT.getChannel();
        ;
        public static final int PHChannel = 30; // REV Pneumatic Hub
        public static final int PDHChannel = 20; // REV Power Distribution Hub
    }

    public static class Algae {
        public static final int intakeMotorChannel = 9;
        public static final int m_AlgaeMtrC = 10;
    }

    public abstract class RobotVersionConstants {
        public static final double flTurnEncoderOffset = 0;
        public static final double frTurnEncoderOffset = 0;
        public static final double blTurnEncoderOffset = 0;
        public static final double brTurnEncoderOffset = 0;
    }

    public class RobotVersion2025 extends RobotVersionConstants {
        public static final double flTurnEncoderOffset = 3.84 - .04 + Math.PI;
        public static final double frTurnEncoderOffset = 1.7 + Math.PI - .03 + Math.PI;
        public static final double rlTurnEncoderOffset = 3.284 + Math.PI;
        public static final double rrTurnEncoderOffset = 4.49 + Math.PI;
    }

    public class RobotVersion2023 extends RobotVersionConstants {
        public static final double flTurnEncoderOffset = 5.3038;
        public static final double frTurnEncoderOffset = Math.PI / 2 - 0.1242 - .05759;
        public static final double rlTurnEncoderOffset = 4.2 + 0.0385;
        public static final double rrTurnEncoderOffset = 2.736 - .06098;
    }

    public static final class Poses {
        public static final Pose2d SeventeenLeft = new Pose2d(3.824, 2.904, new Rotation2d(Math.toRadians(60)));
        public static final Pose2d SeventeenRight = new Pose2d(4.19, 2.78, new Rotation2d(Math.toRadians(60)));
        public static final Pose2d EighteenLeft = new Pose2d(3.22, 4.06, new Rotation2d(Math.toRadians(0)));
        public static final Pose2d EighteenRight = new Pose2d(3.22, 3.76, new Rotation2d(Math.toRadians(0)));
        public static final Pose2d NineteenLeft = new Pose2d(3.9, 5.15, new Rotation2d(Math.toRadians(-60)));
        public static final Pose2d NineteenRight = new Pose2d(3.73, 4.92, new Rotation2d(Math.toRadians(-60)));
        public static final Pose2d TwentyLeft = new Pose2d(5.18, 5.11, new Rotation2d(Math.toRadians(-120)));
        public static final Pose2d TwentyRight = new Pose2d(4.82, 5.23, new Rotation2d(Math.toRadians(-120)));
        public static final Pose2d TwentyOneLeft = new Pose2d(5.77, 3.98, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d TwentyOneRight = new Pose2d(5.78, 4.36, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d TwentyTwoLeft =
                new Pose2d(5.13 /* adding 9 here */, 2.9, new Rotation2d(Math.toRadians(120)));
        public static final Pose2d TwentyTwoRight = new Pose2d(5.34, 3.13, new Rotation2d(Math.toRadians(120)));
        public static final Pose2d SixLeft = FlippingUtil.flipFieldPose(TwentyTwoLeft);
        public static final Pose2d SixRightChanged = new Pose2d(13.94, 3.08, new Rotation2d(120));
        public static final Pose2d TwentyTwoRightChanged = FlippingUtil.flipFieldPose(SixRightChanged);
        public static final Pose2d SixRight = FlippingUtil.flipFieldPose(TwentyTwoRight);
        public static final Pose2d SevenLeft = FlippingUtil.flipFieldPose(TwentyOneLeft);
        public static final Pose2d SevenRight = FlippingUtil.flipFieldPose(TwentyOneRight);
        public static final Pose2d EightLeft = FlippingUtil.flipFieldPose(TwentyLeft);
        public static final Pose2d EightRight = FlippingUtil.flipFieldPose(TwentyRight);
        public static final Pose2d NineLeft = FlippingUtil.flipFieldPose(NineteenLeft);
        public static final Pose2d NineRight = FlippingUtil.flipFieldPose(NineteenRight);
        public static final Pose2d TenLeft = FlippingUtil.flipFieldPose(EighteenLeft);
        public static final Pose2d TenRight = FlippingUtil.flipFieldPose(EighteenRight);
        public static final Pose2d ElevenLeft = FlippingUtil.flipFieldPose(SeventeenLeft);
        public static final Pose2d ElevenRight = FlippingUtil.flipFieldPose(SeventeenRight);
        public static List<Pose2d> PositionsRed = Arrays.asList(
                SixLeft,
                SixRightChanged,
                SevenLeft,
                SevenRight,
                EightLeft,
                EightRight,
                NineLeft,
                NineRight,
                TenLeft,
                TenRight,
                ElevenLeft,
                ElevenRight,
                SeventeenLeft,
                SeventeenRight,
                EighteenLeft,
                EighteenRight,
                NineteenLeft,
                NineteenRight,
                TwentyLeft,
                TwentyRight,
                TwentyOneLeft,
                TwentyOneRight,
                TwentyTwoLeft,
                TwentyTwoRightChanged);
    }

    public static final RobotVersion defaultRobotVersion = RobotVersion.v2025;
}
