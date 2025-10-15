package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Limelights;
import frc.robot.commands.AlgaePIDCommand;
import frc.robot.commands.AprilAlignCommand;
import frc.robot.commands.ElevatorPIDCommand;
import frc.robot.commands.swervedrive.ControllerDelegate;
import frc.robot.commands.swervedrive.SwerveDriveCommand;
import frc.robot.subsystems.LedStrand;
import frc.robot.subsystems.algae.AlgaeIntake;
import frc.robot.subsystems.algae.AlgaeIntakeContext;
import frc.robot.subsystems.algae.AlgaeMechanism;
import frc.robot.subsystems.algae.AlgaeMechanismContext;
import frc.robot.subsystems.climb.ClimbMechanism;
import frc.robot.subsystems.climb.ClimbMechanismContext;
import frc.robot.subsystems.coral.CoralMechanism;
import frc.robot.subsystems.coral.CoralMechanismContext;
import frc.robot.subsystems.coral.Servo;
import frc.robot.subsystems.coral.ServoContext;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainContext;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorContext;
import frc.robot.subsystems.vision.Limelight;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {

    private final Limelight limelightFrl = new Limelight(Limelights.LIMELIGHT_FRONT_LEFT);

    private final Limelight limelightFrr = new Limelight(Limelights.LIMELIGHT_FRONT_RIGHT);

    private final Limelight limelightBack = new Limelight(Limelights.LIMELIGHT_BACK);

    private final LedStrand ledStrand = new LedStrand();

    private final DrivetrainContext drivetrainContext = DrivetrainContext.defaults();

    private final Drivetrain driveTrain = new Drivetrain(drivetrainContext);

    private final CoralMechanism coralMechanism = new CoralMechanism(CoralMechanismContext.defaults());

    private final ClimbMechanism climbMechanism = new ClimbMechanism(ClimbMechanismContext.defaults());

    private final AlgaeMechanismContext algaeMechanismContext = AlgaeMechanismContext.defaults();

    private final AlgaeMechanism algaeMechanism = new AlgaeMechanism(algaeMechanismContext);

    private final AlgaeIntake algaeIntake = new AlgaeIntake(AlgaeIntakeContext.defaults());

    private final ElevatorContext elevatorSettings = ElevatorContext.defaults();

    private final Elevator elevator = new Elevator(elevatorSettings);

    private final ElevatorPIDCommand elevatorPIDCommandDown =
            new ElevatorPIDCommand(this.elevator, elevatorSettings.getDownPosition());
    private final ElevatorPIDCommand elevatorPIDL2Command =
            new ElevatorPIDCommand(this.elevator, elevatorSettings.getLevel2Position());
    private final ElevatorPIDCommand elevatorPIDL3Command =
            new ElevatorPIDCommand(this.elevator, elevatorSettings.getLevel3Position());
    private final ElevatorPIDCommand elevatorPIDL4Command =
            new ElevatorPIDCommand(this.elevator, elevatorSettings.getLevel4Position());
    private final ElevatorPIDCommand elevatorPIDCommandAlgae3 =
            new ElevatorPIDCommand(this.elevator, elevatorSettings.getAlgaeLevel3Position());

    private final Servo servo = new Servo(ServoContext.defaults());

    private final AprilAlignCommand limelightCodeFrontLeft = new AprilAlignCommand(
            this.limelightFrl::getCurrentAprilTag,
            this.limelightFrl::getAprilRotation2d,
            this.driveTrain,
            new Transform2d(.18, 0.00, new Rotation2d(.15)),
            false,
            true,
            this.ledStrand);
    private final AprilAlignCommand limelightCodeFrontRight = new AprilAlignCommand(
            this.limelightFrr::getCurrentAprilTag,
            this.limelightFrr::getAprilRotation2d,
            this.driveTrain,
            new Transform2d(.07, 0.00, new Rotation2d(0.17)),
            false,
            false,
            this.ledStrand);
    private final AprilAlignCommand limelightCodeBack = new AprilAlignCommand(
            this.limelightBack::getCurrentAprilTag,
            this.limelightBack::getAprilRotation2d,
            this.driveTrain,
            new Transform2d(.65, 0.00, new Rotation2d()),
            true,
            false,
            this.ledStrand);
    // Compose the commands correctly, ensuring that each use is a new composition
    private final Command runCoralForward = this.coralMechanism
            .getAdvanceCoralCommand()
            .onlyWhile(() -> !this.coralMechanism.isCoralLoaded())
            .withName("RunCoral");
    private final Command algaeDownAndRunA3 = Commands.race(
            new AlgaePIDCommand(algaeMechanism, algaeMechanismContext.getAlgaeUpPosition()),
            algaeIntake.getAdvanceIntakeOnceCommand(),
            new ElevatorPIDCommand(this.elevator, elevatorSettings.getAlgaeLevel3Position()));
    private final Command algaeDownAndRunA4 = Commands.race(
            new AlgaePIDCommand(algaeMechanism, algaeMechanismContext.getAlgaeUpPosition()),
            algaeIntake.getAdvanceIntakeOnceCommand(),
            new ElevatorPIDCommand(this.elevator, elevatorSettings.getAlgaeLevel4Position()));
    private final Command algaeUpAndStop = Commands.race(
            new AlgaePIDCommand(algaeMechanism, algaeMechanismContext.getAlgaeDownPosition()),
            algaeIntake.getStopIntakeCommand());
    private final Command algaeMiddleAndStop = Commands.race(
            new AlgaePIDCommand(algaeMechanism, algaeMechanismContext.getAlgaeMiddlePosition()),
            algaeIntake.getStopIntakeCommand());
    private final Command algaeGroundCommand = Commands.race(
            new AlgaePIDCommand(algaeMechanism, algaeMechanismContext.getAlgaeGroundPosition()),
            algaeIntake.getReverseIntakeOnceCommand(),
            new ElevatorPIDCommand(this.elevator, elevatorSettings.getLevel2Position()));
    private final Command resetPoseAuto =
            Commands.runOnce(() -> this.driveTrain.resetOdometry(this.currentPath.get(0)), this.driveTrain);

    private final AlgaePIDCommand algaePIDCommandDown =
            new AlgaePIDCommand(algaeMechanism, algaeMechanismContext.getAlgaeUpPosition());
    private final AlgaePIDCommand algaePIDCommandMiddle =
            new AlgaePIDCommand(algaeMechanism, algaeMechanismContext.getAlgaeMiddlePosition());
    private final AlgaePIDCommand algaePIDCommandUp =
            new AlgaePIDCommand(algaeMechanism, algaeMechanismContext.getAlgaeDownPosition());
    private final AlgaePIDCommand algaePIDCommandGround =
            new AlgaePIDCommand(algaeMechanism, algaeMechanismContext.getAlgaeDownPosition());

    /**
     * Creates buttons and controller for: - the driver controller (port 0) - the manipulator controller (port 1) - the
     * debug controller (port 2)
     */
    private final CommandXboxController driverController =
            new CommandXboxController(Constants.Controller.DRIVER_CONTROLLER_CHANNEL);

    private final CommandXboxController manipController =
            new CommandXboxController(Constants.Controller.MANIPULATION_CONTROLLER_CHANNEL);

    private final CommandXboxController debugController =
            new CommandXboxController(Constants.Controller.DEBUG_CONTROLLER_CHANNEL);

    // A chooser for autonomous commands
    private final SendableChooser<Command> autoChooser;

    // Creating 2d field in Sim/ShuffleBoard
    // Trying to get feedback from auto
    private final List<Pose2d> currentPath = new ArrayList<Pose2d>();

    // The constraints for this path.
    public static final PathConstraints SPEED_CONSTRAINTS = new PathConstraints(2, 1.5, 1.5 * Math.PI, 1 * Math.PI);

    public RobotContainer() {
        this.elevator.setName("ElevatorMechanism");
        this.driveTrain.setName("DriveTrain");
        this.coralMechanism.setName("CoralMechanism");
        this.algaeIntake.setName("AlgaeIntake");
        this.algaeMechanism.setName("AlgaeMechanism");

        this.configureShuffleboard();
        this.configureBindings();
        this.registerCommands();

        // Build an auto chooser. This will use Commands.none() as the default option.

        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name:
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Creates a field to be put to the shuffleboard
        SmartDashboard.putData("AUTOPOSITION", (s) -> AutoBuilder.getCurrentPose());
    }

    public Drivetrain getDriveTrain() {
        return driveTrain;
    }

    public Limelight getLimelightFrl() {
        return limelightFrl;
    }

    public Limelight getLimelightFrr() {
        return limelightFrr;
    }

    public Limelight getLimelightBack() {
        return limelightBack;
    }

    public LedStrand getLedStrand() {
        return ledStrand;
    }

    public void periodic() {
        // us trying to set pose for field2d
    }

    private void registerCommands() {
        NamedCommands.registerCommand("Limelight", this.limelightCodeFrontLeft);
        NamedCommands.registerCommand("LimelightSetFirstLeftPriority", this.limelightFrl.getPriorityIDCommand(22, 9));
        NamedCommands.registerCommand("LimelightSetSecondLeftPriority", this.limelightFrl.getPriorityIDCommand(17, 8));
        NamedCommands.registerCommand("LimelightSetSecondLeftPriority2", this.limelightFrl.getPriorityIDCommand(18, 8));
        NamedCommands.registerCommand("LimelightSetFirstRightPriority", this.limelightFrl.getPriorityIDCommand(20, 11));
        NamedCommands.registerCommand("LimelightSetSecondRightPriority", this.limelightFrl.getPriorityIDCommand(19, 6));
        NamedCommands.registerCommand("LimelightBack", this.limelightCodeBack);
        NamedCommands.registerCommand("RobotOrientedLimelight", this.limelightFrl.setLimelightUsageField());
        NamedCommands.registerCommand("SETPOSEfrl", this.resetPoseAuto);
        NamedCommands.registerCommand(
                "PathRESETODMLeft", AutoBuilder.resetOdom(new Pose2d(5.002, 2.806, new Rotation2d(90))));
        NamedCommands.registerCommand(
                "PathRESETODMRight", AutoBuilder.resetOdom(new Pose2d(5.021, 5.253, new Rotation2d(180))));
        NamedCommands.registerCommand(
                "PathRESETODMMiddle", AutoBuilder.resetOdom(new Pose2d(5.802, 3.959, new Rotation2d(180))));
        NamedCommands.registerCommand(
                "ElevatorL2", new ElevatorPIDCommand(this.elevator, elevatorSettings.getLevel2Position()));
        NamedCommands.registerCommand("ElevatorA3", this.algaeDownAndRunA3);
        NamedCommands.registerCommand("ElevatorA4", this.algaeDownAndRunA4);
        NamedCommands.registerCommand("IntakeCoral", this.coralMechanism.getCoralIntakeAutoCommand());
        NamedCommands.registerCommand("ResetOdom", this.driveTrain.getResetOdometryCommand());
        NamedCommands.registerCommand(
                "ElevatorL4",
                new ElevatorPIDCommand(this.elevator, elevatorSettings.getLevel4Position())
                        .onlyWhile(() -> !this.elevator.isElevatorAtPosition()));
        NamedCommands.registerCommand(
                "ElevatorBottom", new ElevatorPIDCommand(this.elevator, elevatorSettings.getDownPosition()));
        NamedCommands.registerCommand("ElevatorUp", this.elevator.getElevatorUpCommand());
        NamedCommands.registerCommand(
                "ShootCoral", this.coralMechanism.getAdvanceCoralCommand().withTimeout(.5));
        NamedCommands.registerCommand("ToggleFieldRelative", this.driveTrain.getToggleFieldRelativeCommand());
        NamedCommands.registerCommand(
                "WaitUntilElevatorTop", new WaitUntilCommand(this.elevator::isElevatorAtPosition));
        NamedCommands.registerCommand("StopDrive", this.driveTrain.getStopModulesCommand());
    }

    /**
     * Creates Command Bindings. Read description down below:
     *
     * <p>
     * Use this method to define your trigger->comand mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
     * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        /**
         * Swerve Drive Controller Command
         *
         * <p>
         * Controls: - Left Stick: Steering - Right Stick: Rotate the robot - Right Trigger: provide gas - Left Trigger:
         * reduce maximum driving speed by 50% RECOMMENDED TO USE
         */
        this.driveTrain.setDefaultCommand(new SwerveDriveCommand(
                ControllerDelegate.builder()
                        .leftXSupplier(this.driverController::getLeftX)
                        .leftYSupplier(this.driverController::getLeftY)
                        .rightXSupplier(this.driverController::getRightX)
                        .rightYSupplier(this.driverController::getRightY)
                        .accelerationSupplier(this.driverController::getRightTriggerAxis)
                        .elevatorDecelerateRatioSupplier(this.elevator::getElevatorDecelerateRatio)
                        .runHalfSpeedConditionSupplier(() -> driverController.getLeftTriggerAxis() >= 0.5)
                        .driver(ControllerDelegate.Driver.ASA)
                        .build(),
                driveTrain));

        // Driver Controller commands
        // - DriveTrain commands (outside of actual driving)
        this.driverController.a().whileTrue(this.limelightCodeBack);
        this.driverController.rightStick().onTrue(this.driveTrain.toggleWheelLockCommand()); // lock wheels
        this.driverController.x().whileTrue(this.limelightCodeFrontLeft);
        this.driverController.y().whileTrue(this.limelightCodeFrontRight);
        this.driverController.b().onTrue(this.driveTrain.resetNavXSensorModule());

        // Elevator Commands
        this.manipController.a().onTrue(this.elevatorPIDCommandDown);
        this.manipController.b().onTrue(this.elevatorPIDL2Command);
        this.manipController.y().onTrue(this.elevatorPIDL3Command);
        this.manipController.x().onTrue(this.elevatorPIDL4Command);
        this.manipController.rightStick().onTrue(this.elevator.getResetPositionCommand());
        // Algae Commands
        this.manipController.leftBumper().onTrue(this.algaeDownAndRunA3);
        this.manipController.rightBumper().onTrue(this.algaeDownAndRunA4);
        this.manipController.back().onTrue(this.algaeUpAndStop);
        this.manipController.start().whileTrue(this.algaeIntake.getReverseIntakeCommand());
        this.manipController.povLeft().onTrue(this.algaeGroundCommand);
        this.manipController.povRight().whileTrue(this.algaeIntake.getAdvanceIntakeCommand());
        this.manipController.leftStick().onTrue(this.algaeMiddleAndStop);

        // Coral Commands
        this.manipController.leftTrigger(.5).whileTrue(this.runCoralForward);
        this.manipController.rightTrigger(.5).whileTrue(this.coralMechanism.getAdvanceCoralCommand());
        this.manipController.povUp().whileTrue(this.coralMechanism.getCoralTroughCommand());
        this.manipController.povDown().whileTrue(this.climbMechanism.getFullyReverseClimbCommand());
        this.servo.setDefaultCommand(this.servo.getForwardPositionCommand());

        this.debugController.rightBumper().whileTrue(this.elevator.getElevatorDownLimitCommand());
        this.debugController.leftBumper().whileTrue(this.elevator.getElevatorUpLimitCommand());
        this.debugController.povUp().onTrue(this.algaeMechanism.getResetAlgaeCommand());
        this.debugController.a().onTrue(this.algaePIDCommandUp);
        this.debugController.b().onTrue(this.algaePIDCommandMiddle);
        this.debugController.x().onTrue(this.algaePIDCommandDown);
        this.debugController.y().onTrue(this.algaePIDCommandGround);

        this.debugController.rightStick().onTrue(this.algaeDownAndRunA4);
        this.debugController.povDown().onTrue(this.elevatorPIDCommandAlgae3);
        this.debugController.leftStick().whileTrue(this.algaeIntake.getReverseIntakeCommand());
        this.debugController.start().onTrue(this.algaeMiddleAndStop);
        this.debugController.leftTrigger(.5).onTrue(this.algaeGroundCommand);
    }

    private void configureShuffleboard() {
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData(this.elevator);

        // Add subsystems
        SmartDashboard.putData(this.driveTrain);
        SmartDashboard.putData(this.driveTrain.getName() + "/Reset Pose 2D", this.driveTrain.getResetOdometryCommand());
        SmartDashboard.putData(this.coralMechanism);
        ;
        SmartDashboard.putData(this.algaeMechanism);
        SmartDashboard.putData(this.algaeIntake);
    }

    // loads New Auto auto file
    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }
}
