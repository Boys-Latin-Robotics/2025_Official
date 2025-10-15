package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorPIDCommand extends Command {
    private final Double position;
    private final Elevator elevator;

    public ElevatorPIDCommand(Elevator elevator, Double position) {
        this.elevator = elevator;
        this.position = position;
        addRequirements(this.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.elevator.initElevatorPID();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.elevator.setElevatorPIDPosition(position);
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.elevator.stopElevatorMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
