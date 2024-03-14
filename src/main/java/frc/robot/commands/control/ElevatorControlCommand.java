package frc.robot.commands.control;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorControlCommand extends Command {
    private ElevatorSubsystem elevator;
    private XboxController xbox;

    private double elevatorPos = 0;

    public ElevatorControlCommand(ElevatorSubsystem elevator, XboxController xbox) {
        this.elevator = elevator;
        this.xbox = xbox;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevatorPos += xbox.getLeftY() * 0.3 * 0.02;
        elevatorPos = Math.min(elevatorPos, 1);
        elevatorPos = Math.max(elevatorPos, 0);
        elevator.setHeight(elevatorPos);
        SmartDashboard.putNumber("Elevator Target Pos", elevatorPos);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.hold();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
