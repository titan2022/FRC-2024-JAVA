package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorControlCommand extends Command {
    private ElevatorSubsystem elevator;
    private XboxController xbox;
    private XboxController driveXbox;

    private double elevatorPos = 0;

    public ElevatorControlCommand(ElevatorSubsystem elevator, XboxController xbox, XboxController driveXbox) {
        this.elevator = elevator;
        this.xbox = xbox;
        this.driveXbox = driveXbox;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        int pov = xbox.getPOV();
        if (pov != -1) {
            int manualVel = 0;
            if (pov == 0) {
                manualVel = 1;
            } else if (pov == 180) {
                manualVel = -1;
            }
            elevator.elevate(manualVel);
            return;
        }
        


        if (-xbox.getLeftY() > 0.6) {
            elevator.setHeight(1);
        } else if (-xbox.getLeftY() < -0.6) {
            elevator.setHeight(0);
        } else if (xbox.getXButton()) {
            elevator.elevate(-.4);
        } else {
            elevator.hold();
        }
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
