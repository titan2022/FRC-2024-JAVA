package frc.robot.utility;

import edu.wpi.first.wpilibj.XboxController;

public class TeleopListener {
    public XboxController controller;
    public boolean run = false;
    public TeleopListener(XboxController controller) {
        this.controller = controller;
    }

    public void enable() {
        run = true;
    }

    public void execute() {
        
    }
}
