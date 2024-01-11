package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.RotationalDrivebase;
import static frc.robot.utility.Constants.Unit.*;

public class XBoxRotationalCommand extends CommandBase {
    private RotationalDrivebase drive;
    private XboxController controller;
    private double maxRate, omega;
    private WPI_Pigeon2 imu;

    /**
     * Controls the rotational velocity of the drivebase with a joystick. 
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     * @param turnRate  The maximum rotational velocity in radians per second.
     */
    public XBoxRotationalCommand(RotationalDrivebase drivebase, XboxController xbox, double turnRate, WPI_Pigeon2 pigeon) {
        drive = drivebase;
        controller = xbox;
        maxRate = turnRate;
        addRequirements(drivebase);
        imu = pigeon;
    }
    /**
     * Controls the rotational velocity of the drivebase with a joystick. 
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     * @param turnRate  The maximum rotational velocity in radians per second.
     */
    public XBoxRotationalCommand(RotationalDrivebase drivebase, XboxController xbox, double turnRate) {
        this(drivebase, xbox, turnRate, null);
    }
    /**
     * Controls the rotational velocity of the drivebase with a joystick.
     * 
     * The maximum rotational velocity defaults to two revolutions per second.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     */
    public XBoxRotationalCommand(RotationalDrivebase drivebase, XboxController xbox) {
        this(drivebase, xbox, 4 * Math.PI);
    }

    @Override
    public void initialize() {}

    private static double applyDeadband(double joy, double deadband) {
        return Math.abs(joy) < deadband ? 0 : joy;
    }
    
    private double scaleVelocity(double joy) {
        return Math.signum(joy) * joy * joy * maxRate;
    }

    @Override
    public void execute() {
        double drift = -imu.getRate() * DEG/S - omega;
        double joy = applyDeadband(controller.getRightX(), 0.1);
        omega = -scaleVelocity(joy);
        drive.setRotation(omega - 0.5*drift);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setRotation(0);
        if(interrupted)
            new StartEndCommand(() -> {
                    controller.setRumble(RumbleType.kRightRumble, 0.5);
                }, () -> {
                    controller.setRumble(RumbleType.kRightRumble, 0);
                }).withTimeout(0.5).schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}