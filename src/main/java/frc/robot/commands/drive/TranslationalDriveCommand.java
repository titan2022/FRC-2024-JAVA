package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.drive.TranslationalDrivebase;
import frc.robot.utility.Localizer;

public class TranslationalDriveCommand extends Command {
    private TranslationalDrivebase drive;
    private XboxController xbox;
    private Localizer localizer;
    private double maxVel;
    private boolean isFieldOriented = true;
    private Rotation2d phiOffset = new Rotation2d(0);
    private double xOffset = 0;
    private double yOffset = 0;

    /**
     * Creates a new TranlationalDriveCommand with optional field orientation.
     * 
     * @param drivebase The drivebase to control.
     * @param localizer The localizer to use for orientation correction.
     * @param xbox      The joystick controller to use.
     * @param velocity  The translational velocity represented by moving the
     *                  translational joystick all the way towards a cardinal
     *                  direction, in
     *                  meters per second.
     */
    public TranslationalDriveCommand(TranslationalDrivebase drive, Localizer localizer, XboxController xbox,
            double maxVel) {
        this.drive = drive;
        this.localizer = localizer;
        this.xbox = xbox;
        this.maxVel = maxVel;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        
    }

    private static double applyDeadband(double joy, double deadband) {
        return Math.abs(joy) < deadband ? 0 : joy;
    }

    private double scaleVelocity(double joy) {
        double scaledVel = Math.signum(joy) * joy * joy * maxVel;
        if (xbox.getLeftBumper()) {
            scaledVel *= 0.3;
        }
        return scaledVel;
    }

    @Override
    public void execute() {
        // if (xbox.getLeftBumperPressed()) {
        //     xOffset = xbox.getLeftX();
        //     yOffset = xbox.getLeftY();
        // }

        if (xbox.getBButtonPressed() && localizer != null) {
            isFieldOriented = !isFieldOriented;
            SmartDashboard.putBoolean("isFieldOriented", isFieldOriented);
        }

        if (xbox.getAButtonPressed()) {
            phiOffset = localizer.getOrientation();
        }

        double speedMult = 1;
        if (xbox.getRightBumper()) {
            speedMult = 0.25;
        }

        double joyX = applyDeadband(xbox.getLeftX() - xOffset, 0.15);
        double joyY = applyDeadband(-xbox.getLeftY() - yOffset, 0.15);
        // double x = 0;
        // double y = 0;
        // if (Math.abs(xbox.getLeftX()) > 0.4) {
        //     x = 1;
        // }

        // if (Math.abs(xbox.getLeftY()) > 0.4) {
        //     y = 1;
        // }
        Translation2d velocity = new Translation2d(scaleVelocity(joyX), scaleVelocity(joyY));
        // Translation2d velocity = new Translation2d(x, y);

        if (isFieldOriented) {
            Rotation2d heading = new Rotation2d(Math.PI / 2).minus(localizer.getHeading().minus(phiOffset));
            velocity = velocity.rotateBy(heading);
        }

        SmartDashboard.putNumber("Target Velocity X", velocity.getX());
        SmartDashboard.putNumber("Target Velocity Y", velocity.getY());
        drive.setVelocity(velocity.times(speedMult));
    }

    @Override
    public void end(boolean interrupted) {
        drive.setVelocity(new Translation2d(0, 0));
        if (interrupted) {
            new StartEndCommand(() -> {
                xbox.setRumble(RumbleType.kLeftRumble, 0.5);
            }, () -> {
                xbox.setRumble(RumbleType.kLeftRumble, 0);
            }).withTimeout(0.5).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}