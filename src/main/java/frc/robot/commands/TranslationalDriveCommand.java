package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;

import static frc.robot.utility.Constants.Unit.*;

public class TranslationalDriveCommand extends Command {
    private TranslationalDrivebase drive;
    private XboxController xbox;
    private Localizer localizer;
    private double maxVel;
    private boolean isFieldOriented = false;
    private Rotation2d phiOffset = new Rotation2d(0);

    /**
     * Creates a new TranlationalDriveCommand with optional field orientation.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     * @param localizer  The gyroscope to use for orientation correction.
     * @param velocity  The translational velocity represented by moving the
     *  translational joystick all the way towards a cardinal direction, in
     *  meters per second.
     */
    public TranslationalDriveCommand(TranslationalDrivebase drive, XboxController xbox, Localizer localizer, double velocity) {
        this.drive = drive;
        this.xbox = xbox;
        this.localizer = localizer;
        maxVel = velocity;
        addRequirements(drive);
    }
    /**
     * Creates a new TranlationalDriveCommand using intrinsic orientation.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     * @param velocity  The translational velocity represented by moving the
     *  translational joystick all the way towards a cardinal direction, in
     *  meters per second.
     */
    public TranslationalDriveCommand(TranslationalDrivebase drivebase, XboxController xbox, double velocity) {
        this(drivebase, xbox, null, velocity);
    }
    /**
     * Creates a new TranlationalDriveCommand with optional field orientation.
     * 
     * The maximum velocity in a cardinal direction defaults to 5 meters per second.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     * @param localizer  The gyroscope to use for orientation correction.
     */
    public TranslationalDriveCommand(TranslationalDrivebase drivebase, XboxController xbox, Localizer localizer) {
        this(drivebase, xbox, localizer, 10.);
    }
    /**
     * Creates a new TranlationalDriveCommand using intrinsic orientation.
     * 
     * The maximum velocity in a cardinal direction defaults to 5 meters per second.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     */
    public TranslationalDriveCommand(TranslationalDrivebase drivebase, XboxController xbox) {
        this(drivebase, xbox, null, 5.);
    }

    @Override
    public void initialize() {
        
    }

    private static double applyDeadband(double joy, double deadband) {
        return Math.abs(joy) < deadband ? 0 : joy;
    }
    
    private double scaleVelocity(double joy) {
        return Math.signum(joy) * joy * joy * maxVel;
    }

    @Override
    public void execute() {
        // if(xbox.getYButtonPressed()){
        //     maxVel = maxVel == 10 ? 1 : 10;
        //     SmartDashboard.putNumber("maxVel", maxVel);
        // }
        if(xbox.getBButtonPressed()){
            isFieldOriented = !isFieldOriented;
            SmartDashboard.putBoolean("isFieldOriented", isFieldOriented);
        }
        if(xbox.getAButtonPressed())
            phiOffset = localizer.getOrientation();
        double joyX = applyDeadband(xbox.getLeftX(), 0.15);
        double joyY = applyDeadband(-xbox.getLeftY(), 0.15);
        // double dpadX = xbox.getPOV() == -1 ? 0 : Math.sin(xbox.getPOV() * DEG / RAD);
        // double dpadY = xbox.getPOV() == -1 ? 0 : Math.cos(xbox.getPOV() * DEG / RAD);
        //SmartDashboard.putNumber("joyX", joyX);
        //SmartDashboard.putNumber("joyY", joyY);
        Translation2d velocity = new Translation2d(
            scaleVelocity(joyX), //  + 0.5 * dpadX
            scaleVelocity(joyY)); //  + 0.5 * dpadY
        //SmartDashboard.putNumber("fieldX", velocity.getX());
        //SmartDashboard.putNumber("fieldY", velocity.getY());
        if(isFieldOriented){
            Rotation2d heading = new Rotation2d(Math.PI/2).minus(localizer.getHeading().minus(phiOffset));
            //SmartDashboard.putNumber("heading", heading.getDegrees());
            velocity = velocity.rotateBy(heading);
        }
        SmartDashboard.putNumber("robotX", velocity.getX());
        SmartDashboard.putNumber("robotY", velocity.getY());
        drive.setVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setVelocity(new Translation2d(0, 0));
        if(interrupted)
            new StartEndCommand(() -> {
                    xbox.setRumble(RumbleType.kLeftRumble, 0.5);
                }, () -> {
                    xbox.setRumble(RumbleType.kLeftRumble, 0);
                }).withTimeout(0.5).schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}