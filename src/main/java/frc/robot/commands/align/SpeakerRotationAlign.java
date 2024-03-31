package frc.robot.commands.align;
// package frc.robot.commands;

import static frc.robot.utility.Constants.Unit.IN;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.commands.drive.RotationCommand;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;

public class SpeakerRotationAlign extends Command{
    private final RotationalDrivebase rotationalDrivebase;
    private final Localizer localizer;
    private PIDController pid = new PIDController(0, 0, 0);

    public SpeakerRotationAlign(RotationalDrivebase rotationalDrivebase, Localizer localizer){
        this.rotationalDrivebase = rotationalDrivebase;
        this.localizer = localizer;
        // addRequirements(rotationalDrivebase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Rotation2d speaker_rotation = localizer.getSpeakerPosition().getAngle();
        Rotation2d speaker_orientation = localizer.getSpeakerOrientation();
        double offset = speaker_orientation.minus(speaker_rotation).getRadians();
        SmartDashboard.putNumber("angle_offset", offset);
        double output = pid.calculate(offset);
        // rotationalDrivebase.setRotationalVelocity(new Rotation2d(output));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}