package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.TranslationalDrivebase;
import frc.robot.utility.Utility;

/** An example command that uses an example subsystem. */
public class TranslationCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private TranslationalDrivebase driveBase;
  private Translation2d velocity;
  private double time;
  private double endTime;

  public TranslationCommand(Translation2d position, double speed, TranslationalDrivebase driveBase) {
    this.driveBase = driveBase;
    //Gets speed in direction of displacement
    velocity = Utility.scaleVector(position, speed);
    time = position.getNorm() / speed;

    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTime = Timer.getFPGATimestamp() + time;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("StartTime", startTime);
    // SmartDashboard.putNumber("EndTime", endTime);
    // SmartDashboard.putNumber("CurrentTime", Timer.getFPGATimestamp());
    // driveBase.setVelocity(velocity);

    // if (error.getNorm() < velocityDeadband) {
    //   controlVelocity = new Translation2d(0, 0);
    // } else {
    //   controlVelocity = error.times(kP);
    // }
    driveBase.setVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SmartDashboard.putBoolean("Reached end()", true);
    driveBase.setVelocity(new Translation2d(0, 0));
    // SmartDashboard.putBoolean("Reached end() 2", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() > endTime)
      return true;
    else
      return false;
  }
}
