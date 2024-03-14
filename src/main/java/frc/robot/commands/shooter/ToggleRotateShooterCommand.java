// package frc.robot.commands.shooter;

// import frc.robot.utility.Constants.Unit;
// import frc.robot.utility.Constants.Unit.*;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.IndexerSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;

// /** An example command that uses an example subsystem. */
// public class ToggleRotateShooterCommand extends Command {
//     @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
//     // public static final double RAMP_TIME = 1;
//     // public static final double SHOOT_DURATION = 0.25;
//     // public static final double INDEX_SPEED = 0.5;
//     public ShooterSubsystem shooter;
//     public XboxController xbox;
//     public Rotation2d angle;
//     public boolean toggleOn = false;
    
//     public ToggleRotateShooterCommand(Rotation2d angle, ShooterSubsystem shooter, XboxController xbox) {
//         this.shooter = shooter;
//         this.angle = angle;
//         this.xbox = xbox;
//     }

//     @Override
//     public void initialize() {
        
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         if (xbox.getYButton())
//             toggleOn = !toggleOn;

//         if (toggleOn)
//             shooter.setRotation(angle);
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         shooter.holdAngle();
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }

