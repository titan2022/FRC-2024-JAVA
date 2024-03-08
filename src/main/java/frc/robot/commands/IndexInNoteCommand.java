// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

<<<<<<< HEAD:src/main/java/frc/robot/commands/ShootSpeakerCommand.java
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
=======
>>>>>>> linkage-shooter:src/main/java/frc/robot/commands/IndexInNoteCommand.java
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SlamDunkerSubsystem;
import edu.wpi.first.wpilibj.Timer;


/** An example command that uses an example subsystem. */
<<<<<<< HEAD:src/main/java/frc/robot/commands/ShootSpeakerCommand.java
public class ShootSpeakerCommand extends Command {
    public static final double RAMP_UP_TIME = 1;
    public static final double SHOOT_TIME = 0.5;
    public static final double INDEXER_SPEED = 0.5;

    private ShooterSubsystem shooter;
    private double speed;
    private double rampTime;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public ShootSpeakerCommand(double speed, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.speed = speed;

        addRequirements(shooter);
=======
public class IndexInNoteCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static double DURATION_AFTER_NOTE = 0.5;
    public double speed;
    public ElevatorSubsystem elevator;

    public IndexInNoteCommand(double speed, ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.speed = speed;

        addRequirements(elevator);
>>>>>>> linkage-shooter:src/main/java/frc/robot/commands/IndexInNoteCommand.java
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rampTime = Timer.getFPGATimestamp() + RAMP_UP_TIME;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
<<<<<<< HEAD:src/main/java/frc/robot/commands/ShootSpeakerCommand.java
        shooter.shoot(speed);
        if (Timer.getFPGATimestamp() > rampTime) {
            shooter.setIndexer(INDEXER_SPEED);
        }
=======
        elevator.indexSpeed(speed);
>>>>>>> linkage-shooter:src/main/java/frc/robot/commands/IndexInNoteCommand.java
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
<<<<<<< HEAD:src/main/java/frc/robot/commands/ShootSpeakerCommand.java
        shooter.shoot(0);
        shooter.setIndexer(0);
=======
        elevator.indexSpeed(0);
>>>>>>> linkage-shooter:src/main/java/frc/robot/commands/IndexInNoteCommand.java
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
<<<<<<< HEAD:src/main/java/frc/robot/commands/ShootSpeakerCommand.java
        if (Timer.getFPGATimestamp() < rampTime + SHOOT_TIME)
            return false;
        else
            return true;
    }
}
=======
        if (elevator.hasNote())
            return true;
        else 
            return false;
    }
  }
>>>>>>> linkage-shooter:src/main/java/frc/robot/commands/IndexInNoteCommand.java
