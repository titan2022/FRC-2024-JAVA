package frc.robot.commands.align;
// package frc.robot.commands;

// import static frc.robot.utility.Constants.Unit.DEG;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.IndexerSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.SwerveDriveSubsystem;
// import frc.robot.utility.Localizer;

// /**
//  * Aligns robot with note and intakes it in until it hits the beam breaker
//  */
// public class IntakeWithAlignCommand extends Command {
// 	private static final double MAX_ROT_SPEED = Math.PI / 4;
// 	private static final double MAX_POS_SPEED = 1;
// 	private static final double MAX_POS_TIME = 2 * 1000; // In milliseconds
// 	private static final double ALIGNMENT_THRESHOLD = 2 * DEG;

// 	private SwerveDriveSubsystem drive;
// 	private IntakeSubsystem intake;
// 	private IndexerSubsystem indexer;
// 	private Localizer localizer;

// 	private Rotation2d targetHeading;
// 	private int timer = 0;

// 	public IntakeWithAlignCommand(SwerveDriveSubsystem drive, IntakeSubsystem intake, IndexerSubsystem indexer,
// 			Localizer localizer, boolean withLimelight) {
// 		this.drive = drive;
// 		this.intake = intake;
// 		this.indexer = indexer;
// 		this.localizer = localizer;
// 	}

// 	public IntakeWithAlignCommand(SwerveDriveSubsystem drive, IntakeSubsystem intake, IndexerSubsystem indexer,
// 			Localizer localizer) {
// 		this(drive, intake, indexer, localizer, false);
// 	}

// 	@Override
// 	public void initialize() {
// 		drive.getTranslational().setVelocity(new Translation2d(0, 0));
// 		drive.getRotational().setRotationalVelocity(new Rotation2d(0));
// 		intake.setWheelSpeed(1000);
// 	}

// 	@Override
// 	public void execute() {
// 		if (localizer != null) {
// 			targetHeading = localizer.getNoteHeading();
// 		}

// 		double headingDiff = targetHeading.minus(localizer.getHeading()).getRadians();
// 		if (Math.abs(headingDiff) > ALIGNMENT_THRESHOLD) {
// 			Rotation2d newVelocity = new Rotation2d(Math.copySign(MAX_ROT_SPEED, headingDiff));
// 			drive.getRotational().setRotationalVelocity(newVelocity);
// 		} else {
// 			drive.getTranslational().setVelocity(new Translation2d(0, MAX_POS_SPEED));
// 			timer += 20;
// 		}
// 	}

// 	@Override
// 	public boolean isFinished() {
// 		return timer > MAX_POS_TIME || indexer.hasNote();
// 	}

// 	@Override
// 	public void end(boolean isInterrupted) {

// 	}
// }
