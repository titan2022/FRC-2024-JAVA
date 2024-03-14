// package frc.robot.utility.controller;

// import java.util.ArrayList;
// import java.util.function.Consumer;
// import java.util.function.Function;

// import edu.wpi.first.math.Num;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.commands.MoveElevatorCommand;
// import frc.robot.subsystems.ElevatorSubsystem;

// public class TeleopListener {
//     public XboxController xbox;

//     private Function<Void, Double> XBOX_LEFT_X;
//     private Function<Void, Boolean> XBOX_X_BUTTON;
//     private Function<Void, Boolean> XBOX_Y_BUTTON;
//     private Function<Void, Boolean> XBOX_B_BUTTON;

//     private Function<Void, Boolean> XBOX_LEFT_BUMPER;
//     private Function<Void, Boolean> XBOX_RIGHT_BUMPER;

//     private Consumer<Boolean> MOVE_UP_ELEVATOR_COMMAND;
//     private Consumer<Boolean> MOVE_DOWN_ELEVATOR_COMMAND;
//     private Consumer<Boolean> ELEVATOR_ENCODER_RESET;


//     private ArrayList<Keybind> inputs = new ArrayList<Keybind>();
//     public boolean run = false;

//     public TeleopListener(XboxController xbox, ElevatorSubsystem elevator) {
//         setupKeybinds(xbox);
//         setupCommands(elevator);
//         bindKeysToAction();
//     }

//     public void setupKeybinds(XboxController xbox) {
//         //The 'a' is because Java is dumb and requires a type 
//         //for the Function<Void, . . .> class
//         //Treat it like it doesnt exist
//         XBOX_LEFT_X = (a) -> xbox.getLeftX();
//         XBOX_LEFT_BUMPER = (a) -> xbox.getLeftBumper();
//         XBOX_RIGHT_BUMPER = (a) -> xbox.getRightBumper();
//         XBOX_Y_BUTTON = (a) -> xbox.getYButton();
//         XBOX_X_BUTTON = (a) -> xbox.getXButton();
//         XBOX_B_BUTTON = (a) -> xbox.getBButton();
//     }

//     public void setupCommands(ElevatorSubsystem elevator) {
//         MOVE_UP_ELEVATOR_COMMAND = (Boolean a) -> {
//             CommandScheduler.getInstance().schedule(
//                 new MoveElevatorCommand(true, elevator)
//             );
//         };
//         MOVE_UP_ELEVATOR_COMMAND = (Boolean a) -> {
//             CommandScheduler.getInstance().schedule(
//                 new MoveElevatorCommand(true, elevator)
//             );
//         };

//         ELEVATOR_ENCODER_RESET = (Boolean a) -> {elevator.resetEncoder();};
//     }

//     public void bindKeysToAction() {
//         inputs.add(new BooleanKeybind(XBOX_LEFT_BUMPER, MOVE_UP_ELEVATOR_COMMAND));
//         inputs.add(new BooleanKeybind(XBOX_RIGHT_BUMPER, MOVE_DOWN_ELEVATOR_COMMAND));
//         inputs.add(new BooleanKeybind(XBOX_X_BUTTON, ELEVATOR_ENCODER_RESET));
//     }

//     public void enable() {
//         run = true;
//     }

//     public void disable() {
//         run = false;
//     }

//     public void execute() {
//         if (run) {
//             for (int i = 0; i < inputs.size(); i++) {
//                 inputs.get(i).update();
//                 inputs.get(i).execute();
//             }
//         }
//     }
// }
