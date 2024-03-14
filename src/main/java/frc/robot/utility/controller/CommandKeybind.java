package frc.robot.utility.controller;

import java.util.function.Consumer;
import java.util.function.Function;

public class CommandKeybind implements Keybind {
    public Function<Void, Boolean> listener;
    public Consumer<Boolean> function;
    public boolean value;
    public double deltaTime;

    public CommandKeybind(Function<Void, Boolean> listener, Consumer<Boolean> function) {
        this.listener = listener;
        this.function = function;
    }

    public void update() {
        value = listener.apply(null);
        if (value && deltaTime <= 0) 
            deltaTime += 5;

        if (deltaTime > 0)
            deltaTime--;
    }
    public void execute() {
        if (value && deltaTime <= 0)
            function.accept(value);
    }
}
