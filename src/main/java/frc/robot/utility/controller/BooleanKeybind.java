package frc.robot.utility.controller;

import java.util.function.Consumer;
import java.util.function.Function;

public class BooleanKeybind implements Keybind {
    public Function<Void, Boolean> listener;
    public Consumer<Boolean> function;
    public boolean value;

    public BooleanKeybind(Function<Void, Boolean> listener, Consumer<Boolean> function) {
        this.listener = listener;
        this.function = function;
    }

    @Override
    public void update() {
        value = listener.apply(null);
    }

    @Override
    public void execute() {
        function.accept(value);
    }
}
