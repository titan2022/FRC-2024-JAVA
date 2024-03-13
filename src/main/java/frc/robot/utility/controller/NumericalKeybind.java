package frc.robot.utility.controller;

import java.util.function.Consumer;
import java.util.function.Function;

public class NumericalKeybind implements Keybind {
    public Function<Void, Double> listener;
    public Consumer<Double> function;
    public double value;

    public NumericalKeybind(Function<Void, Double> listener, Consumer<Double> function) {
        this.listener = listener;
        this.function = function;
    }

    public void update() {
        value = listener.apply(null);
    }
    public void execute() {
        function.accept(value);
    }
    
}
