package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.input.GamepadStatic;

import java.util.ArrayList;

public class CommandMachine {

    private ArrayList<CommandSequenceTrigger> commandSequences = new ArrayList<>();
    private int currentCommandIndex;

    public CommandMachine() {
        this.currentCommandIndex = 0;
    }

    public CommandMachine addCommandSequence(CommandSequence commandSequence, GamepadStatic.Input triggerCondition) {
        CommandSequenceTrigger commandSequenceTrigger = new CommandSequenceTrigger(commandSequence, triggerCondition);
        commandSequences.add(commandSequenceTrigger);
        return this;
    }

    public CommandMachine build() {
        return this;
    }

    /**
     *
     * @return index of the current command waiting for gamepad input
     */
    public int getCurrentCommandIndex() {
        return currentCommandIndex;
    }

    public void next() {
        CommandSequenceTrigger currentCommand = commandSequences.get(currentCommandIndex);
        currentCommand.trigger();
        skip();
    }

    public void skip() {
        if (currentCommandIndex == commandSequences.size() - 1) {
            currentCommandIndex = 0;
        } else {
            currentCommandIndex++;
        }
    }

    public void reset() {
        currentCommandIndex = 0;
    }

    public void run(Gamepad gamepad) {
        CommandSequenceTrigger currentCommand = commandSequences.get(currentCommandIndex);

        if (GamepadStatic.isButtonPressed(gamepad, currentCommand.triggerCondition)) {
            next();
        }
    }
}