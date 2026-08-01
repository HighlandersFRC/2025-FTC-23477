
package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.RobotLog;
import java.util.*;

import org.firstinspires.ftc.teamcode.Command;
import org.firstinspires.ftc.teamcode.Subsystem;

import org.firstinspires.ftc.teamcode.Tools.robot;

public class commandScheduler {
    private static commandScheduler instance;
    private final List<Command> scheduledCommands = new ArrayList<>();
    private final Map<Subsystem, Command> activeSubsystemCommands = new HashMap<>();
    private robot newRobot;

    public commandScheduler() {

    }

    public static commandScheduler getInstance() {
        if (instance == null) {
            instance = new commandScheduler();
        }
        return instance;
    }


    public void setNewRobot(robot robot) {
        this.newRobot = robot;
    }

    public void schedule(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();

        if (requiredSubsystem != null) {
            Command activeCommand = activeSubsystemCommands.get(requiredSubsystem);

            if (activeCommand != null && activeCommand != command) {
                cancel(activeCommand);   // 🔥 THIS WAS MISSING
            }

            activeSubsystemCommands.put(requiredSubsystem, command);
        }


        // Schedule and start the new command if not already in the list
        if (!scheduledCommands.contains(command)) {
            scheduledCommands.add(command);
            command.start();
            RobotLog.d("Command Scheduled: " + command.getClass().getSimpleName());
        }
    }
    public boolean isSubsystemBusy(Subsystem subsystem) {
        return activeSubsystemCommands.containsKey(subsystem);
    }

    public void run() {
        List<Command> finishedCommands = new ArrayList<>();

        // Execute scheduled commands and handle completion
        for (Command command : new ArrayList<>(scheduledCommands)) {
            if (command.isFinished()) {
                command.end();
                finishedCommands.add(command);
                RobotLog.d("Command Finished and Ended: " + command.getClass().getSimpleName());

                Subsystem subsystem = command.getRequiredSubsystem();
                if (subsystem != null) {
                    activeSubsystemCommands.remove(subsystem);

                    // Only reschedule default command if no other commands are active for this subsystem
                }
            } else {
                command.execute();
            }
        }

        scheduledCommands.removeAll(finishedCommands);

        // Ensure default commands are scheduled when needed
    }


    public void printCurrentCommands() {
        RobotLog.d("===== Current Commands =====");
        for (Map.Entry<Subsystem, Command> entry : activeSubsystemCommands.entrySet()) {
            RobotLog.d("Subsystem: " + entry.getKey().getClass().getSimpleName() +
                    ", Command: " + entry.getValue().getClass().getSimpleName());
        }
        RobotLog.d("============================");
    }

    public void cancel(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();
        if (requiredSubsystem != null) {
            activeSubsystemCommands.remove(requiredSubsystem);
        }

        command.end();
        scheduledCommands.remove(command);
        RobotLog.d("Command Cancelled: " + command.getClass().getSimpleName());
    }

    private Set<Subsystem> getAllSubsystems() {
        Set<Subsystem> subsystems = new HashSet<>();
        if (newRobot != null) {
            // subsystems.add(newRobot.arm); UPDATE
            // subsystems.add(newRobot.intakeSubsystem); UPDATE
            // subsystems.add(newRobot.wrist);  UPDATE
        }
        return subsystems;
    }

    public boolean isCommandScheduled(Command command) {
        Subsystem subsystem = command.getRequiredSubsystem();
        return subsystem != null && activeSubsystemCommands.get(subsystem) == command;
    }


    public void removeDuplicateCommands() {
        List<Command> uniqueCommands = new ArrayList<>();

        for (Command command : new ArrayList<>(scheduledCommands)) {
            String name = command.getClass().getSimpleName();
            scheduledCommands.removeIf(c -> c.getClass().getSimpleName().equalsIgnoreCase(name));
            uniqueCommands.add(command);
        }

        scheduledCommands.addAll(uniqueCommands);
    }
}