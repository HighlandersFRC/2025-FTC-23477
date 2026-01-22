package org.firstinspires.ftc.teamcode.PathingTool;

import static org.firstinspires.ftc.teamcode.Tools.Constants.X_PID_P;
import static org.firstinspires.ftc.teamcode.Tools.Constants.Y_PID_P;
import static org.firstinspires.ftc.teamcode.Tools.Constants.YAW_PID_P;

import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.firstinspires.ftc.teamcode.Tools.Vector;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class PolarPathFollower implements Command {

    private final Drive drive;
    private final CommandScheduler scheduler;
    private final JSONArray points;
    private final HashMap<String, Supplier<Command>> commandMap;
    private final HashMap<String, BooleanSupplier> conditionMap;

    private int lastPointIndex = 0;

    private double pathStartTime;
    private double nextX, nextY;
    private final Set<String> activeCommandKeys = new HashSet<>();

    public PolarPathFollower(Drive drive, JSONObject pathJSON,
                             HashMap<String, Supplier<Command>> commandMap,
                             HashMap<String, BooleanSupplier> conditionMap,
                             CommandScheduler scheduler) throws JSONException {
        this.drive = drive;
        this.scheduler = scheduler;
        this.points = pathJSON.getJSONArray("sampled_points");
        this.commandMap = commandMap;
        this.conditionMap = conditionMap;
    }

    private double getCurrentTime() {
        return (System.currentTimeMillis() / 1000.0) - pathStartTime;
    }

    @Override
    public void start() {
        pathStartTime = System.currentTimeMillis() / 1000.0;

        try {
            JSONObject firstPoint = points.getJSONObject(0);
            nextX = firstPoint.getDouble("x");
            nextY = firstPoint.getDouble("y");
            double nextThetaDeg = Math.toDegrees(firstPoint.getDouble("angle"));

            Mouse.setPosition(nextX, nextY, nextThetaDeg);
        } catch (JSONException e) {
            throw new RuntimeException("Error reading first point from JSON", e);
        }

        YAW_PID_P.setMinInput(-180);
        YAW_PID_P.setMaxInput(180);
    }

    @Override
    public void execute() {
        FinalPose.poseUpdate();
        double elapsedTime = getCurrentTime();

        JSONObject point = null;
        try {
            for (int i = lastPointIndex; i < points.length(); i++) {
                JSONObject p = points.getJSONObject(i);
                if (p.getDouble("time") > elapsedTime) break;
                point = p;
                lastPointIndex = i;
            }

            if (point == null) return;

            nextX = point.getDouble("x");
            nextY = point.getDouble("y");
            double nextThetaDeg = Math.toDegrees(point.getDouble("angle"));

            double currentX = FinalPose.x;
            double currentY = FinalPose.y;
            double currentTheta = FinalPose.yaw;
            X_PID_P.setSetPoint(nextX);
            X_PID_P.updatePID(currentX);

            Y_PID_P.setSetPoint(nextY);
            Y_PID_P.updatePID(currentY);

            YAW_PID_P.setSetPoint(nextThetaDeg);
            YAW_PID_P.updatePID(currentTheta);

            Vector relativePos = new Vector(X_PID_P.getResult(), Y_PID_P.getResult());
            drive.autoDrive(relativePos, YAW_PID_P.getResult());

            JSONArray commands = point.optJSONArray("commands");
            if (commands != null) {
                for (int i = 0; i < commands.length(); i++) {
                    JSONObject commandJSON = commands.getJSONObject(i);
                    if (commandJSON.has("command")) {
                        String commandKey = commandJSON.getString("command");
                        if (!activeCommandKeys.contains(commandKey)) {
                            Command command = parseCommand(commandJSON);
                            if (command != null) {
                                scheduler.schedule(command);
                                activeCommandKeys.add(commandKey);
                            }
                        }
                    } else {
                        Command command = parseCommand(commandJSON);
                        if (command != null) scheduler.schedule(command);
                    }
                }
            }

        } catch (JSONException e) {
            throw new RuntimeException("Error reading point data from JSON", e);
        }
    }


    private Command parseCommand(JSONObject commandJSON) throws JSONException {
        if (commandJSON.has("command")) {
            return singleCommandFromJSON(commandJSON);
        } else if (commandJSON.has("parallel_command_group")) {
            return parseParallelCommandGroup(commandJSON.getJSONArray("parallel_command_group"));
        } else if (commandJSON.has("sequential_command_group")) {
            return parseSequentialCommandGroup(commandJSON.getJSONArray("sequential_command_group"));
        } else if (commandJSON.has("conditional_command")) {
            return parseConditionalCommand(commandJSON.getJSONObject("conditional_command"));
        }
        return null;
    }

    private Command singleCommandFromJSON(JSONObject commandJSON) throws JSONException {
        String commandName = commandJSON.getString("command");

        Supplier<Command> commandSupplier = commandMap.get(commandName);
        if (commandSupplier != null) {
            return commandSupplier.get();
        }

        return null;
    }


    private Command parseParallelCommandGroup(JSONArray commands) throws JSONException {
        ArrayList<Command> list = new ArrayList<>();
        for (int i = 0; i < commands.length(); i++) {
            Command c = parseCommand(commands.getJSONObject(i));
            if (c != null) list.add(c);
        }
        return new ParallelCommandGroup(scheduler, Parameters.ALL, list.toArray(new Command[0]));
    }

    private Command parseSequentialCommandGroup(JSONArray commands) throws JSONException {
        ArrayList<Command> list = new ArrayList<>();
        for (int i = 0; i < commands.length(); i++) {
            Command c = parseCommand(commands.getJSONObject(i));
            if (c != null) list.add(c);
        }
        return new SequentialCommandGroup(scheduler, list.toArray(new Command[0]));
    }

    private Command parseConditionalCommand(JSONObject commandJSON) throws JSONException {
        BooleanSupplier condition = conditionMap.get(commandJSON.getString("condition"));
        Command onTrue = parseCommand(commandJSON.getJSONObject("on_true"));
        Command onFalse = parseCommand(commandJSON.getJSONObject("on_false"));
        return new ConditionalCommand(onTrue, onFalse, condition);
    }

    @Override
    public void end() {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        try {
            if (points.length() == 0) return true;
            double lastTime = points.getJSONObject(points.length() - 1).getDouble("time");
            return getCurrentTime() >= lastTime;
        } catch (JSONException e) {
            return true;
        }
    }


    @Override
    public Subsystem getRequiredSubsystem() {
        return drive;
    }
}