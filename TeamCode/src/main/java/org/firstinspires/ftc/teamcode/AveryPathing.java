package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.X_PID_P;
import static org.firstinspires.ftc.teamcode.Tools.Constants.YAW_PID_P;
import static org.firstinspires.ftc.teamcode.Tools.Constants.Y_PID_P;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.firstinspires.ftc.teamcode.Tools.Vector;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AveryPathing implements Command {
    Drive drive;
    CommandScheduler scheduler = new CommandScheduler();
    private HashMap<String, BooleanSupplier> conditionMap;
    private HashMap<String, Supplier<Command>> commandMap;
    private JSONArray points;
    private double pathStartTime;
    private double nextX, nextY;
    private double nextTheta;
    private final ArrayList<Command> activeCommands = new ArrayList<>();

    public void PolarPathFollower(Drive drive, JSONObject pathJSON,
                                  HashMap<String, Supplier<Command>> commandMap,
                                  HashMap<String, BooleanSupplier> conditionMap,
                                  CommandScheduler scheduler) throws JSONException {
        this.scheduler = scheduler;
        this.points = pathJSON.getJSONArray("sampled_points");
        this.commandMap = commandMap;
        this.conditionMap = conditionMap;
        this.drive = drive;
    }

    public AveryPathing(Drive drive,HashMap<String, BooleanSupplier> conditionMap, HashMap<String, Supplier<Command>> commandMap, JSONObject points) throws JSONException {
        this.conditionMap = conditionMap;
        this.commandMap = commandMap;
        this.points = points.getJSONArray("sampled_points");
        this.drive = drive;
    }

    private double getPathTime() {return System.currentTimeMillis() /1000.0;}
    private double getCurrentTime() {return getPathTime()-pathStartTime;}

    @Override
    public void start() {

        this.pathStartTime = getPathTime();

        try {
            JSONObject currentPoint = points.getJSONObject(0);
            nextX = currentPoint.getDouble("x");
            nextY = currentPoint.getDouble("y");
            nextTheta = currentPoint.getDouble("angle");
        }catch (JSONException e) {
            throw new RuntimeException("Error reading point data from JSON", e);

        }
    }

    @Override
    public void execute() {

        FinalPose.poseUpdate();
        double elapsedTime = getPathTime()-pathStartTime;

        int index = (int) ((elapsedTime +0.05)/0.01);
        if (index >= points.length()) {
            index = points.length() - 1;
        }

        try {
            JSONObject currentPoint = points.getJSONObject(index);
            nextX = currentPoint.getDouble("x");
            nextY = currentPoint.getDouble("y");
            double nextTheta = currentPoint.getDouble("angle");

            double currentX = FinalPose.x;
            double currentY = FinalPose.y;
            double currentTheta = Math.toRadians(FinalPose.yaw);

            X_PID_P.setSetPoint(nextX);
            X_PID_P.updatePID(currentX);

            Y_PID_P.setSetPoint(nextY);
            Y_PID_P.updatePID(currentY);

            YAW_PID_P.setSetPoint(nextTheta);
            YAW_PID_P.updatePID(currentTheta);

            Vector relativePos = new Vector(X_PID_P.getResult(), Y_PID_P.getResult());
            drive.autoDrive(relativePos, YAW_PID_P.getResult());

            JSONArray commands = points.getJSONObject(index).optJSONArray("commands");
            if (commands != null) {
                for (int i = 0; i <commands.length(); i++) {
                    JSONObject commandJSON = commands.getJSONObject(i);
                    Command command = parseCommand(commandJSON);
                    if (command != null) {
                        scheduler.schedule(command);
                        activeCommands.add(command);

                    }
                }
            } System.out.println("VectorX:" + relativePos.getI() + ",VectorY:" + relativePos.getJ()
                    + ",Theta:" + currentTheta + ",Index:" + index);
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
        if (commandMap.containsKey(commandName)) {
            return commandMap.get(commandName).get();
        }
        return null;
    }

    private Command parseSequentialCommandGroup(JSONArray commands) throws JSONException {
        ArrayList<Command> commandList = new ArrayList<>();
        for (int i = 0; i < commands.length(); i++) {
            Command command = parseCommand(commands.getJSONObject(i));
            if (command != null) {
                commandList.add(command);
            }
        }
        return new SequentialCommandGroup(scheduler, commandList.toArray(new Command[0]));
    }

    private Command parseConditionalCommand(JSONObject commandJSON) throws JSONException {
        BooleanSupplier condition = conditionMap.get(commandJSON.getString("condition"));
        Command onTrue = parseCommand(commandJSON.getJSONObject("on_true"));
        Command onFalse = parseCommand(commandJSON.getJSONObject("on_false"));
        return new ConditionalCommand(onTrue, onFalse, condition);
    }

    private Command parseParallelCommandGroup(JSONArray commands) throws JSONException {
        ArrayList<Command> commandList = new ArrayList<>();
        for (int i = 0; i < commands.length(); i++) {
            Command command = parseCommand(commands.getJSONObject(i));
            if (command != null) {
                commandList.add(command);
            }
        }
        return new ParallelCommandGroup(scheduler, Parameters.ALL, commandList.toArray(new Command[0]));
    }


    @Override
    public void end() {
    drive.stop();
    }

    @Override
    public boolean isFinished() {
        return getCurrentTime() >= points.length() * 0.01;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return drive;
    }
}
