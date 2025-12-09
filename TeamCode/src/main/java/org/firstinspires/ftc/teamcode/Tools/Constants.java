
package org.firstinspires.ftc.teamcode.Tools;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandShootSequence;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Constants {
    public static PID IntakeHoldPID = new PID(0.5, 0, 0);
    static float feedForward = (float) 1 / 6000;
    public static PIDF velocityPID = new PIDF(0.005, 0.00001, 0.0001, feedForward);

    public static HashMap<String, Supplier<Command>> commandMap = new HashMap<>();
    public static HashMap<String, BooleanSupplier> conditionMap = new HashMap<>();

    public static class AprilTagData {
        public double positionY;
        public double size;
        public double tagangle;

        public AprilTagData(double positionX, double positionY, double size, double tagangle) {

            this.positionY = positionY;
            this.size = size;
            this.tagangle = tagangle;
        }
    }


    @NonNull
    public static Command SHOOT(CommandScheduler scheduler, NewRobot robot, boolean isAuto) {
        long duration = isAuto ? 2500 : 3000;
        return new CommandShootSequence(
                scheduler,
                robot.shooterStates,
                robot.sequencerState,
                3400,
                duration
        );
    }


    public static final Map<Integer, AprilTagData> aprilTagMap = new HashMap<>();

    static {
        aprilTagMap.put(14, new AprilTagData(3.048, 3.66, 0.1016, 0));
        aprilTagMap.put(15, new AprilTagData(3.66, 1.83, 0.1016, 0));
        aprilTagMap.put(16, new AprilTagData(3.048, 0, 0.1016, 0));
    }

    public static final double AUTONOMOUS_LOOKAHEAD_DISTANCE = 1;
    public static final double AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS = 1;
    public static final double AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS = Math.PI;

}