
package org.firstinspires.ftc.teamcode.Tools;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.CommandIndex;
import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandQueue;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;
import org.firstinspires.ftc.teamcode.Commands.CommandSpinRight;
import org.firstinspires.ftc.teamcode.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Constants {
    public static HashMap<String, Supplier<Command>> commandMap = new HashMap<>();
    public static HashMap<String, BooleanSupplier> conditionMap = new HashMap<>();


    // Drive Auto
    public static final PID xPID = new PID(1.5, 0, 0);
    public static final PID thetaPID = new PID(1.3, 0, 0.001);
    public static final PID yPID = new PID(5, 0, 0);

    public static final double DISTANCE_TOLERANCE = 0.1;
    public static final double THETA_TOLERANCE = 2.0;


    // Limelight
    public static final PID thetaPIDLimelight = new PID(0.04, 0.0, 0.01);
    public static double lastTx = 0;
    public static final double TX_TOLERANCE = 1.5;
    public static final double MAX_TURN = 0.45;
    public static final double MIN_TURN = 0.08;

    public static final double cameraHeightI = 11.5;

    public static final double tagHeight = 0.762;
    public static final double maxAngle = 180;

    public static final double MIN_ANGLE = 10;
    public static final double MAX_ANGLE = 80;

    public static final double MAX_STEP = 2.0; // degrees per loop

    public static final PID tiltPID = new PID(0.5, 0, 0);


    // Intake
    public static final PID IntakeHoldPID = new PID(0.5, 0, 0);

    // Indexer
    public static double IndexerPos;
    public static final PID IndexerPID = new PID(0.5, 0, 0);

    // Shooter
    public static double targetRPM = 0;
    public static int lastEncoderPos = 0;
    public static long lastTime = 0;
    public static double ticksPerSecond = 0;

    public static final float feedForward = (float) 1 / 6000;
    public static final PIDF velocityPID = new PIDF(0.05, 0.00001, 0.0001, feedForward);

    public static final double[][] SHOOTER_LOOKUP = {
            {0.0, 3500},
            {1.341, 4500.0},
            {1.6378, 4900.0},
            {2.571, 6000}
    };

    public static final long durationMs = 3600;
    public static final long durationMsAuto = 4500;

    @NonNull
    public static SequentialCommandGroup SHOOT(CommandScheduler scheduler, NewRobot robot, long duration, boolean isAuto) {
        double distance = Limelight.getDistance(
                tagHeight
        );
        if (isAuto) {
            return new SequentialCommandGroup(
                    scheduler,
                    new ConditionalCommand(
                            new ParallelCommandGroup(
                                    scheduler, Parameters.ANY,
                                    new CommandShoot(robot.shooterStates, distance, duration),
                                    new CommandSpinRight(robot.sequencerState, duration)
                            ),
                            new CommandShoot(robot.shooterStates, distance, duration+300),
                            () -> robot.shooterStates.isAtTargetVelocity()
                    ),

                    new CommandIntake(robot.intakeStates, 1000),

                    new ConditionalCommand(
                            new ParallelCommandGroup(
                                    scheduler, Parameters.ANY,
                                    new CommandShoot(robot.shooterStates, distance, duration),
                                    new CommandSpinRight(robot.sequencerState, duration)
                            ),
                            new CommandShoot(robot.shooterStates, distance, duration),
                            () -> robot.shooterStates.isAtTargetVelocity()
                    )
            );
        } else {
            return new SequentialCommandGroup(
                    scheduler, new ConditionalCommand(
                    new ParallelCommandGroup(
                            scheduler, Parameters.ANY,
                            new CommandShoot(robot.shooterStates, distance, duration),
                            new CommandSpinRight(robot.sequencerState, duration)
                    ),
                    new CommandShoot(robot.shooterStates, distance, duration),
                    () -> robot.shooterStates.isAtTargetVelocity()
            )
            );

        }
    }


    @NonNull
    public static SequentialCommandGroup IndexTest(CommandScheduler scheduler, NewRobot robot, long duration) {
        double distance = 0;
            return new SequentialCommandGroup(
                    scheduler,
                    new ConditionalCommand(
                            new ParallelCommandGroup(
                                    scheduler, Parameters.ANY,
                                    new CommandShoot(robot.shooterStates, distance, duration),
                                    new CommandQueue(robot.queueState, duration)
                            ),
                            new CommandShoot(robot.shooterStates, distance, duration),
                            () -> robot.shooterStates.isAtTargetVelocity()
                    )
//                    ,new CommandIndex(robot.indexerState, duration),
//                    new ConditionalCommand(
//                            new ParallelCommandGroup(
//                                    scheduler, Parameters.ANY,
//                                    new CommandShoot(robot.shooterStates, distance, duration),
//                                    new CommandQueue(robot.queueState, duration)
//                            ),
//                            new CommandShoot(robot.shooterStates, distance, duration),
//                            () -> robot.shooterStates.isAtTargetVelocity()
//                    ),
//                    new CommandIndex(robot.indexerState, duration),
//                    new ConditionalCommand(
//                            new ParallelCommandGroup(
//                                    scheduler, Parameters.ANY,
//                                    new CommandShoot(robot.shooterStates, distance, duration),
//                                    new CommandQueue(robot.queueState, duration)
//                            ),
//                            new CommandShoot(robot.shooterStates, distance, duration),
//                            () -> robot.shooterStates.isAtTargetVelocity()
//                    )


            );

    }


    // PolarPathing
    public static final double AUTONOMOUS_LOOKAHEAD_DISTANCE = 1;
    public static final double AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS = 1;
    public static final double AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS = Math.PI;


    public static final PID xPIDP = new PID(3.6, 0, 1.9);
    public static final PID yPIDP = new PID(3.6, 0, 1.9);
    public static final PID yawPIDP = new PID(5, 0, 0);

}