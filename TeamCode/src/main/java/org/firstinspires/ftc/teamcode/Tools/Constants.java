
package org.firstinspires.ftc.teamcode.Tools;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.Intake.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.Queuer.CommandQueue;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Shooter.CommandShoot;
import org.firstinspires.ftc.teamcode.Commands.Queuer.CommandSpinRight;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Commands.HelperCommands.CommandWaitToShoot;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.SequentialCommandGroup;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Constants {
    public static HashMap<String, Supplier<Command>> commandMap = new HashMap<>();
    public static HashMap<String, BooleanSupplier> conditionMap = new HashMap<>();




    // Drive Auto
    public static final PID X_PID = new PID(1.5, 0, 0);
    public static final PID THETA_PID = new PID(1.3, 0, 0.001);
    public static final PID Y_PID = new PID(5, 0, 0);

    public static final double DISTANCE_TOLERANCE = 0.1;
    public static final double THETA_TOLERANCE = 2.0;





    // Limelight
    public static final PID THETA_PID_LIMELIGHT = new PID(0.04, 0.0, 0.01);
    public static double LAST_TX = 0;
    public static final double TX_TOLERANCE = 1.5;
    public static final double MAX_TURN = 0.45;
    public static final double MIN_TURN = 0.08;

        //Calibrations

    public static final double CAMERA_HEIGHT_INCHES = 12;
    public static final double CAMERA_ANGLE_DEGREES = 30;

        //Real Constants

    public static final double TAG_HEIGHT = 0.762;
    public static final double MAX_ANGLE_2 = 180;

    public static final double MIN_ANGLE = 10;
    public static final double MAX_ANGLE = 80;

    public static final double MAX_STEP = 2.0; // degrees per loop

    public static final PID TILT_PID = new PID(0.5, 0, 0);





    // Intake
    public static final PID INTAKE_PID_HOLD = new PID(0.5, 0, 0);







    // Indexer
    public static double INDEXER_POS;
    public static final PID INDEXER_PID = new PID(0.5, 0, 0);



//Mouse
public static final double METERS_TO_INCHES = 39.3701;



    // Shooter
    public static double TARGET_RPM = 0;

    private static final float FEED_FORWARD = (float) 0.5;
    public static final PIDF VELOCITY_PID = new PIDF(0, 0, 0, FEED_FORWARD);

    public static final double[][] SHOOTER_LOOKUP = {
            // 3100 is good for 0.0
            {0.0, 2500},
            {1.341, 4500.0},
            {1.6378, 4900.0},
            {2.571, 5400}
    };

    public static final long DURATION_MS = 3600;
    public static final long DURATION_MS_AUTO = 4500;

    public static final double FLYWHEEL_GEAR_RATIO = 1.0 / 1.0;
    public static final double MOTOR_TICKS_PER_REV = 28;

    public static final long STABLE_DURATION_MS = 500;

    public static final long STABLE_DURATION_S = (long) 3.648;

    public static double LAST_RPM = TARGET_RPM;

    @NonNull
    public static SequentialCommandGroup SHOOT(CommandScheduler scheduler, NewRobot robot, long duration, boolean isAuto) {
        double distance = Limelight.getDistance(
                TAG_HEIGHT
        );
        if (isAuto) {
            return new SequentialCommandGroup(
                    scheduler,
                    new ConditionalCommand(
                            new ParallelCommandGroup(
                                    scheduler, Parameters.ANY,
                                    new CommandShoot(robot.shooterStates, distance),
                                    new CommandSpinRight(robot.sequencerState, duration)
                            ),
                            new CommandShoot(robot.shooterStates, distance),
                            () -> robot.shooterStates.isAtTargetVelocity()
                    ),

                    new CommandIntake(robot.intakeStates, 1000),

                    new ConditionalCommand(
                            new ParallelCommandGroup(
                                    scheduler, Parameters.ANY,
                                    new CommandShoot(robot.shooterStates, distance),
                                    new CommandSpinRight(robot.sequencerState, duration)
                            ),
                            new CommandShoot(robot.shooterStates, distance),
                            () -> robot.shooterStates.isAtTargetVelocity()
                    )
            );
        } else {
            return new SequentialCommandGroup(
                    scheduler, new ConditionalCommand(
                    new ParallelCommandGroup(
                            scheduler, Parameters.ANY,
                            new CommandShoot(robot.shooterStates, distance),
                            new CommandSpinRight(robot.sequencerState, duration)
                    ),
                    new CommandShoot(robot.shooterStates, distance),
                    () -> robot.shooterStates.isAtTargetVelocity()
            )
            );

        }
    }


    @NonNull
    public static SequentialCommandGroup IndexTest(CommandScheduler scheduler, NewRobot robot, long duration) {
        double distance = 0;
        duration = STABLE_DURATION_S * 1000;
          return new SequentialCommandGroup(
                scheduler,
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                scheduler, Parameters.ANY,
                                new CommandShoot(robot.shooterStates, distance),
                                new CommandQueue(robot.queueState, duration)
                        ),
                        new CommandWaitToShoot(robot.shooterStates, distance),
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


    public static final PID X_PID_P = new PID(3.6, 0, 1.9);
    public static final PID Y_PID_P = new PID(3.6, 0, 1.9);
    public static final PID YAW_PID_P = new PID(5, 0, 0);

}