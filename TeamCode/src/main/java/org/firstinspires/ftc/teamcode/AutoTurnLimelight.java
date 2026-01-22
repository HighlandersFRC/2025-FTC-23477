package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Tools.Limelight;
import org.firstinspires.ftc.teamcode.Tools.PID;

@TeleOp(name = "Auto Turn Limelight Button")
public class AutoTurnLimelight extends LinearOpMode {

    private Drive drive;

    private final PID turnPID = new PID(0.045, 0.0, 0.030);
    private double lastTx = 0;

    private static final double TX_TOLERANCE = 1.5; // degrees
    private static final double MAX_TURN = 0.45;
    private static final double MIN_TURN = 0.08;

    private boolean autoTurnActive = false;
    private boolean xPreviouslyPressed = false;
    private boolean targetReachedPrinted = false;

    @Override
    public void runOpMode() {

        drive = new Drive("drive", hardwareMap);
        Limelight.init(hardwareMap);

        turnPID.setSetPoint(0.0);

        telemetry.addLine("Auto Turn Limelight Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Toggle auto-turn with X button
            if (gamepad1.x && !xPreviouslyPressed) {
                autoTurnActive = !autoTurnActive;
                targetReachedPrinted = false; // reset message
            }
            xPreviouslyPressed = gamepad1.x;

            LLResult result = Limelight.getResult();

            if (autoTurnActive && result != null && result.isValid()) {

                double tx = result.getTx();

                // Smooth tx
                double smoothTx = 0.3 * lastTx + 0.7 * tx;
                lastTx = smoothTx;

                // PID turn
                double turnPower = -turnPID.updatePID(smoothTx);

                // Deadband
                if (Math.abs(smoothTx) < TX_TOLERANCE) {
                    turnPower = 0;
                    if (targetReachedPrinted) {
                        telemetry.addLine("Target Reached!");
                        targetReachedPrinted = false;
                    }
                }

                // Min power
                if (turnPower != 0 && Math.abs(turnPower) < MIN_TURN) {
                    turnPower = Math.signum(turnPower) * MIN_TURN;
                }

                // Clamp
                turnPower = Math.max(-MAX_TURN, Math.min(MAX_TURN, turnPower));

                // Drive turn
                drive.drive(turnPower, -turnPower, turnPower, turnPower);

                telemetry.addData("Mode", "AUTO TURN");
                telemetry.addData("TX", "%.2f", tx);
                telemetry.addData("TurnPower", "%.2f", turnPower);
                telemetry.addData("Is Reached", Math.abs(smoothTx) < TX_TOLERANCE);

            } else {
                // Manual field-centric driving
                drive.FieldCentric(gamepad1);
                telemetry.addData("Mode", "MANUAL - Field Centric");
            }

            telemetry.update();
        }
    }
}
