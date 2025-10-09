package org.firstinspires.ftc.robotcontroller.external.samples.externalhardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class limeLightVision extends LinearOpMode {


    public class PID {
        private double kP, kI, kD;
        private double lastError = 0;
        private double integral = 0;

        public PID(double kp, double ki, double kd) {
            this.kP = kp;
            this.kI = ki;
            this.kD = kd;
        }

        public double update(double error) {
            integral += error;
            double derivative = error - lastError;
            lastError = error;

            return (kP * error) + (kI * integral) + (kD * derivative);
        }

        public void reset() {
            lastError = 0;
            integral = 0;
        }
    }


    public void runOpMode() throws InterruptedException {


        PID pid = new PID(0.2, 0.0, 0.0);

        Limelight3A limelight;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        DcMotor pivotMotor;
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DcMotor pivotMotor2;
        pivotMotor2 = hardwareMap.get(DcMotor.class, "pivotMotor2");
        pivotMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                double powerx = pid.update(tx);
                double powery = pid.update(ty);

                int error = 1;
                int motorpos1 = pivotMotor.getCurrentPosition();
                int motorpos2 = pivotMotor2.getCurrentPosition();
                int avgmotorpos = (motorpos1 + motorpos2)/2;
                int id = result.getFiducialResults().get(0).getFiducialId();

                String family = result.getFiducialResults().get(0).getFamily();
                String curpos = String.valueOf(result.getFiducialResults().get(0).getRobotPoseFieldSpace());
                String camtarpos = String.valueOf(result.getFiducialResults().get(0).getCameraPoseTargetSpace());
                String robtarpos = String.valueOf(result.getFiducialResults().get(0).getRobotPoseTargetSpace());

                if (tx <= Math.abs (error)) {
                    sleep(0);
                }else{
                    pivotMotor.setTargetPosition(10000);
                    pivotMotor2.setTargetPosition(10000);
                    pivotMotor.setPower(powerx);
                    pivotMotor2.setPower(powerx);

                }

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("ta", ta);
                telemetry.addData("id", id);
                telemetry.addData("family", family);
                telemetry.addData("Current Position", curpos);
                telemetry.addData("Target position in comparison to camera", camtarpos);
                telemetry.addData("Target Position in comparison to the robot", robtarpos);
                telemetry.addData("Current Motor Positions", motorpos1 +"", motorpos2);
                telemetry.update();
            }
        }
    }
}