package org.firstinspires.ftc.robotcontroller.external.samples.externalhardware;

//import statements
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//begin class
@TeleOp
public class limeLightVision extends LinearOpMode {

//inner class to initialize PID
    public static class PID {
        private final double kP;
        private final double kI;
        private final double kD;
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

//method call
    public void runOpMode() throws InterruptedException {

//sets p, i, and d values for PID
        PID pid = new PID(0.05, 0.0, 0.0);

        //initializes limelight
        Limelight3A limelight;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        //initializes rightFront
        DcMotor rightFront;
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //initializes leftFront
        DcMotor leftFront;
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //initializes rightBack
        DcMotor rightBack;
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //initializes leftBack
        DcMotor leftBack;
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //starts the code
        waitForStart();
        while (opModeIsActive()) {

            //gets the latest result from limelight
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {

                //initializes doubles
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                double powerx = pid.update(tx);
                double powery = pid.update(ty);

                //initializes integers
                int error = 1;
                int motorposr1 = rightFront.getCurrentPosition();
                int motorposl1 = leftFront.getCurrentPosition();
                int motorposr2 = rightBack.getCurrentPosition();
                int motorposl2 = leftBack.getCurrentPosition();
                int avgmotorpos = (motorposr1 + motorposl1 + motorposr2 + motorposl2)/4;
                int id = result.getFiducialResults().get(0).getFiducialId();

                //initializes strings
                String family = result.getFiducialResults().get(0).getFamily();
                String curpos = String.valueOf(result.getFiducialResults().get(0).getRobotPoseFieldSpace());
                String robtarpos = String.valueOf(result.getFiducialResults().get(0).getRobotPoseTargetSpace());

                //tracks the Apriltag in x
                if (tx <= Math.abs (error)) {
                    sleep(0);
                }else{
                    //sets motor positions for x
                    leftFront.setTargetPosition((int) (avgmotorpos-(Math.abs(tx))));
                    rightFront.setTargetPosition(-(int) (avgmotorpos-(Math.abs(tx))));
                    leftBack.setTargetPosition((int) (avgmotorpos-(Math.abs(tx))));
                    rightBack.setTargetPosition(-(int) (avgmotorpos-(Math.abs(tx))));
                    //sets motor power for x
                    leftFront.setPower(powerx);
                    rightFront.setPower(-powerx);
                    leftBack.setPower(powerx);
                    rightBack.setPower(-powerx);
                }

                //tracks the Apriltag in y
                if (ty <= Math.abs (error)) {
                    sleep(0);
                }else{
                    //sets motor positions for y
                    leftFront.setTargetPosition((int) (avgmotorpos-(Math.abs(ty))));
                    rightFront.setTargetPosition(-(int) (avgmotorpos-(Math.abs(ty))));
                    leftBack.setTargetPosition((int) (avgmotorpos-(Math.abs(ty))));
                    rightBack.setTargetPosition(-(int) (avgmotorpos-(Math.abs(ty))));
                    //sets motor power for y
                    leftFront.setPower(powery);
                    rightFront.setPower(powery);
                    leftBack.setPower(-powery);
                    rightBack.setPower(-powery);
                }

                //shows telemetry on the driver station
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("ta", ta);
                telemetry.addData("id", id);
                telemetry.addData("family", family);
                telemetry.addData("Current Position", curpos);
                telemetry.addData("Target Position in comparison to the robot", robtarpos);
                telemetry.addData("Current Motor Positions", avgmotorpos);
                telemetry.update();
            }
        }
    }
}