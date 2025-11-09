package org.firstinspires.ftc.teamcode;

//import statements
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Tools.PID;

import java.util.List;

//begin class
@TeleOp
public class limeLightTurner extends LinearOpMode {


    //method call
    public void runOpMode() throws InterruptedException {

        //initializes limelight
        Limelight3A limelight;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        //initializes rightFront
        DcMotor right_front;
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initializes leftFront
        DcMotor left_front;
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initializes rightBack
        DcMotor right_back;
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initializes leftBack
        DcMotor left_back;
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //starts the code
        waitForStart();
        while (opModeIsActive()) {

            //gets the latest result from limelight
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                //initializes doubles
                double tx = result.getTx();
                double ty = result.getTy();
                double bx = result.getBotpose().getPosition().x;
                double by = result.getBotpose().getPosition().y;

                //initializes pid
                PID pidmt = new PID(0.04, 0.0, 0.0);
                PID pidrotx = new PID(0.04, 0.0, 0.0);
                PID pidroty = new PID(0.04, 0.0, 0.0);


                //continues double initialization


                double avgmotopos = (right_back.getCurrentPosition() + right_front.getCurrentPosition() + left_back.getCurrentPosition() + left_front.getCurrentPosition());
                double motorpowerdivid = (pidmt.updatePID(avgmotopos) + pidrotx.updatePID(tx) + pidroty.updatePID(ty));
                double rot = result.getBotpose().getOrientation().getYaw();


                //initializes integers
                int error = 1;
                int motorposr1 = right_front.getCurrentPosition();
                int motorposl1 = left_front.getCurrentPosition();
                int motorposr2 = right_back.getCurrentPosition();
                int motorposl2 = left_back.getCurrentPosition();
                int id = result.getFiducialResults().get(0).getFiducialId();
                int targetCount = fiducials.size();

                //initializes strings
                String family = result.getFiducialResults().get(0).getFamily();
                String curpos = String.valueOf(result.getFiducialResults().get(0).getRobotPoseFieldSpace());
                String robtarpos = String.valueOf(result.getFiducialResults().get(0).getRobotPoseTargetSpace());
                double power = Math.max(-0.4, Math.abs(motorpowerdivid));

                if (tx < -error && id == 24) {
                    right_front.setPower(-power);
                    left_front.setPower(-power);
                    right_back.setPower(power);
                    left_back.setPower(power);
                } else if (tx > error && id == 24) {
                    right_front.setPower(power);
                    left_front.setPower(power);
                    right_back.setPower(-power);
                    left_back.setPower(-power);
                } else if (targetCount == 0){
                    right_front.setPower(0);
                    left_front.setPower(0);
                    right_back.setPower(0);
                    left_back.setPower(0);
                } else {
                    telemetry.addData("status", "localized");
                }

                //shows telemetry on the driver station
                telemetry.addData("yaw", rot);
                telemetry.addData("pitch", ty);
                telemetry.addData("coordinates", "x: " + bx + " y: " + by);
                telemetry.addData("tx", tx);
                telemetry.addData("id", id);
                telemetry.addData("family", family);
                telemetry.addData("Current Position", curpos);
                telemetry.addData("Target Position in comparison to the robot", robtarpos);
                telemetry.addData("Current Motor Positions", "Right Front: " + motorposr1 + " Left Front: " + motorposl1 + " Right Back: " + motorposr2 + " Left Back: " + motorposl2);
                telemetry.update();
            }
        }
    }
}