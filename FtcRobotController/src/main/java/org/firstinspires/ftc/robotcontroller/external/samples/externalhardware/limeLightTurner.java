package org.firstinspires.ftc.robotcontroller.external.samples.externalhardware;

//import statements
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//begin class
@TeleOp
public class limeLightTurner extends LinearOpMode {


//method call
    public void runOpMode() throws InterruptedException {

//sets p, i, and d values for PID
        PID pidrf = new PID(0.05, 0.0, 0.0);
        PID pidrb = new PID(0.05, 0.0, 0.0);
        PID pidlf = new PID(0.05, 0.0, 0.0);
        PID pidlb = new PID(0.05, 0.0, 0.0);


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

                //initializes doubles
                double tx = result.getTx();
                double ty = result.getTy();
                double bx = result.getBotpose().getPosition().x;
                double by = result.getBotpose().getPosition().y;
                double powerlf = pidlf.update(tx);
                double powerlb = pidlb.update(tx);
                double powerrf = pidrf.update(tx);
                double powerrb = pidrb.update(tx);
                double rotylf = pidlf.update(by);
                double rotylb = pidlb.update(by);
                double rotyrf = pidrf.update(by);
                double rotyrb = pidrb.update(by);
                double rotxlf = pidlf.update(bx);
                double rotxlb = pidlb.update(bx);
                double rotxrf = pidlf.update(bx);
                double rotxrb = pidlb.update(bx);

                powerlf = Math.max(-0.4, Math.min(0.4, powerlf));
                powerlb = Math.max(-0.4, Math.min(0.4, powerlb));
                powerrf = Math.max(-0.4, Math.min(0.4, powerrf));
                powerrb = Math.max(-0.4, Math.min(0.4, powerrb));

                //initializes integers
                int error = 1;
                int motorposr1 = right_front.getCurrentPosition();
                int motorposl1 = left_front.getCurrentPosition();
                int motorposr2 = right_back.getCurrentPosition();
                int motorposl2 = left_back.getCurrentPosition();
                int id = result.getFiducialResults().get(0).getFiducialId();

                //initializes strings
                String family = result.getFiducialResults().get(0).getFamily();
                String curpos = String.valueOf(result.getFiducialResults().get(0).getRobotPoseFieldSpace());
                String robtarpos = String.valueOf(result.getFiducialResults().get(0).getRobotPoseTargetSpace());

                if (tx < -error) {
                    right_front.setPower((powerrf) + (rotxrf) + (rotyrf)/3 );
                    left_front.setPower((-powerlf) + (-rotylf) + (-rotxlf)/3);
                    right_back.setPower((-powerrb) + (-rotyrb) + (-rotxrb)/3);
                    left_back.setPower((powerlb) + (rotxlb) + (rotylb)/3);
                } else if (tx > error) {
                    right_front.setPower((-powerrf) + (-rotxrf) + (-rotyrf)/3);
                    left_front.setPower((powerlf) + (rotxlf) + (rotylf)/3);
                    right_back.setPower((powerrb) + (rotxrb) + (rotyrb)/3);
                    left_back.setPower((-powerlb) + (-rotxlb) + (-rotylb)/3);
                } else{
                    right_front.setPower(0);
                    left_front.setPower(0);
                    right_back.setPower(0);
                    left_back.setPower(0);
                }

                //shows telemetry on the driver station
                telemetry.addData("yaw", tx);
                telemetry.addData("pitch", ty);
                telemetry.addData("coordinates", "x: " + bx + " y: " + by);
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