package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Feild.Peripherals;

public class FieldOfMerit {


    private static double fieldY;
    private static double fieldX;
    private static double theta;
    public static String currentState = "Odometry Pods";



    public static void initialize(HardwareMap hardwareMap) {
        Drive drive = new Drive("drive",hardwareMap);
        Peripherals.initialize(hardwareMap);

        Peripherals.resetYaw();
    }



    public static void processTags() {

/*        double X = Peripherals.getLimelightX();
        double Y = Peripherals.getLimelightY();
        double robotYaw = Peripherals.getYaw();*/
       /* double X = 0;
        double Y = 0;
        double robotYaw = 0;

        if (X != 0 || Y != 0) {

            currentState = "Vision";

            fieldX = X;
            fieldY = Y;
            theta = robotYaw;

            DriveCommand.setPosition(fieldX, fieldY, theta);
            FinalPose.setfinalPose(fieldX, fieldY, theta);
        }

        else {


            fieldX = DriveCommand.getOdometryX();
            fieldY = DriveCommand.getOdometryY();
            theta = DriveCommand.getOdometryTheta();

            currentState = "Odometry Pods";
            FinalPose.setfinalPose(fieldX, fieldY, theta);
        }*/
    }

    public double getFieldX() {
        return fieldX;
    }

    public double getFieldY() {
        return fieldY;
    }

    public double getTheta() {
        return theta;
    }


}






