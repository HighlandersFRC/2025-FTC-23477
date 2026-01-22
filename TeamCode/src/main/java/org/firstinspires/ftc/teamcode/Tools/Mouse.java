package org.firstinspires.ftc.teamcode.Tools;


import static org.firstinspires.ftc.teamcode.Tools.Constants.METERS_TO_INCHES;

import com.qualcomm.robotcore.hardware.HardwareMap;
public class Mouse {
    private static double fieldX;
    private static double fieldY;
    private static double theta;
    private static SparkFunOTOS mouse;

    public static void init(HardwareMap hardwareMap) {
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
    }
    public static void configureOtos() {
        mouse.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
        mouse.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.1524, -0.1016, 0);
        mouse.setOffset(offset);
        mouse.setLinearScalar(0.96292729898); // Calibrate
        mouse.setAngularScalar(1.01530630663); // Calibrate
        mouse.calibrateImu();
        mouse.resetTracking();
        System.out.println("configured");
        System.out.println("theta"+offset.h);
    }
    public static void setPosition(double x, double y, double theta){
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, theta);
        mouse.setPosition(currentPosition);
    }
    public static void update() {
        SparkFunOTOS.Pose2D field = mouse.getPosition();
        fieldX = field.x;
        fieldY = field.y;
        theta = field.h;
    }
    public static double getX() {
        return fieldX;
    }
    public static double getY() {
        return fieldY;
    }
    public static double getTheta() {
        return theta;
    }

    public static double getXM() {
    return getX() * METERS_TO_INCHES;
    }
    public static double getYM() {
        return getY() * METERS_TO_INCHES;
    }



}