package org.firstinspires.ftc.teamcode.RoadRunner_classes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive_mec_drive {

    // ===== Моторы =====
    private DcMotor fl, fr, bl, br;
    private IMU imu;
    private Telemetry telemetry;

    // ===== PID коэффициенты =====
    private double kP = 0.1755;
    private double kI = 0.0000001;
    private double kD = 0.021;

    private double targetHeading;
    private double integralSum = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    public Drive_mec_drive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        fl = hardwareMap.get(DcMotor.class, "left_up");
        fr = hardwareMap.get(DcMotor.class, "right_up");
        bl = hardwareMap.get(DcMotor.class, "left_down");
        br = hardwareMap.get(DcMotor.class, "right_back");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

        targetHeading = getHeading();
        timer.reset();
    }

    // ===== Основной метод движения =====
    public void drive(double forward,
                      double rotationInput,
                      boolean leftBumper, boolean rightBumper,
                      double boost) {

        double x = 0;
        double y = forward;

        // ===== Крабовый ход =====
        if (leftBumper) {
            x = -1.0 * 1.1;
            y = 0.0;
            rotationInput = 0;
        } else if (rightBumper) {
            x = 1.0 * 1.1;
            y = 0.0;
            rotationInput = 0;
        }

        double speedMultiplier = 0.8 + boost * 0.8;

        double magnitude = Math.hypot(x, y);
        double angle = Math.atan2(y, x);

        double sin = Math.sin(angle - Math.PI / 4);
        double cos = Math.cos(angle - Math.PI / 4);

        double rotation;

        // ===== Удержание угла =====
        if (Math.abs(rotationInput) < 0.05) {

            double error = angleWrap(targetHeading - getHeading());

            double dt = timer.seconds();
            timer.reset();

            integralSum += error * dt;
            double derivative = (error - lastError) / dt;

            rotation = error * kP
                    + integralSum * kI
                    + derivative * kD;

            lastError = error;

        } else {
            rotation = rotationInput;
            targetHeading = getHeading();
            integralSum = 0;
        }

        // ===== Моторы =====
        double flPower = magnitude * cos + rotation;
        double frPower = magnitude * sin - rotation;
        double blPower = magnitude * sin + rotation;
        double brPower = magnitude * cos - rotation;

        double max = Math.max(1.0,
                Math.max(Math.abs(flPower),
                        Math.max(Math.abs(frPower),
                                Math.max(Math.abs(blPower), Math.abs(brPower)))));

        fl.setPower(flPower / max * speedMultiplier);
        fr.setPower(frPower / max * speedMultiplier);
        bl.setPower(blPower / max * speedMultiplier);
        br.setPower(brPower / max * speedMultiplier);

        telemetry.addData("Heading", Math.toDegrees(getHeading()));
        telemetry.addData("Target", Math.toDegrees(targetHeading));
    }

    // ===== IMU =====
    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
