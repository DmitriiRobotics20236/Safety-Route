
package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Mecanum_IMU_Complete")
public class DT extends LinearOpMode {

    // ===== МОТОРЫ =====
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // ===== IMU =====
    IMU imu;

    // ===== Heading Hold =====
    double targetHeading = 0;
    boolean headingLocked = false;

    // ===== ПЛАВНОСТЬ =====
    double fl = 0, fr = 0, bl = 0, br = 0;
    static final double SMOOTH = 0.20;

    // ===== НАСТРОЙКИ =====
    static final double TURN_DEADZONE = 0.05;
    static final double HEADING_KP = 0.1; // подбирается

    @Override
    public void runOpMode() {

        // ===== МОТОРЫ =====
        frontLeft = hardwareMap.get(DcMotor.class, "left_up");
        frontRight = hardwareMap.get(DcMotor.class, "right_up");
        backLeft = hardwareMap.get(DcMotor.class, "left_down");
        backRight = hardwareMap.get(DcMotor.class, "right_back");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // ===== IMU CONFIG (Logo UP / USB FORWARD) =====
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();

        while (opModeIsActive()) {

            // ===== СТИКИ =====
            double y = gamepad1.left_stick_y;   // вперёд / назад
            double x = gamepad1.left_stick_x;    // вбок
            double turnInput = gamepad1.right_stick_x;

            // deadzone поворота
            if (Math.abs(turnInput) < TURN_DEADZONE) {
                turnInput = 0;
            }

            // ===== УСКОРЕНИЕ =====
            double boost = gamepad1.right_trigger;
            double speed = 0.5 + boost * 0.75;

            // ===== IMU =====
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double currentHeading = angles.getYaw(AngleUnit.RADIANS);

            double rotation;

            if (turnInput != 0) {
                // водитель крутит сам
                rotation = turnInput;
                targetHeading = currentHeading;
                headingLocked = false;
            } else {
                // удержание текущего курса
                if (!headingLocked) {
                    targetHeading = currentHeading;
                    headingLocked = true;
                }

                double error = targetHeading - currentHeading;
                error = Math.atan2(Math.sin(error), Math.cos(error)); // нормализация

                rotation = error * HEADING_KP;
            }

            // ===== МЕКАНУМ (SIN / COS) =====
            double r = Math.hypot(x, y);
            double angle = Math.atan2(y, x) - Math.PI / 4;

            double sin = Math.sin(angle);
            double cos = Math.cos(angle);

            double max = Math.max(Math.abs(sin), Math.abs(cos));
            if (max < 1e-6) max = 1;

            double flTarget = (r * cos / max + rotation) * speed;
            double frTarget = (r * sin / max - rotation) * speed;
            double blTarget = (r * sin / max + rotation) * speed;
            double brTarget = (r * cos / max - rotation) * speed;


            double maxPower = Math.max(
                    Math.max(Math.abs(flTarget), Math.abs(frTarget)),
                    Math.max(Math.abs(blTarget), Math.abs(brTarget))
            );

            if (maxPower > 1.0) {
                flTarget /= maxPower;
                frTarget /= maxPower;
                blTarget /= maxPower;
                brTarget /= maxPower;
            }

            // ===== ПЛАВНОСТЬ =====
            fl += (flTarget - fl) * SMOOTH;
            fr += (frTarget - fr) * SMOOTH;
            bl += (blTarget - bl) * SMOOTH;
            br += (brTarget - br) * SMOOTH;

            // ===== МОТОРЫ =====
            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // ===== TELEMETRY =====
            telemetry.addData("Yaw", Math.toDegrees(currentHeading));
            telemetry.addData("Target", Math.toDegrees(targetHeading));
            telemetry.addData("Speed", speed);
            telemetry.update();
        }
    }
}