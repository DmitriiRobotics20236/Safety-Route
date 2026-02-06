package org.firstinspires.ftc.teamcode.TeleOps;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "DriveV2")
public class Drive_test extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Для плавности
    private double flPower = 0, frPower = 0, blPower = 0, brPower = 0;
    private static final double SMOOTHING = 0.08; // чем меньше — тем плавнее

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotor.class, "left_up");
        frontRight = hardwareMap.get(DcMotor.class, "right_up");
        backLeft   = hardwareMap.get(DcMotor.class, "left_down");
        backRight  = hardwareMap.get(DcMotor.class, "right_back");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            // ===== СТИКИ =====
            double y = gamepad1.left_stick_y;   // вперёд / назад
            double x = gamepad1.left_stick_x;    // вбок
            double rotation = gamepad1.right_stick_x; // поворот

            // ===== УСКОРЕНИЕ =====
            double boost = gamepad1.right_trigger; // 0..1
            double speedFactor = 0.4 + boost * 0.6; // базовая + ускорение

            // ===== МЕКАНУМ ЧЕРЕЗ SIN / COS =====
            double r = Math.hypot(x, y);
            double robotAngle = Math.atan2(y, x) - Math.PI / 4;

            double sin = Math.sin(robotAngle);
            double cos = Math.cos(robotAngle);

            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double flTarget = (r * cos / max + rotation) * speedFactor;
            double frTarget = (r * sin / max - rotation) * speedFactor;
            double blTarget = (r * sin / max + rotation) * speedFactor;
            double brTarget = (r * cos / max - rotation) * speedFactor;

            // ===== НОРМАЛИЗАЦИЯ =====
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

            // ===== ПЛАВНОСТЬ (RAMP) =====
            flPower += (flTarget - flPower) * SMOOTHING;
            frPower += (frTarget - frPower) * SMOOTHING;
            blPower += (blTarget - blPower) * SMOOTHING;
            brPower += (brTarget - brPower) * SMOOTHING;

            // ===== МОТОРЫ =====
            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);

            telemetry.addData("Speed", speedFactor);
            telemetry.update();
        }
    }
}
