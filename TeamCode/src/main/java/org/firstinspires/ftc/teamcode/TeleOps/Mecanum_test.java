package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Mecanum_test extends LinearOpMode {
    DcMotor frontLeft, back_Left;
    DcMotor frontRight, back_right;

    IMU imu;

    double initialHeading = 0; // направление, в которое смотрел робот при старте

    @Override
    public void runOpMode() {
        // Инициализация моторов
        frontLeft = hardwareMap.get(DcMotor.class, "left_up");
        back_Left = hardwareMap.get(DcMotor.class, "left_down");
        frontRight = hardwareMap.get(DcMotor.class, "right_up");
        back_right = hardwareMap.get(DcMotor.class, "right_back");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Настройка IMU и ориентации хаба
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;

        initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (opModeIsActive()) {
            drive_mecanum();

        }

    }

    public void drive_mecanum() {
        // Получаем "сырые" значения с геймпада
        double rawY = -gamepad1.left_stick_y;
        double rawX = gamepad1.left_stick_x;
        double rawRx = gamepad1.right_stick_x;

        // Плавное управление: синус
        double y = Math.sin(rawY * Math.PI / 2);
        double x = Math.sin(rawX * Math.PI / 2);
        double rx = Math.sin(rawRx * Math.PI / 2); // по умолчанию — как от игрока

        // Ускорение от правого триггера
        double acceleration = 0.4 + 0.6 * gamepad1.right_trigger;

        // Получаем текущий угол (Yaw)
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Автовыравнивание: если поворот не выполняется (джойстик в покое)
        if (Math.abs(rawRx) < 0.05) {
            // Расчёт ошибки угла
            double headingError = normalizeAngle(currentHeading - initialHeading);
            double kP = 0.01; // коэффициент коррекции — можно подбирать
            rx = -headingError * kP; // отрицательный, чтобы компенсировать отклонение
        }

        // Сброс курса вручную
        if (gamepad1.options) {
            imu.resetYaw();
            initialHeading = 0; // обнуляем начальный курс
        }

        // Heading в радианах для field-centric
        double botHeadingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Преобразуем движение в поле-центричное
        double rotX = x * Math.cos(-botHeadingRad) - y * Math.sin(-botHeadingRad);
        double rotY = x * Math.sin(-botHeadingRad) + y * Math.cos(-botHeadingRad);

        rotX *= 1.1; // компенсация потерь

        // Нормализация и расчёт мощности
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = (rotY + rotX + rx) / denominator * acceleration;
        double backLeftPower = (rotY - rotX + rx) / denominator * acceleration;
        double frontRightPower = (rotY - rotX - rx) / denominator * acceleration;
        double backRightPower = (rotY + rotX - rx) / denominator * acceleration;

        // Применяем мощности
        frontLeft.setPower(frontLeftPower);
        back_Left.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        back_right.setPower(backRightPower);

        // Отображаем телеметрию
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addData("Initial Heading", initialHeading);
        telemetry.addData("Heading Error", normalizeAngle(currentHeading - initialHeading));
        telemetry.addData("Correction (rx)", rx);
        telemetry.addData("Acceleration", acceleration);
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
