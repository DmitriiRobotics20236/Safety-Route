package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Last_Mecanum {

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    // Коэффициент компенсации (подбирается экспериментально)
    double correctionMultiplier = 0.95; // если уводит вправо — уменьшаем левую сторону (<1)

    public Last_Mecanum(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("left_up");
        backLeftMotor = hardwareMap.dcMotor.get("left_down");
        frontRightMotor = hardwareMap.dcMotor.get("right_up");
        backRightMotor = hardwareMap.dcMotor.get("right_back");

        // Реверс левой стороны
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double y, double x, double rx, double leftTrigger) {

        // Если зажат левый триггер — едем ровно вперёд
        if (leftTrigger > 0.1) {
            driveStraight(leftTrigger);
            return;
        }

        x *= 1.81; // компенсация страфа

        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rx),
                1
        );

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    // Простая "ровная" езда вперёд через домножение
    private void driveStraight(double speed) {

        double leftPower = speed * correctionMultiplier;
        double rightPower = speed;

        frontLeftMotor.setPower(-leftPower);
        backLeftMotor.setPower(-leftPower);
        frontRightMotor.setPower(-rightPower);
        backRightMotor.setPower(-rightPower);
    }
}