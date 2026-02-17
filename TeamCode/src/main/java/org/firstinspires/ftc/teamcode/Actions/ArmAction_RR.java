package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// ===== Класс управления рукой и создание Action для RoadRunner =====
public class ArmAction_RR {

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    public ArmAction_RR(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor_arm");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor_arm");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    // ===== Сброс энкодеров =====
    public void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ===== Асинхронное движение руки =====
    public void moveToPositionAsync(int targetTicks, double power) {
        leftMotor.setTargetPosition(targetTicks);
        rightMotor.setTargetPosition(targetTicks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(Math.abs(power));
        rightMotor.setPower(Math.abs(power));
    }

    public boolean isBusy() {
        return leftMotor.isBusy() || rightMotor.isBusy();
    }

    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public int getCurrentPosition() {
        return leftMotor.getCurrentPosition();
    }

    // ===== Метод для создания Action для RoadRunner =====
    public Action createAction(int targetTicks, double power, Telemetry telemetry) {
        return new Action() {
            private boolean executed = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!executed) {
                    moveToPositionAsync(targetTicks, power);
                    executed = true;
                    if (telemetry != null) {
                        telemetry.addData("ArmAction", "Moved to " + targetTicks);
                        telemetry.update();
                    }
                }
                return true; // мгновенное завершение Action
                // если нужно ждать окончания движения: return !isBusy();
            }
        };
    }
}
