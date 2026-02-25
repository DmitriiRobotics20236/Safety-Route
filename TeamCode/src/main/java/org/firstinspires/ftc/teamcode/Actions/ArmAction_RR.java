package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmAction_RR {

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    public ArmAction_RR(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor_arm");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor_arm");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Всегда работаем без энкодеров
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    // ===== Action по времени =====
    public Action createTimedAction(double moveTimeSeconds, double power, Telemetry telemetry) {

        return new Action() {

            private final ElapsedTime timer = new ElapsedTime();
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {

                if (!started) {
                    timer.reset();

                    leftMotor.setPower(power);
                    rightMotor.setPower(power);

                    started = true;
                }

                double currentTime = timer.seconds();

                // Телеметрия
                if (telemetry != null) {
                    telemetry.addData("Arm Time", currentTime);
                    telemetry.addData("Arm Power", power);
                    telemetry.update();
                }

                packet.put("ArmTime", currentTime);

                // Остановить по времени
                if (currentTime >= moveTimeSeconds) {
                    stop();
                    return false; // завершить Action
                }

                return true;
            }
        };
    }
}