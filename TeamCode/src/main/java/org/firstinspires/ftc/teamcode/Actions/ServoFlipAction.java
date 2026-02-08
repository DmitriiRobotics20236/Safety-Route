package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoFlipAction implements Action {

    public enum TargetPosition {
        A,
        B
    }

    private final Servo servo3;
    private final Servo servo4;
    private final double positionA;
    private final double positionB;
    private final TargetPosition target;
    private boolean done = false;

    public ServoFlipAction(Servo servo1, Servo servo2,
                           double positionA, double positionB,
                           TargetPosition target) {
        this.servo3 = servo1;
        this.servo4 = servo2;
        this.positionA = positionA;
        this.positionB = positionB;
        this.target = target;
    }

    // 🔑 Главное: метод с TelemetryPacket, как требует интерфейс
    @Override
    public boolean run(TelemetryPacket packet) {
        if (!done) {
            double pos = (target == TargetPosition.A) ? positionA : positionB;
            servo3.setPosition(pos);
            servo4.setPosition(pos);
            done = true;
        }
        return false; // Action выполнен
    }
}
