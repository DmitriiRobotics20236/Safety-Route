package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoFlipAction_2 implements Action {

    public enum TargetPosition {
        A,
        B
    }

    private final Servo serv5;
    private final Servo serv6;
    private final double positionA;
    private final double positionB;
    private final TargetPosition target;
    private boolean done = false;

    public ServoFlipAction_2(Servo servo1, Servo servo2,
                           double positionA, double positionB,
                           TargetPosition target) {
        this.serv5 = servo1;
        this.serv6 = servo2;
        this.positionA = positionA;
        this.positionB = positionB;
        this.target = target;
    }

    // 🔑 Главное: метод с TelemetryPacket, как требует интерфейс
    @Override
    public boolean run(TelemetryPacket packet) {
        if (!done) {
            double pos = (target == TargetPosition.A) ? positionA : positionB;
            serv5.setPosition(pos);
            serv6.setPosition(pos);
            done = true;
        }
        return false; // Action выполнен
    }
}
