package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.modules.Servo_claw_plastina;

public class PlastinaAction implements Action {

    private final Servo_claw_plastina claw;
    private final boolean open; // true = открыть, false = закрыть
    private boolean executed = false; // чтобы сделать действие один раз

    public PlastinaAction(Servo_claw_plastina claw, boolean open) {
        this.claw = claw;
        this.open = open;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!executed) {
            claw.update(open);
            executed = true;
        }
        return true; // возвращаем true, так как действие мгновенное и завершено
    }
}
