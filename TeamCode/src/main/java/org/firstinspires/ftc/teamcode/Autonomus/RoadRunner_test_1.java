package org.firstinspires.ftc.teamcode.Autonomus;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Actions.PlastinaAction;
import org.firstinspires.ftc.teamcode.Actions.ServoFlipAction;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.modules.Servo_claw_plastina;
import org.firstinspires.ftc.teamcode.modules.Servo_flip;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "first_auto", group = "AUTO")
public class RoadRunner_test_1 extends LinearOpMode {
    Servo servo3, servo4; // серво переворот

    Servo servo1, servo2; // серво пластина


    @Override
    public void runOpMode() {
        // СервоПластина
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo_claw_plastina claw1 = new Servo_claw_plastina(servo1, 0.04, 0.09); //левый
        Servo_claw_plastina claw2 = new Servo_claw_plastina(servo2, 0.06, 0.0); //правый

        //СервоПереворот
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo4.setDirection(Servo.Direction.REVERSE);
        double POS_A = -1.0;
        double POS_B = 0.45;
        //Езда
        Pose2d beginPose = new Pose2d(new Vector2d(-70, -12), Math.toRadians(0));
        MecanumDrive driveMEC = new MecanumDrive(hardwareMap, beginPose);


        waitForStart();
        Action path = driveMEC.actionBuilder(beginPose)
                .splineTo(new Vector2d(48, 48), Math.PI)
                .stopAndAdd((new ServoFlipAction(servo3, servo4, POS_A, POS_B,
                        ServoFlipAction.TargetPosition.A)))
                .splineTo(new Vector2d(45,45), Math.PI)
                .stopAndAdd(new PlastinaAction(claw1,true))
                .stopAndAdd(new PlastinaAction(claw2,false))
                .build();


        Actions.runBlocking(new SequentialAction(path));

    }
}
