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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "first_auto", group = "AUTO")
public class RoadRunner_test_1 extends LinearOpMode {

    Servo claw1;



    public class OpencClaw implements InstantFunction {

 
        @Override
        public void run() {
            claw1.setPosition(0);

        }
    }

    @Override
    public void runOpMode() {
        claw1 = hardwareMap.get(Servo.class, "clawq");


        Pose2d beginPose = new Pose2d(new Vector2d(-70, -12), Math.toRadians(0));

        MecanumDrive driveMEC = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        Action path = driveMEC.actionBuilder(beginPose)
                .splineTo(new Vector2d(48, 48), Math.PI)
                .build();


        Actions.runBlocking(new SequentialAction(path));



    }


}
