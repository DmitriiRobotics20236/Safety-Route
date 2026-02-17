package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.TET;
import org.openftc.easyopencv.*;

@Autonomous(name="Sign Auto")
public class AutoVision extends LinearOpMode {

    OpenCvCamera camera;
    TET pipeline;

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode() {

        // Моторы
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Если нужно — развернуть один мотор
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Получаем ID для превью камеры
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        // 🔵 ВАЖНО: имя должно совпадать с конфигом!
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 1"),
                        cameraMonitorViewId
                );

        pipeline = new TET();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480);
            }

            @Override
            public void onError(int errorCode) {}
        });

        waitForStart();

        while (opModeIsActive()) {

            if (pipeline.signDetected) {
                // Едем вперёд
                leftMotor.setPower(0.5);
                rightMotor.setPower(0.5);
            } else {
                // Поворачиваем направо
                leftMotor.setPower(0.5);
                rightMotor.setPower(-0.5);
            }

            telemetry.addData("Sign detected", pipeline.signDetected);
            telemetry.update();
        }
    }
}
