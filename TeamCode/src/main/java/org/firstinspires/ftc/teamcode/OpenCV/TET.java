package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.*;

public class TET extends OpenCvPipeline {

    Mat hsv = new Mat();
    Mat blueMask = new Mat();
    Mat blackMask = new Mat();
    Mat hierarchy = new Mat();

    public boolean signDetected = false;

    @Override
    public Mat processFrame(Mat input) {

        signDetected = false;

        // HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // --- Голубой цвет ---
        Core.inRange(
                hsv,
                new Scalar(90, 80, 80),
                new Scalar(130, 255, 255),
                blueMask
        );

        // Убираем шум
        Imgproc.morphologyEx(
                blueMask,
                blueMask,
                Imgproc.MORPH_CLOSE,
                Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5))
        );

        // Контуры голубого
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(
                blueMask,
                contours,
                hierarchy,
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        for (MatOfPoint contour : contours) {

            double area = Imgproc.contourArea(contour);
            if (area < 3000) continue; // отсеиваем мелкие объекты

            Rect rect = Imgproc.boundingRect(contour);

            // Проверка формы (почти квадрат)
            double ratio = (double) rect.width / rect.height;
            if (ratio < 0.7 || ratio > 1.3) continue;

            // --- Проверяем чёрный цвет внутри ---
            Mat roi = hsv.submat(rect);

            Core.inRange(
                    roi,
                    new Scalar(0, 0, 0),
                    new Scalar(180, 255, 60),
                    blackMask
            );

            double blackPixels = Core.countNonZero(blackMask);

            if (blackPixels > rect.area() * 0.05) {
                // НАШЛИ ЗНАК
                signDetected = true;

                // Рисуем рамку
                Imgproc.rectangle(
                        input,
                        rect,
                        new Scalar(0, 255, 0),
                        3
                );

                Imgproc.putText(
                        input,
                        "PEDESTRIAN SIGN",
                        new Point(rect.x, rect.y - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        new Scalar(0, 255, 0),
                        2
                );

                roi.release();
                break;
            }

            roi.release();
        }

        return input;
    }
}
