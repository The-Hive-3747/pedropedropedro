package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import android.util.Size;

import java.util.List;

@Disabled
@TeleOp(name = "Color Blob Locator for Red/Blue", group = "Concept")
public class VisionTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Choose color to detect: change to "RED" or "BLUE" based on your team's task
        String colorToDetect = "RED";  // Change to "BLUE" for blue detection

        // Define the color range for the selected color
        ColorBlobLocatorProcessor colorLocator;
        if (colorToDetect.equals("RED")) {
            colorLocator = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.RED)  // Use predefined red color range
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // Search central 1/4 of camera view
                    .setDrawContours(true)
                    .setBlurSize(5)
                    .build();
        } else {
            colorLocator = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.BLUE)  // Use predefined blue color range
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // Search central 1/4 of camera view
                    .setDrawContours(true)
                    .setBlurSize(5)
                    .build();
        }

        // Build the vision portal with the color locator process
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(1280, 720))  //320 , 240 Use a lower resolution for better performance
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);  // Speed up telemetry updates, for debugging

        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Get the detected blobs
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            // Filter out unwanted blobs based on area
            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);

            // Optionally, you can sort the blobs based on area, density, or aspect ratio
            ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

            telemetry.addLine(" Area Density Aspect  Center");

            // Display the details of each detected blob
            for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                RotatedRect boxFit = blob.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        blob.getContourArea(), blob.getDensity(), blob.getAspectRatio(),
                        (int) boxFit.center.x, (int) boxFit.center.y));
            }

            telemetry.update();
            sleep(50);
        }
    }
}