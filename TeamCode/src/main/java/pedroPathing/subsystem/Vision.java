package pedroPathing.subsystem;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

public class Vision {
    private static int VISION_LEFT_CLOSE_TICKS = 0;
    private static int VISION_RIGHT_CLOSE_TICKS = 0;
    private static int VISION_LEFT_MEDIUM_TICKS = 665;
    private static int VISION_RIGHT_MEDIUM_TICKS = 665;
    private static int VISION_LEFT_FAR_TICKS = 1008;
    private static int VISION_RIGHT_FAR_TICKS = 1008;
    // Choose color to detect: change to "RED" or "BLUE" based on your team's task
    String colorToDetect = "RED";  // Change to "BLUE" for blue detection

    // Define the color range for the selected color
    ColorBlobLocatorProcessor colorLocator;
    HardwareMap hardwareMap = null;
    Telemetry telemetry = null;
    VisionPortal portal = null;
    public class BlobPoint {
        public int x = 0;
        public int y = 0;
        public double aspectRatio = 0.0;
    }
    public Vision(HardwareMap hmap, Telemetry tele) {
        hardwareMap = hmap;
        telemetry = tele;
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
            portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))  // Use a lower resolution for better performance
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
            portal.stopStreaming();
    }
    public void stopVision() {
        portal.stopStreaming();
    }
    public void startVision() {
        portal.resumeStreaming();
    }
    public boolean isVisionReady() {
        return portal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY;
    }
    public BlobPoint findClosestBlob(){
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
        //Put it here so its not mad
        return null;
    }
}
