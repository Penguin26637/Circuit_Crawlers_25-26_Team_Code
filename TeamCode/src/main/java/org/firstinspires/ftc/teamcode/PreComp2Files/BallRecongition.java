    package org.firstinspires.ftc.teamcode.PreComp2Files;

    import android.graphics.Color;
    import android.util.Size;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
    import org.firstinspires.ftc.vision.VisionPortal;
    import org.firstinspires.ftc.vision.opencv.Circle;
    import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
    import org.firstinspires.ftc.vision.opencv.ColorRange;



    import java.util.List;
    import java.util.concurrent.TimeUnit;

    @TeleOp(name = "BallRecongition", group = "Camera Vision")
    public class BallRecongition extends LinearOpMode {

        // Toggle for showing radius telemetry
        private boolean showRadius = true;
        public ExposureControl myExposureControl;
        public FocusControl myFocusControl;
        public PtzControl myPtzControl;
        public GainControl myGainControl;
        public WhiteBalanceControl myWhiteBalanceControl;

        @Override
        public void runOpMode() {



            // Green detector
            ColorBlobLocatorProcessor greenLocator =
                    new ColorBlobLocatorProcessor.Builder()
                            .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                            .setDrawContours(true)
                            .setCircleFitColor(Color.GREEN)
                            .setBlurSize(5)
                            .build();

            // Purple detector
            ColorBlobLocatorProcessor purpleLocator =
                    new ColorBlobLocatorProcessor.Builder()
                            .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                            .setDrawContours(true)
                            .setCircleFitColor(Color.MAGENTA)
                            .setBlurSize(5)
                            .build();

            VisionPortal portal = new VisionPortal.Builder()
                    .addProcessor(greenLocator)
                    .addProcessor(purpleLocator)
                    .setCameraResolution(new Size(320, 240))
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .build();

            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.startCameraStream(portal, 30);


    //        myExposureControl = portal.getCameraControl(ExposureControl.class);
    //        myFocusControl = portal.getCameraControl(FocusControl.class);
    //        myGainControl = portal.getCameraControl(GainControl.class);
    //        myPtzControl = portal.getCameraControl(PtzControl.class);
    //        myWhiteBalanceControl = portal.getCameraControl(WhiteBalanceControl.class);


    //        myExposureControl.setExposure(1, TimeUnit.MILLISECONDS);
            telemetry.setMsTransmissionInterval(100);

            waitForStart();

            while (opModeIsActive() || opModeInInit()) {
                myExposureControl = portal.getCameraControl(ExposureControl.class);
                myFocusControl = portal.getCameraControl(FocusControl.class);
                myGainControl = portal.getCameraControl(GainControl.class);
                myPtzControl = portal.getCameraControl(PtzControl.class);
                myWhiteBalanceControl = portal.getCameraControl(WhiteBalanceControl.class);
                myExposureControl.setExposure(1, TimeUnit.MILLISECONDS);
                myWhiteBalanceControl.setWhiteBalanceTemperature(6500);
                telemetry.addData("preview on/off", "... Camera Stream\n");

                List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenLocator.getBlobs();
                List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();

                // ---------------- Blob Filtering ----------------

                // Strict contour area filter
                ColorBlobLocatorProcessor.Util.filterByCriteria(
                        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 200, 20000, greenBlobs);
                ColorBlobLocatorProcessor.Util.filterByCriteria(
                        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 200, 20000, purpleBlobs);

                // Stricter circularity filter
                ColorBlobLocatorProcessor.Util.filterByCriteria(
                        ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.5, 1.0, greenBlobs);
                ColorBlobLocatorProcessor.Util.filterByCriteria(
                        ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.5, 1.0, purpleBlobs);

                // Remove all small blobs (radius < 12)
    //            greenBlobs.removeIf(b -> b.getCircle().getRadius() < 12);
    //            purpleBlobs.removeIf(b -> b.getCircle().getRadius() < 12);

                // Update detection flags
    //            boolean greenDetected = !greenBlobs.isEmpty();
    //            boolean purpleDetected = !purpleBlobs.isEmpty();

                // Find closest ball by distance to center
                double frameCenterX = 320 / 2.0;
                double frameCenterY = 240 / 2.0;
                Circle closestCircle = null;
                String closestColor = "None";
                double minDistance = Double.MAX_VALUE;

                for (ColorBlobLocatorProcessor.Blob b : greenBlobs) {
                    Circle c = b.getCircle();
                    double dist = Math.hypot(c.getX() - frameCenterX, c.getY() - frameCenterY);
                    if (dist < minDistance) {
                        minDistance = dist;
                        closestCircle = c;
                        closestColor = "Green";
                    }
                }

                for (ColorBlobLocatorProcessor.Blob b : purpleBlobs) {
                    Circle c = b.getCircle();
                    double dist = Math.hypot(c.getX() - frameCenterX, c.getY() - frameCenterY);
                    if (dist < minDistance) {
                        minDistance = dist;
                        closestCircle = c;
                        closestColor = "Purple";
                    }
                }

                boolean greenCloser = "Green".equals(closestColor);
                boolean purpleCloser = "Purple".equals(closestColor);

                // ---------------- Phone telemetry ----------------
                telemetry.addData("Green Balls", greenBlobs.size());
                telemetry.addData("Purple Balls", purpleBlobs.size());
                telemetry.addData("Green Closer", greenCloser);
                telemetry.addData("Purple Closer", purpleCloser);
                telemetry.addData("Closest Color", closestColor);

                telemetry.update();

                // ---------------- Dashboard telemetry ----------------
                TelemetryPacket packet = new TelemetryPacket();
    //            packet.put("Green Detected");
    //            packet.put("Purple Detected");
                packet.put("Green Balls", greenBlobs.size());
                packet.put("Purple Balls", purpleBlobs.size());
                packet.put("Green Closer", greenCloser);
                packet.put("Purple Closer", purpleCloser);
                packet.put("Closest Color", closestColor);
                packet.put("Show Radius", showRadius);

                dashboard.sendTelemetryPacket(packet);

                // Toggle radius display using gamepad A
                if (gamepad1.a) showRadius = !showRadius;

                sleep(50);
            }
        }
    }
