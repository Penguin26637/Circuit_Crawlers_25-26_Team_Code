package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * RedOuttakeDetectionSystem - Detects red objects and signals flipper activation
 *
 * This class can be instantiated in any OpMode (TeleOp or Autonomous) to provide
 * intelligent red object detection for outtake operations.
 *
 * REGION OF INTEREST (ROI) CONFIGURATION:
 * The ROI parameters restrict detection to a specific area of the camera view.
 * All values are percentages from 0.0 to 1.0:
 * - roiLeftPercent = 0.0 (left edge) to 1.0 (right edge)
 * - roiRightPercent = 0.0 (left edge) to 1.0 (right edge)
 * - roiTopPercent = 0.0 (top edge) to 1.0 (bottom edge)
 * - roiBottomPercent = 0.0 (top edge) to 1.0 (bottom edge)
 *
 * EXAMPLE ROI CONFIGURATIONS:
 * - Full frame: Left=0.0, Top=0.0, Right=1.0, Bottom=1.0
 * - Bottom half only: Left=0.0, Top=0.5, Right=1.0, Bottom=1.0
 * - Center square: Left=0.25, Top=0.25, Right=0.75, Bottom=0.75
 * - Left side only: Left=0.0, Top=0.0, Right=0.5, Bottom=1.0
 *
 * These can be adjusted via FTC Dashboard in real-time!
 *
 * USAGE EXAMPLE:
 * <pre>
 * // In your OpMode's initialization:
 * RedOuttakeDetectionSystem outtakeSystem = new RedOuttakeDetectionSystem(hardwareMap);
 *
 * // In your main loop:
 * outtakeSystem.update();
 * if (outtakeSystem.shouldFlipUp()) {
 *     // Activate flipper to up position
 *     flipper.goToPosition(0.2);
 * } else {
 *     // Keep flipper down
 *     flipper.goToPosition(0);
 * }
 * </pre>
 */
@Config
public class RedOuttakeDetectionSystem {

    // ==================== CONFIGURATION PARAMETERS ====================

    // Camera settings
    public static double cameraZoom = 1.0;  // 1.0 = fully zoomed out (minimum zoom)

    // Region of Interest (ROI) settings - as percentage of frame
    // 0.0 = left/top edge, 1.0 = right/bottom edge
    public static double roiLeftPercent = 0.0;      // Left boundary (0-1)
    public static double roiTopPercent = 0.0;       // Top boundary (0-1)
    public static double roiRightPercent = 1.0;     // Right boundary (0-1)
    public static double roiBottomPercent = 1.0;    // Bottom boundary (0-1)

    // Object detection thresholds
    public static int minContourArea = 200;
    public static int maxContourArea = 20000;
    public static double minCircularity = 0.5;

    // Timing parameters
    public static double detectionCooldownSeconds = 0.5;
    public static double flipUpDurationMs = 1000;  // How long to keep flipper up after detection

    // ==================== HARDWARE COMPONENTS ====================
    private ElapsedTime cooldownTimer = new ElapsedTime();
    private ElapsedTime flipTimer = new ElapsedTime();
    private VisionPortal visionPortal;
    private ColorBlobLocatorProcessor redLocator;
    private FtcDashboard dashboard;

    // Camera controls
    private ExposureControl exposureControl;
    private WhiteBalanceControl whiteBalanceControl;
    private PtzControl ptzControl;

    // ==================== STATE VARIABLES ====================

    public boolean shouldFlipUp = false;
    private String detectedObjectColor = "None";
    private boolean flipTimerActive = false;
    private boolean redDetected = false;

    // Initialization flag
    private boolean initialized = false;

    // ==================== CONSTRUCTOR ====================

    /**
     * Creates a new RedOuttakeDetectionSystem
     *
     * @param hardwareMap The hardware map from your OpMode
     */
    public RedOuttakeDetectionSystem(HardwareMap hardwareMap) {
        try {
            // Initialize Webcam Vision for RED detection
            redLocator = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.RED)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setDrawContours(true)
                    .setCircleFitColor(Color.RED)
                    .setBlurSize(5)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .addProcessor(redLocator)
                    .setCameraResolution(new Size(640, 480))
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .build();

            // Initialize Dashboard
            dashboard = FtcDashboard.getInstance();

            // Start camera stream to dashboard
            dashboard.startCameraStream(visionPortal, 0);

            // Start cooldown timer
            cooldownTimer.reset();

            initialized = true;

        } catch (Exception e) {
            initialized = false;
            // System will report not initialized
        }
    }

    // ==================== PUBLIC UPDATE METHOD ====================

    /**
     * Updates the system - call this once per loop iteration
     * This method performs all detection and decision logic
     */
    public void update() {
        if (!initialized) return;

        // Step 1: Configure camera controls
        configureCameraControls();

        // Step 2: Detect red object and determine flip decision
        detectRedObjectAndDecide();
    }

    // ==================== PUBLIC GETTER METHODS ====================

    /**
     * Returns whether the flipper should be in the up position
     * @return true if flipper should be up, false otherwise
     */
    public boolean shouldFlipUp() {
        return shouldFlipUp;
    }

    /**
     * Returns the detected object color
     * @return "Red", "None", or "Unknown"
     */
    public String getDetectedObjectColor() {
        return detectedObjectColor;
    }

    /**
     * Returns whether red was detected
     * @return true if red object is detected
     */
    public boolean isRedDetected() {
        return redDetected;
    }

    /**
     * Returns whether the system initialized successfully
     * @return true if all hardware initialized correctly
     */
    public boolean isInitialized() {
        return initialized;
    }

    // ==================== PUBLIC TELEMETRY METHOD ====================

    /**
     * Sends telemetry data to Driver Station and Dashboard
     * @param telemetry The telemetry object from your OpMode
     */
    public void sendTelemetry(Telemetry telemetry) {
        if (!initialized) {
            telemetry.addData("Outtake System", "NOT INITIALIZED");
            return;
        }

        // Dashboard telemetry with detection info
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Red Detected", redDetected);
        packet.put("Object Color", detectedObjectColor);
        packet.put("FLIP UP", shouldFlipUp);
        packet.put("Flip Timer Active", flipTimerActive);

        // Add detection circle info
        List<ColorBlobLocatorProcessor.Blob> redBlobs = redLocator.getBlobs();
        packet.put("Red Blobs Detected", redBlobs.size());

        if (!redBlobs.isEmpty()) {
            ColorBlobLocatorProcessor.Blob largest = redBlobs.get(0);
            packet.put("Largest Blob Area", largest.getContourArea());
            packet.put("Largest Blob X", largest.getBoxFit().center.x);
            packet.put("Largest Blob Y", largest.getBoxFit().center.y);
        }

        dashboard.sendTelemetryPacket(packet);

        // Driver Station telemetry
        telemetry.addLine("=== OUTTAKE SYSTEM ===");
        telemetry.addData("Red Detected", redDetected ? "YES" : "NO");
        telemetry.addData("Object Color", detectedObjectColor);
        telemetry.addData("FLIP UP", shouldFlipUp ? "YES" : "NO");
        telemetry.addData("Blobs Detected", redBlobs.size());
    }

    /**
     * Sends detailed telemetry (more verbose)
     */
    public void sendDetailedTelemetry(Telemetry telemetry) {
        if (!initialized) {
            telemetry.addData("Outtake System", "NOT INITIALIZED");
            return;
        }

        List<ColorBlobLocatorProcessor.Blob> redBlobs = redLocator.getBlobs();

        telemetry.addLine("=== RED DETECTION ===");
        telemetry.addData("Red Detected", redDetected);
        telemetry.addData("Detected Color", detectedObjectColor);
        telemetry.addData("Blobs Found", redBlobs.size());
        telemetry.addLine();
        telemetry.addLine("=== DECISION ===");
        telemetry.addData("FLIP UP", shouldFlipUp ? "YES" : "NO");
        telemetry.addData("Flip Timer Active", flipTimerActive);
        telemetry.addData("Cooldown (s)", String.format("%.2f", cooldownTimer.seconds()));
        telemetry.addLine();
        telemetry.addLine("=== CAMERA ===");
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("FPS", visionPortal.getFps());
        telemetry.addLine();
        telemetry.addLine("=== ROI (Region of Interest) ===");
        telemetry.addData("Left", String.format("%.0f%%", roiLeftPercent * 100));
        telemetry.addData("Top", String.format("%.0f%%", roiTopPercent * 100));
        telemetry.addData("Right", String.format("%.0f%%", roiRightPercent * 100));
        telemetry.addData("Bottom", String.format("%.0f%%", roiBottomPercent * 100));
    }

    // ==================== PUBLIC UTILITY METHODS ====================

    /**
     * Force reset the flip state
     */
    public void resetFlipState() {
        shouldFlipUp = false;
        flipTimerActive = false;
        cooldownTimer.reset();
    }

    /**
     * Cleanup method - call when OpMode stops
     */
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Start/restart the camera stream to dashboard
     */
    public void startCameraStream() {
        if (dashboard != null && visionPortal != null) {
            dashboard.startCameraStream(visionPortal, 0);
        }
    }

    /**
     * Stop the camera stream
     */
    public void stopCameraStream() {
        if (dashboard != null) {
            dashboard.stopCameraStream();
        }
    }

    // ==================== PRIVATE DETECTION METHODS ====================

    private void configureCameraControls() {
        try {
            exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
            ptzControl = visionPortal.getCameraControl(PtzControl.class);

            if (exposureControl != null) {
                exposureControl.setExposure(1, TimeUnit.MILLISECONDS);
            }
            if (whiteBalanceControl != null) {
                whiteBalanceControl.setWhiteBalanceTemperature(6500);
            }
            if (ptzControl != null) {
                // Set zoom to minimum (fully zoomed out)
                int minZoom = ptzControl.getMinZoom();
                ptzControl.setZoom(minZoom);
            }
        } catch (Exception e) {
            // Camera controls may not be ready yet
        }
    }

    private void detectRedObjectAndDecide() {
        // Handle flip timer duration
        if (flipTimerActive && flipTimer.milliseconds() > flipUpDurationMs) {
            shouldFlipUp = false;
            flipTimerActive = false;
            cooldownTimer.reset();
        }

        // If timer is still active, maintain flip state and continue
        if (flipTimerActive) {
            shouldFlipUp = true;
            return;
        }

        // Reset flip state
        shouldFlipUp = false;
        redDetected = false;

        // Check cooldown period
        if (cooldownTimer.seconds() < detectionCooldownSeconds) {
            return;
        }

        // Get red detections from webcam
        List<ColorBlobLocatorProcessor.Blob> redBlobs = redLocator.getBlobs();

        // Filter blobs by area
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                minContourArea, maxContourArea, redBlobs);

        // Filter blobs by circularity
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                minCircularity, 1.0, redBlobs);

        // Calculate ROI boundaries in pixels
        int frameWidth = 640;
        int frameHeight = 480;
        int roiLeft = (int) (roiLeftPercent * frameWidth);
        int roiTop = (int) (roiTopPercent * frameHeight);
        int roiRight = (int) (roiRightPercent * frameWidth);
        int roiBottom = (int) (roiBottomPercent * frameHeight);

        // Filter blobs by ROI (only keep blobs inside the region)
        redBlobs.removeIf(blob -> {
            double blobX = blob.getBoxFit().center.x;
            double blobY = blob.getBoxFit().center.y;
            return blobX < roiLeft || blobX > roiRight ||
                    blobY < roiTop || blobY > roiBottom;
        });

        // Find closest red object to center of ROI
        double roiCenterX = (roiLeft + roiRight) / 2.0;
        double roiCenterY = (roiTop + roiBottom) / 2.0;
        Circle closestCircle = null;
        String webcamColor = "None";
        double minDistance = Double.MAX_VALUE;

        for (ColorBlobLocatorProcessor.Blob b : redBlobs) {
            Circle c = b.getCircle();
            double dist = Math.hypot(c.getX() - roiCenterX, c.getY() - roiCenterY);
            if (dist < minDistance) {
                minDistance = dist;
                closestCircle = c;
                webcamColor = "Red";
            }
        }

        // If no red object detected, return early
        if (closestCircle == null) {
            detectedObjectColor = "None";
            shouldFlipUp = false;
            return;
        }

        // Red detected by webcam
        detectedObjectColor = "Red";
        redDetected = true;

        // Activate flip if red is detected
        if (redDetected) {
            shouldFlipUp = true;
            flipTimer.reset();
            flipTimerActive = true;
        }
    }
}