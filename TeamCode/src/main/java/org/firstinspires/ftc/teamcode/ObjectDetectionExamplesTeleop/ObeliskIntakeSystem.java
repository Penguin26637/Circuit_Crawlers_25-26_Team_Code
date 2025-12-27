package org.firstinspires.ftc.teamcode.ObjectDetectionExamplesTeleop;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * ObeliskIntakeSystem - Modular ball intake decision system
 *
 * This class can be instantiated in any OpMode (TeleOp or Autonomous) to provide
 * intelligent ball pickup decisions based on obelisk pattern detection.
 *
 * USAGE EXAMPLE:
 * <pre>
 * // In your OpMode's initialization:
 * ObeliskIntakeSystem intakeSystem = new ObeliskIntakeSystem(hardwareMap);
 *
 * // In your main loop:
 * intakeSystem.update();
 * if (intakeSystem.shouldPickup()) {
 *     // Run intake motors
 * }
 * </pre>
 */
@Config
public class ObeliskIntakeSystem {

    // ==================== CONFIGURATION PARAMETERS ====================

    // Hue thresholds for color sensors (editable via FTC Dashboard)
    public static float greenMinHue = 90;
    public static float greenMaxHue = 150;
    public static float purpleMinHue = 230;
    public static float purpleMaxHue = 300;
    public static int smoothingSamples = 5;

    // Ball detection thresholds
    public static int minContourArea = 200;
    public static int maxContourArea = 20000;
    public static double minCircularity = 0.5;

    // HuskyLens update rate
    public static int huskyReadPeriodSeconds = 1;
    public static boolean patternDetected = false;

    // Timing parameters
    public static double cooldownSeconds = 2.0;
    public static double intakeTimeoutMs = 1000;

    // ==================== HARDWARE COMPONENTS ====================
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime intaketime = new ElapsedTime();
    private HuskyLens huskyLens;
    private RevColorSensorV3 colorSensor1;
    private RevColorSensorV3 colorSensor2;
    private VisionPortal visionPortal;
    private ColorBlobLocatorProcessor greenLocator;
    private ColorBlobLocatorProcessor purpleLocator;
    private FtcDashboard dashboard;

    // Camera controls
    private ExposureControl exposureControl;
    private WhiteBalanceControl whiteBalanceControl;

    // ==================== STATE VARIABLES ====================

    // Obelisk pattern flags (only one should be true at a time)
    private boolean GPP = false;  // Green-Purple-Purple
    private boolean PGP = false;  // Purple-Green-Purple
    private boolean PPG = false;  // Purple-Purple-Green

    // Ball detection and decision
    public boolean pickup = false;
    private String obeliskPattern = "Unknown";
    private String detectedBallColor = "None";

    // Ball counter (tracks which ball in sequence we're expecting)
    public static int ball = 1;

    // Timing control
    private Deadline huskyRateLimit;
    private boolean intakeTimerActive = false;

    // Initialization flag
    private boolean initialized = false;

    // ==================== CONSTRUCTOR ====================

    /**
     * Creates a new ObeliskIntakeSystem
     *
     * @param hardwareMap The hardware map from your OpMode
     */
    public ObeliskIntakeSystem(HardwareMap hardwareMap) {
        try {
            // Initialize HuskyLens
            huskyLens = hardwareMap.get(HuskyLens.class, "hl");
            if (huskyLens.knock()) {
                huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
            }

            // Initialize Color Sensors
            colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "cs");
            colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "cs2");

            // Initialize Webcam Vision
            greenLocator = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setDrawContours(true)
                    .setCircleFitColor(Color.GREEN)
                    .setBlurSize(5)
                    .build();

            purpleLocator = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setDrawContours(true)
                    .setCircleFitColor(Color.MAGENTA)
                    .setBlurSize(5)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .addProcessor(greenLocator)
                    .addProcessor(purpleLocator)
                    .setCameraResolution(new Size(320, 240))
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .build();

            // Initialize Dashboard
            dashboard = FtcDashboard.getInstance();
            dashboard.startCameraStream(visionPortal, 30);

            // Initialize timing
            huskyRateLimit = new Deadline(huskyReadPeriodSeconds, TimeUnit.SECONDS);
            huskyRateLimit.expire();

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

        // Step 1: Detect Obelisk Pattern (periodically)
        if (huskyRateLimit.hasExpired()) {
            detectObeliskPattern();
            huskyRateLimit.reset();
        }

        // Step 2: Configure camera controls
        configureCameraControls();

        // Step 3: Detect incoming ball and determine pickup decision
        detectBallAndDecide();
    }

    // ==================== PUBLIC GETTER METHODS ====================

    /**
     * Returns whether the robot should pick up the current ball
     * @return true if ball should be picked up, false otherwise
     */
    public boolean shouldPickup() {
        return pickup;
    }

    /**
     * Returns the current obelisk pattern as a string
     * @return "GPP", "PGP", "PPG", "Unknown", or "Incomplete"
     */
    public String getObeliskPattern() {
        return obeliskPattern;
    }

    /**
     * Returns the detected ball color
     * @return "Green", "Purple", "None", or "Unknown"
     */
    public String getDetectedBallColor() {
        return detectedBallColor;
    }

    /**
     * Returns whether the system initialized successfully
     * @return true if all hardware initialized correctly
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Returns whether GPP pattern is active
     */
    public boolean isGPP() {
        return GPP;
    }

    /**
     * Returns whether PGP pattern is active
     */
    public boolean isPGP() {
        return PGP;
    }

    /**
     * Returns whether PPG pattern is active
     */
    public boolean isPPG() {
        return PPG;
    }

    // ==================== PUBLIC TELEMETRY METHOD ====================

    /**
     * Sends telemetry data to Driver Station and Dashboard
     * @param telemetry The telemetry object from your OpMode
     */
    public void sendTelemetry(Telemetry telemetry) {
        if (!initialized) {
            telemetry.addData("Intake System", "NOT INITIALIZED");
            return;
        }

        // Dashboard telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Obelisk Pattern", obeliskPattern);
        packet.put("GPP Active", GPP);
        packet.put("PGP Active", PGP);
        packet.put("PPG Active", PPG);
        packet.put("Ball Color", detectedBallColor);
        packet.put("PICKUP DECISION", pickup);
        packet.put("Ball Number", ball);
        dashboard.sendTelemetryPacket(packet);

        // Driver Station telemetry
        telemetry.addLine("=== INTAKE SYSTEM ===");
        telemetry.addData("Pattern", obeliskPattern);
        telemetry.addData("Ball Color", detectedBallColor);
        telemetry.addData("Ball #", ball);
        telemetry.addData("PICKUP", pickup ? "YES" : "NO");
    }

    /**
     * Sends detailed telemetry (more verbose)
     */
    public void sendDetailedTelemetry(Telemetry telemetry) {
        if (!initialized) {
            telemetry.addData("Intake System", "NOT INITIALIZED");
            return;
        }

        telemetry.addLine("=== OBELISK PATTERN ===");
        telemetry.addData("Pattern", obeliskPattern);
        telemetry.addData("GPP", GPP);
        telemetry.addData("PGP", PGP);
        telemetry.addData("PPG", PPG);
        telemetry.addLine();
        telemetry.addLine("=== BALL DETECTION ===");
        telemetry.addData("Detected Color", detectedBallColor);
        telemetry.addData("Expected Ball #", ball);
        telemetry.addLine();
        telemetry.addLine("=== DECISION ===");
        telemetry.addData("PICKUP", pickup ? "YES" : "NO");
        telemetry.addData("Intake Timer Active", intakeTimerActive);
    }

    // ==================== PUBLIC UTILITY METHODS ====================

    /**
     * Manually set the obelisk pattern (useful for testing)
     * @param pattern "GPP", "PGP", or "PPG"
     */
    public void setObeliskPattern(String pattern) {
        GPP = pattern.equals("GPP");
        PGP = pattern.equals("PGP");
        PPG = pattern.equals("PPG");
        obeliskPattern = pattern;
    }

    /**
     * Force an immediate obelisk pattern update (bypasses rate limiting)
     */
    public void forceObeliskUpdate() {
        detectObeliskPattern();
    }

    /**
     * Reset the ball counter to 1
     */
    public void resetBallCounter() {
        ball = 1;
    }

    /**
     * Cleanup method - call when OpMode stops
     */
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // ==================== PRIVATE DETECTION METHODS ====================

    private void detectObeliskPattern() {
        if (!patternDetected) {
            // Reset all pattern flags
            GPP = false;
            PGP = false;
            PPG = false;
            obeliskPattern = "Unknown";
        }

        List<HuskyLens.Block> blocks = Arrays.asList(huskyLens.blocks());

        // Check each detected block
        for (HuskyLens.Block block : blocks) {
            if (block.id == 1) {
                setObeliskPattern("GPP");
                patternDetected = true;
                break;
            } else if (block.id == 2) {
                setObeliskPattern("PGP");
                patternDetected = true;
                break;
            } else if (block.id == 3) {
                setObeliskPattern("PPG");
                patternDetected = true;
                break;
            }
        }

        // Update pattern flags
        if (obeliskPattern.equals("GPP")) {
            GPP = true;
        } else if (obeliskPattern.equals("PGP")) {
            PGP = true;
        } else if (obeliskPattern.equals("PPG")) {
            PPG = true;
        }
    }

    private void configureCameraControls() {
        try {
            exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

            if (exposureControl != null) {
                exposureControl.setExposure(1, TimeUnit.MILLISECONDS);
            }
            if (whiteBalanceControl != null) {
                whiteBalanceControl.setWhiteBalanceTemperature(6500);
            }
        } catch (Exception e) {
            // Camera controls may not be ready yet
        }
    }

    private void detectBallAndDecide() {
        // Handle intake timeout
        if (intakeTimerActive && intaketime.milliseconds() > intakeTimeoutMs) {
            pickup = false;
            intakeTimerActive = false;
        }

        // If timer is still active, maintain pickup state and continue
        if (intakeTimerActive) {
            pickup = true;
            return;
        }

        // Reset pickup state
        pickup = false;

        // Get ball detections from webcam
        List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenLocator.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();

        // Filter blobs by area
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                minContourArea, maxContourArea, greenBlobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                minContourArea, maxContourArea, purpleBlobs);

        // Filter blobs by circularity
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                minCircularity, 1.0, greenBlobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                minCircularity, 1.0, purpleBlobs);

        // Find closest ball to center of frame
        double frameCenterX = 320 / 2.0;
        double frameCenterY = 240 / 2.0;
        Circle closestCircle = null;
        String webcamColor = "None";
        double minDistance = Double.MAX_VALUE;

        for (ColorBlobLocatorProcessor.Blob b : greenBlobs) {
            Circle c = b.getCircle();
            double dist = Math.hypot(c.getX() - frameCenterX, c.getY() - frameCenterY);
            if (dist < minDistance) {
                minDistance = dist;
                closestCircle = c;
                webcamColor = "Green";
            }
        }

        for (ColorBlobLocatorProcessor.Blob b : purpleBlobs) {
            Circle c = b.getCircle();
            double dist = Math.hypot(c.getX() - frameCenterX, c.getY() - frameCenterY);
            if (dist < minDistance) {
                minDistance = dist;
                closestCircle = c;
                webcamColor = "Purple";
            }
        }

        // If no ball detected, return early
        if (closestCircle == null) {
            detectedBallColor = "None";
            pickup = false;
            return;
        }

        // Double-check with color sensors
        String colorSensorResult = getColorFromSensors();

        // Consensus-based decision
        if (webcamColor.equals(colorSensorResult) && !webcamColor.equals("None")) {
            detectedBallColor = webcamColor;
        } else {
            // Fallback to webcam if sensors disagree
            detectedBallColor = webcamColor;
        }

        // Decide whether to pickup based on obelisk pattern
        boolean shouldPickupThisBall = shouldPickupBasedOnPattern(detectedBallColor);

        // Start intake timer if pickup decision is true
        if (shouldPickupThisBall) {
            pickup = true;
            intaketime.reset();
            intakeTimerActive = true;

            // Increment ball counter only when starting new intake
            incrementBallCounter(detectedBallColor);
        }
    }

    private String getColorFromSensors() {
        // Get smoothed hue readings from both sensors
        float cs1Hue = getAverageHue(colorSensor1, smoothingSamples);
        float cs2Hue = getAverageHue(colorSensor2, smoothingSamples);

        // Average between both sensors
        float avgHue = (cs1Hue + cs2Hue) / 2.0f;

        // Determine color
        if (avgHue >= greenMinHue && avgHue <= greenMaxHue) {
            return "Green";
        } else if (avgHue >= purpleMinHue && avgHue <= purpleMaxHue) {
            return "Purple";
        }

        return "Unknown";
    }

    private float getAverageHue(RevColorSensorV3 sensor, int samples) {
        float sumHue = 0;
        for (int i = 0; i < samples; i++) {
            float r = sensor.red();
            float g = sensor.green();
            float b = sensor.blue();
            int colorInt = Color.rgb((int) r, (int) g, (int) b);
            float[] hsv = new float[3];
            Color.colorToHSV(colorInt, hsv);
            sumHue += hsv[0];
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        return sumHue / samples;
    }

    private boolean shouldPickupBasedOnPattern(String ballColor) {
        // Check if ball color is valid
        if (ballColor.equals("None") || ballColor.equals("Unknown")) {
            return false;
        }

        // Check cooldown period
        if (runtime.seconds() < cooldownSeconds) {
            return false;
        }

        // Decision logic based on obelisk pattern
        if (GPP) {
            // Green-Purple-Purple sequence
            if (ballColor.equals("Green") && ball == 1) {
                return true;
            } else if (ballColor.equals("Purple") && ball == 2) {
                return true;
            } else if (ballColor.equals("Purple") && ball == 3) {
                return true;
            }
        } else if (PGP) {
            // Purple-Green-Purple sequence
            if (ballColor.equals("Purple") && ball == 1) {
                return true;
            } else if (ballColor.equals("Green") && ball == 2) {
                return true;
            } else if (ballColor.equals("Purple") && ball == 3) {
                return true;
            }
        } else if (PPG) {
            // Purple-Purple-Green sequence
            if (ballColor.equals("Purple") && ball == 1) {
                return true;
            } else if (ballColor.equals("Purple") && ball == 2) {
                return true;
            } else if (ballColor.equals("Green") && ball == 3) {
                return true;
            }
        }

        // If pattern unknown or ball doesn't match expected sequence, don't pick up
        return false;
    }

    /**
     * Increments the ball counter based on current pattern and detected color
     */
    private void incrementBallCounter(String ballColor) {
        // Reset cooldown timer when incrementing ball counter
        runtime.reset();

        // Increment ball counter based on current state
        if (ball == 1) {
            ball = 2;
        } else if (ball == 2) {
            ball = 3;
        } else if (ball == 3) {
            ball = 1;
        }
    }
}