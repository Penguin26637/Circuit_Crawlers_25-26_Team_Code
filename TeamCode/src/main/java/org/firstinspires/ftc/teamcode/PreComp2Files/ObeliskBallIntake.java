package org.firstinspires.ftc.teamcode.PreComp2Files;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "HL_WC_CS Integration", group = "Testing")
@Config
public class ObeliskBallIntake extends LinearOpMode {

    // ==================== CONFIGURATION PARAMETERS ====================
    
    // Hue thresholds for color sensors
    public static float greenMinHue = 90;
    public static float greenMaxHue = 150;
    public static float purpleMinHue = 230;
    public static float purpleMaxHue = 300;
    public static int smoothingSamples = 5;
    
    // Ball detection thresholds
    public static int minContourArea = 200;
    public static int maxContourArea = 20000;
    public static double minCircularity = 0.5;
    
    // ==================== HARDWARE COMPONENTS ====================
    
    private HuskyLens huskyLens;
    private RevColorSensorV3 colorSensor1;
    private RevColorSensorV3 colorSensor2;
    private VisionPortal visionPortal;
    private ColorBlobLocatorProcessor greenLocator;
    private ColorBlobLocatorProcessor purpleLocator;
    private FtcDashboard dashboard;
    
    // Camera controls
    private ExposureControl exposureControl;
    private GainControl gainControl;
    private WhiteBalanceControl whiteBalanceControl;
    
    // ==================== STATE VARIABLES ====================
    
    // Obelisk order flags (only one should be true at a time)
    private boolean GPP = false;  // Green-Purple-Purple
    private boolean PGP = false;  // Purple-Green-Purple
    private boolean PPG = false;  // Purple-Purple-Green
    
    // Ball detection and decision
    private boolean pickup = false;
    private String obeliskPattern = "Unknown";
    private String detectedBallColor = "None";
    
    private static final int HUSKY_READ_PERIOD = 1; // seconds
    
    @Override
    public void runOpMode() {
        
        // ==================== INITIALIZATION ====================
        
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();
        
        // Initialize HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "hl");
        if (!huskyLens.knock()) {
            telemetry.addData("ERROR", "HuskyLens not responding!");
            telemetry.update();
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        
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
        
        telemetry.setMsTransmissionInterval(100);
        telemetry.addData("Status", "Ready to start!");
        telemetry.update();
        
        waitForStart();
        
        Deadline huskyRateLimit = new Deadline(HUSKY_READ_PERIOD, TimeUnit.SECONDS);
        huskyRateLimit.expire();
        
        // ==================== MAIN LOOP ====================
        
        while (opModeIsActive()) {
            
            // Step 1: Detect Obelisk Pattern (periodically)
//            if (huskyRateLimit.hasExpired()) {
//                detectObeliskPattern();
//                huskyRateLimit.reset();
//            }
            
            // Step 2: Configure camera controls
            configureCameraControls();
            
            // Step 3: Detect incoming ball and determine pickup decision
            detectBallAndDecide();
            
            // Step 4: Send telemetry
            sendTelemetry();
            
            sleep(50);
        }
    }
    
    // ==================== OBELISK PATTERN DETECTION ====================
    
    private void detectObeliskPattern() {
        // Reset all pattern flags
        GPP = false;
        PGP = false;
        PPG = false;
        obeliskPattern = "Unknown";
        
        HuskyLens.Block[] blocks = huskyLens.blocks();
        
        if (blocks.length < 3) {
            obeliskPattern = "Incomplete (need 3 blocks)";
            return;
        }
        
        // Sort blocks by X position (left to right)
        java.util.Arrays.sort(blocks, (a, b) -> Integer.compare(a.x, b.x));
        
        // Determine pattern based on IDs
        // Assuming ID 1 = Green, ID 2 = Purple (adjust based on your setup)
        int first = blocks[0].id;
        int second = blocks[1].id;
        int third = blocks[2].id;
        
        // Map IDs to colors (customize based on your HuskyLens configuration)
        // For this example: 1=Green, 2=Purple
        String pos1 = (first == 1) ? "G" : "P";
        String pos2 = (second == 1) ? "G" : "P";
        String pos3 = (third == 1) ? "G" : "P";
        
        obeliskPattern = pos1 + pos2 + pos3;
        
        // Set the appropriate flag
        if (obeliskPattern.equals("GPP")) {
            GPP = true;
        } else if (obeliskPattern.equals("PGP")) {
            PGP = true;
        } else if (obeliskPattern.equals("PPG")) {
            PPG = true;
        }
    }
    
    // ==================== CAMERA CONFIGURATION ====================
    
    private void configureCameraControls() {
        try {
            exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            gainControl = visionPortal.getCameraControl(GainControl.class);
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
    
    // ==================== BALL DETECTION AND DECISION ====================
    
    private void detectBallAndDecide() {
        pickup = false;
        
        // Get ball detections from webcam
        List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenLocator.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();
        
        // Filter blobs
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 
                minContourArea, maxContourArea, greenBlobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 
                minContourArea, maxContourArea, purpleBlobs);
        
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 
                minCircularity, 1.0, greenBlobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 
                minCircularity, 1.0, purpleBlobs);
        
        // Find closest ball
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
        
        // Consensus-based decision (require webcam AND color sensors to agree)
        if (webcamColor.equals(colorSensorResult) && !webcamColor.equals("None")) {
            detectedBallColor = webcamColor;
        } else {
            // Fallback to webcam if sensors disagree
            detectedBallColor = webcamColor;
        }
        
        // Decide whether to pickup based on obelisk pattern
        pickup = shouldPickup(detectedBallColor);
    }
    
    // ==================== COLOR SENSOR READING ====================
    
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
        }
        return sumHue / samples;
    }
    
    // ==================== PICKUP DECISION LOGIC ====================
    
    private boolean shouldPickup(String ballColor) {
        if (ballColor.equals("None") || ballColor.equals("Unknown")) {
            return false;
        }
        
        // Decision logic based on obelisk pattern
        if (GPP) {
            // Green-Purple-Purple: pick up Green balls
            return ballColor.equals("Green");
        } else if (PGP) {
            // Purple-Green-Purple: pick up Purple balls
            return ballColor.equals("Purple");
        } else if (PPG) {
            // Purple-Purple-Green: pick up Purple balls
            return ballColor.equals("Purple");
        }
        
        // If pattern unknown, don't pick up
        return false;
    }
    
    // ==================== TELEMETRY ====================
    
    private void sendTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        
        // Obelisk pattern
        packet.put("Obelisk Pattern", obeliskPattern);
        packet.put("GPP Active", GPP);
        packet.put("PGP Active", PGP);
        packet.put("PPG Active", PPG);
        
        // Ball detection
        packet.put("Ball Color", detectedBallColor);
        packet.put("PICKUP DECISION", pickup);
        
        // Send to dashboard
        dashboard.sendTelemetryPacket(packet);
        
        // Driver station telemetry
        telemetry.addLine("=== OBELISK PATTERN ===");
        telemetry.addData("Pattern", obeliskPattern);
        telemetry.addData("GPP", GPP);
        telemetry.addData("PGP", PGP);
        telemetry.addData("PPG", PPG);
        telemetry.addLine();
        telemetry.addLine("=== BALL DETECTION ===");
        telemetry.addData("Detected Color", detectedBallColor);
        telemetry.addLine();
        telemetry.addLine("=== DECISION ===");
        telemetry.addData("PICKUP", pickup ? "YES" : "NO");
        telemetry.update();
    }
    
    // ==================== PUBLIC ACCESSOR METHODS ====================
    
    /**
     * Returns whether the robot should pick up the current ball
     */
    public boolean shouldPickup() {
        return pickup;
    }
    
    /**
     * Returns the current obelisk pattern
     */
    public String getObeliskPattern() {
        return obeliskPattern;
    }
    
    /**
     * Returns the detected ball color
     */
    public String getDetectedBallColor() {
        return detectedBallColor;
    }
}