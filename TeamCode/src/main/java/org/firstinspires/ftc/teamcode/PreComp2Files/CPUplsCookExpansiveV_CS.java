package org.firstinspires.ftc.teamcode.PreComp2Files;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.Circle;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="CPUplsCook_VisionAndColorSensors", group="TELEOP")
@Config
@Disabled
public class CPUplsCookExpansiveV_CS extends LinearOpMode {

    // --- Gamepad 1 drive motors ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- Wheel brake ---
    public static boolean wheelBreak = false;
    public static int wheelBreakTargetFL, wheelBreakTargetFR, wheelBreakTargetBL, wheelBreakTargetBR;
    public static double wheelBreak_kP = 0.01;
    public static double wheelBreak_maxPower = 0.2;
    public static int wheelBreak_maxError = 100;

    // --- Odometry encoders (these motors are ONLY used for odometry, no power should be applied) ---
    private DcMotor intake, intake2;  // Odometry pods - left and right
    private DcMotor shooter;  // Odometry pod - back

    // --- Functional motors/servos ---
    private DcMotorEx flywheel;
    private CRServo intakeToShooter, intakeToShooter2;

    public static boolean intakeActive = false;
    public static boolean shooterActive = false;
    public static boolean shooterUp = false;

    public static double intake_speed = 0.5;
    public static double shooter_power = 1;
    public static double intakeToShooter_power = 0.5;

    // --- Odometry constants ---
    public static double TICKS_PER_INCH = 337.2;
    public static double TRACK_WIDTH = 13.5;
    public static double BACK_WHEEL_OFFSET = 8;

    private double xPos = 0, yPos = 0, heading = 0;
    private int prevLeft = 0, prevRight = 0, prevBack = 0;

    // --- Runtime ---
    private final ElapsedTime runtime = new ElapsedTime();

    // --- Dashboard ---
    private FtcDashboard dashboard;

    public static boolean slow_mode = false;
    public static boolean robot_centric = true;
    public static boolean field_centric = false;
    public static double kP = 0.01;
    public static double maxPower = 0.2;
    public static int maxError = 100;

    public static final double TICKS_PER_REV = 112.0;
    public static double MAX_RPM_UNDER_LOAD = 1400.0;

    // ============= Controller Gains =============
    public static double kV = 0.0006785714285714286;
    public static double kS = 0.06;
    public static double kP_F = 0.0004;
    public static double kI = 0.0002;
    public static double kD = 0.00005;
    public static double integralLimit = 0.2;

    // ============= Internal State ============
    private ElapsedTime dtTimer = new ElapsedTime();
    private double lastPosition = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;

    public static double targetRPM = 980.0;
    public static double currentRPM;
    public static double error;
    public static double output;
    public static double ff;
    public static double pid;

    // ============= Button State Tracking (for debouncing) =============
    private boolean prevRightBumper = false;
    private boolean prevLeftStickButton = false;
    private boolean prevRightStickButton = false;

    // ============= VISION SYSTEM =============
    private VisionPortal visionPortal;
    private ColorBlobLocatorProcessor greenLocator;
    private ColorBlobLocatorProcessor purpleLocator;

    // Vision camera controls
    public ExposureControl myExposureControl;
    public FocusControl myFocusControl;
    public PtzControl myPtzControl;
    public GainControl myGainControl;
    public WhiteBalanceControl myWhiteBalanceControl;

    // Vision data (volatile for thread safety)
    private volatile int greenBallCount = 0;
    private volatile int purpleBallCount = 0;
    private volatile String closestColor = "None";
    private volatile double closestBallX = 0;
    private volatile double closestBallY = 0;

    // Vision configuration
    public static boolean showRadius = true;
    public static int CAMERA_WIDTH = 320;
    public static int CAMERA_HEIGHT = 240;

    // ============= COLOR SENSORS =============
    private RevColorSensorV3 cs1;
    private RevColorSensorV3 cs2;

    // Color sensor configuration (editable on dashboard)
    public static float greenMinHue = 90;
    public static float greenMaxHue = 150;
    public static float purpleMinHue = 230;
    public static float purpleMaxHue = 300;
    public static int smoothingSamples = 5;

    // Color sensor data (volatile for thread safety)
    private volatile float cs1Hue = 0;
    private volatile float cs2Hue = 0;
    private volatile float avgHue = 0;
    private volatile String detectedColor = "Unknown";

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) result = Math.min(result, voltage);
        }
        return result;
    }

    private void updateOdometry() {
        // --- Read encoder values ---
        int leftPos = -1 * (intake.getCurrentPosition());
        int rightPos = (intake2.getCurrentPosition());
        int backPos = (shooter.getCurrentPosition());

        int deltaLeft = leftPos - prevLeft;
        int deltaRight = rightPos - prevRight;
        int deltaBack = backPos - prevBack;

        prevLeft = leftPos;
        prevRight = rightPos;
        prevBack = backPos;

        // --- Convert ticks to inches ---
        double dLeft = deltaLeft / TICKS_PER_INCH;
        double dRight = deltaRight / TICKS_PER_INCH;
        double dBack = deltaBack / TICKS_PER_INCH;

        // --- Odometry math ---
        double dHeading = (dRight - dLeft) / TRACK_WIDTH;
        double dForward = (dLeft + dRight) / 2.0;
        double dSide = dBack - (dHeading * BACK_WHEEL_OFFSET);

        // Update global position
        double sinHeading = Math.sin(heading);
        double cosHeading = Math.cos(heading);

        xPos += dForward * cosHeading - dSide * sinHeading;
        yPos += dForward * sinHeading + dSide * cosHeading;
        heading += dHeading;
    }

    private void initializeVision() {
        // Green detector
        greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setCircleFitColor(Color.GREEN)
                .setBlurSize(5)
                .build();

        // Purple detector
        purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setCircleFitColor(Color.MAGENTA)
                .setBlurSize(5)
                .build();

        // Create vision portal
        visionPortal = new VisionPortal.Builder()
                .addProcessor(greenLocator)
                .addProcessor(purpleLocator)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        // Start camera stream on dashboard
        dashboard.startCameraStream(visionPortal, 30);
    }

    private void updateVisionData() {
        // Get camera controls and configure
        try {
            if (myExposureControl == null) {
                myExposureControl = visionPortal.getCameraControl(ExposureControl.class);
            }
            if (myFocusControl == null) {
                myFocusControl = visionPortal.getCameraControl(FocusControl.class);
            }
            if (myGainControl == null) {
                myGainControl = visionPortal.getCameraControl(GainControl.class);
            }
            if (myPtzControl == null) {
                myPtzControl = visionPortal.getCameraControl(PtzControl.class);
            }
            if (myWhiteBalanceControl == null) {
                myWhiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
            }

            if (myExposureControl != null && myExposureControl.getMode() != ExposureControl.Mode.Manual) {
                myExposureControl.setMode(ExposureControl.Mode.Manual);
                myExposureControl.setExposure(1, TimeUnit.MILLISECONDS);
            }
            if (myWhiteBalanceControl != null && myWhiteBalanceControl.getMode() != WhiteBalanceControl.Mode.MANUAL) {
                myWhiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
                myWhiteBalanceControl.setWhiteBalanceTemperature(6500);
            }
        } catch (Exception e) {
            // Camera controls may not be available immediately
            telemetry.addData("Vision Warning", "Camera controls not ready");
        }

        // Get detected blobs
        List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenLocator.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();

        // Filter blobs
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 200, 20000, greenBlobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 200, 20000, purpleBlobs);

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.5, 1.0, greenBlobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.5, 1.0, purpleBlobs);

        // Update counts
        greenBallCount = greenBlobs.size();
        purpleBallCount = purpleBlobs.size();

        // Find closest ball to center
        double frameCenterX = CAMERA_WIDTH / 2.0;
        double frameCenterY = CAMERA_HEIGHT / 2.0;
        Circle closestCircle = null;
        String tempClosestColor = "None";
        double minDistance = Double.MAX_VALUE;

        for (ColorBlobLocatorProcessor.Blob b : greenBlobs) {
            Circle c = b.getCircle();
            if (c != null) {
                double dist = Math.hypot(c.getX() - frameCenterX, c.getY() - frameCenterY);
                if (dist < minDistance) {
                    minDistance = dist;
                    closestCircle = c;
                    tempClosestColor = "Green";
                }
            }
        }

        for (ColorBlobLocatorProcessor.Blob b : purpleBlobs) {
            Circle c = b.getCircle();
            if (c != null) {
                double dist = Math.hypot(c.getX() - frameCenterX, c.getY() - frameCenterY);
                if (dist < minDistance) {
                    minDistance = dist;
                    closestCircle = c;
                    tempClosestColor = "Purple";
                }
            }
        }

        // Update closest ball data
        closestColor = tempClosestColor;

        if (closestCircle != null) {
            closestBallX = closestCircle.getX();
            closestBallY = closestCircle.getY();
        } else {
            closestBallX = 0;
            closestBallY = 0;
        }
    }

    // Returns average hue over multiple readings for smoothing
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
            sleep(2); // short delay between readings
        }
        return sumHue / samples;
    }

    private void updateColorSensors() {
        // Get smoothed hue readings from both sensors
        cs1Hue = getAverageHue(cs1, smoothingSamples);
        cs2Hue = getAverageHue(cs2, smoothingSamples);

        // Average between both sensors
        avgHue = (cs1Hue + cs2Hue) / 2.0f;

        // Robust color detection
        if (avgHue >= greenMinHue && avgHue <= greenMaxHue) {
            detectedColor = "Green";
        } else if (avgHue >= purpleMinHue && avgHue <= purpleMaxHue) {
            detectedColor = "Purple";
        } else {
            detectedColor = "Unknown";
        }
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize dashboard first
        dashboard = FtcDashboard.getInstance();

        // --- Hardware Mapping ---
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backl");
        backRightDrive = hardwareMap.get(DcMotor.class, "backr");

        // Odometry pods (these motors are used ONLY for encoder readings)
        intake = hardwareMap.get(DcMotor.class, "i");
        intake2 = hardwareMap.get(DcMotor.class, "i2");
        shooter = hardwareMap.get(DcMotor.class, "s");

        // Functional motor - flywheel mapped to same port as shooter odometry
        flywheel = hardwareMap.get(DcMotorEx.class, "s");

        // Servos
        intakeToShooter = hardwareMap.get(CRServo.class, "its");
        intakeToShooter2 = hardwareMap.get(CRServo.class, "its2");

        // Color sensors
        cs1 = hardwareMap.get(RevColorSensorV3.class, "cs");
        cs2 = hardwareMap.get(RevColorSensorV3.class, "cs2");

        // --- Odometry encoder setup ---
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ensure odometry pods don't receive power
        intake.setPower(0);
        intake2.setPower(0);
        shooter.setPower(0);

        prevLeft = intake.getCurrentPosition();
        prevRight = intake2.getCurrentPosition();
        prevBack = shooter.getCurrentPosition();

        // --- Motor directions ---
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // --- Initialize IMU ---
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // --- Initialize Vision System ---
        initializeVision();

        lastPosition = flywheel.getCurrentPosition();
        dtTimer.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Vision", "Ready");
        telemetry.addData("Color Sensors", "Ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double nerf = 0.75;

        while (opModeIsActive()) {
            updateOdometry();
            updateVisionData();
            updateColorSensors();  // Update color sensor readings

            double batteryVoltage = getBatteryVoltage();

            double Logdrive = -gamepad1.left_stick_y * nerf;
            double LATdrive = -gamepad1.left_stick_x * nerf;
            double Turndrive = -gamepad1.right_stick_x * nerf;

            // --- Wheel brake toggle ---
            boolean currentLeftStick = gamepad1.left_stick_button;
            boolean currentRightStick = gamepad1.right_stick_button;

            if (currentLeftStick && currentRightStick &&
                    !(prevLeftStickButton && prevRightStickButton)) {
                // Button press detected (rising edge)
                wheelBreak = !wheelBreak;

                if (wheelBreak) {
                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    frontLeftDrive.setTargetPosition(0);
                    frontRightDrive.setTargetPosition(0);
                    backLeftDrive.setTargetPosition(0);
                    backRightDrive.setTargetPosition(0);

                    frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    wheelBreakTargetFL = frontLeftDrive.getCurrentPosition();
                    wheelBreakTargetFR = frontRightDrive.getCurrentPosition();
                    wheelBreakTargetBL = backLeftDrive.getCurrentPosition();
                    wheelBreakTargetBR = backRightDrive.getCurrentPosition();
                }
            }

            prevLeftStickButton = currentLeftStick;
            prevRightStickButton = currentRightStick;

            // --- Slow mode toggle ---
            boolean currentRightBumper = gamepad1.right_bumper;
            if (currentRightBumper && !prevRightBumper) {
                // Button press detected (rising edge)
                slow_mode = !slow_mode;
                nerf = slow_mode ? 0.1 : 0.75;
            }
            prevRightBumper = currentRightBumper;

            // --- Wheel brake control ---
            if (wheelBreak) {
                applyWheelBrake(frontLeftDrive, wheelBreakTargetFL);
                applyWheelBrake(frontRightDrive, wheelBreakTargetFR);
                applyWheelBrake(backLeftDrive, wheelBreakTargetBL);
                applyWheelBrake(backRightDrive, wheelBreakTargetBR);
                telemetry.addData("WHEEL BRAKE ACTIVE", "True");

            } else if (!wheelBreak && robot_centric) {
                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                backLeftDrive.setPower(Logdrive - LATdrive + Turndrive);
                backRightDrive.setPower(-Logdrive + LATdrive + Turndrive);
                frontLeftDrive.setPower(Logdrive + LATdrive + Turndrive);
                frontRightDrive.setPower(-Logdrive - LATdrive + Turndrive);

            } else if (!wheelBreak && field_centric) {
                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                double y = -gamepad1.left_stick_y * nerf;
                double x = gamepad1.left_stick_x * nerf;
                double rx = gamepad1.right_stick_x * nerf;

                if (gamepad1.start) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftDrive.setPower(frontLeftPower);
                backLeftDrive.setPower(backLeftPower);
                frontRightDrive.setPower(frontRightPower);
                backRightDrive.setPower(backRightPower);
            }

            // --- Gamepad 2 controls ---
            handleShooter();

            // --- Dashboard telemetry (includes vision and color sensor data) ---
            sendDashboardTelemetry(batteryVoltage);

            telemetry.update();
        }

        // Cleanup vision portal
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void applyWheelBrake(DcMotor motor, int target) {
        int error = target - motor.getCurrentPosition();
        error = Math.max(-maxError, Math.min(maxError, error));
        double power = kP * error;
        power = Math.max(-maxPower, Math.min(maxPower, power));
        motor.setPower(power);
    }

    private void handleShooter() {
        if (gamepad2.right_trigger > 0.2) {
            double pos = flywheel.getCurrentPosition();
            double dt = dtTimer.seconds();
            intakeToShooter.setPower(intakeToShooter_power);
            intakeToShooter2.setPower(intakeToShooter_power);

            if (dt < 0.001) dt = 0.001;
            dtTimer.reset();

            double deltaTicks = pos - lastPosition;
            lastPosition = pos;

            double velocityTicksPerSec = flywheel.getVelocity();
            currentRPM = (velocityTicksPerSec / TICKS_PER_REV) * 60.0;

            ff = 0.0;
            if (targetRPM > 20) {
                ff = kS + kV * targetRPM;
            }

            error = targetRPM - currentRPM;

            integral += error * dt;
            integral = Math.max(-integralLimit, Math.min(integralLimit, integral));

            double derivative = (error - lastError) / dt;
            lastError = error;

            pid = kP_F * error + kI * integral + kD * derivative;

            output = ff + pid;
            output = Math.max(-1.0, Math.min(1.0, output));

            flywheel.setPower(output);
        } else {
            intakeToShooter.setPower(0);
            intakeToShooter2.setPower(0);
            flywheel.setPower(0.24);
        }
    }

    private void sendDashboardTelemetry(double batteryVoltage) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        // --- Draw field grid ---
        canvas.setStroke("#404040");
        for (int i = -72; i <= 72; i += 24) {
            canvas.strokeLine(i, -72, i, 72);
            canvas.strokeLine(-72, i, 72, i);
        }

        canvas.setStroke("#FFFFFF");
        canvas.strokeRect(-72, -72, 144, 144);

        canvas.setStroke("#FFFF00");
        canvas.strokeLine(-10, 0, 10, 0);
        canvas.strokeLine(0, -10, 0, 10);

        double robotSize = 18;
        canvas.setStroke("#3FBAFF");
        canvas.setFill("#3FBAFF");
        canvas.fillRect(xPos - robotSize / 2, yPos - robotSize / 2, robotSize, robotSize);

        double headingLineLength = 12;
        double headingX = xPos + headingLineLength * Math.cos(heading);
        double headingY = yPos + headingLineLength * Math.sin(heading);
        canvas.setStroke("#FF0000");
        canvas.setStrokeWidth(3);
        canvas.strokeLine(xPos, yPos, headingX, headingY);

        canvas.setStroke("#00FF00");
        canvas.fillCircle(xPos, yPos, 3);

        // --- Odometry & Drive Telemetry ---
        packet.put("Wheel Brake Active", wheelBreak);
        packet.put("Intake Active", intakeActive);
        packet.put("Shooter Active", shooterActive);
        packet.put("Robot X (in)", xPos);
        packet.put("Robot Y (in)", yPos);
        packet.put("Heading (rad)", heading);
        packet.put("Heading (deg)", Math.toDegrees(heading));
        packet.put("Front Left Encoder", frontLeftDrive.getCurrentPosition());
        packet.put("Front Right Encoder", frontRightDrive.getCurrentPosition());
        packet.put("Back Left Encoder", backLeftDrive.getCurrentPosition());
        packet.put("Back Right Encoder", backRightDrive.getCurrentPosition());
        packet.put("Odo Left Raw", -intake.getCurrentPosition());
        packet.put("Odo Right Raw", intake2.getCurrentPosition());
        packet.put("Odo Back Raw", shooter.getCurrentPosition());
        packet.put("Slow Mode", slow_mode);
        packet.put("Battery Voltage (V)", batteryVoltage);

        // --- Shooter Telemetry ---
        packet.put("Target RPM", targetRPM);
        packet.put("Actual RPM", currentRPM);
        packet.put("Error RPM", error);
        packet.put("Error %", targetRPM != 0 ? (error / targetRPM) * 100.0 : 0);
        packet.put("Total Power", output);
        packet.put("FF Power", ff);
        packet.put("PID Power", pid);
        packet.put("Integral", integral);
        packet.put("ITS", intakeToShooter.getPower());
        packet.put("ITS2", intakeToShooter2.getPower());

        // --- VISION TELEMETRY ---
        String currentClosestColor = closestColor;
        int currentGreenCount = greenBallCount;
        int currentPurpleCount = purpleBallCount;
        double currentBallX = closestBallX;
        double currentBallY = closestBallY;

        packet.put("=== VISION ===", "");
        packet.put("Green Balls Detected", currentGreenCount);
        packet.put("Purple Balls Detected", currentPurpleCount);
        packet.put("Closest Ball Color", currentClosestColor);
        packet.put("Closest Ball X", currentBallX);
        packet.put("Closest Ball Y", currentBallY);

        // --- COLOR SENSOR TELEMETRY ---
        String currentDetectedColor = detectedColor;
        float currentCS1Hue = cs1Hue;
        float currentCS2Hue = cs2Hue;
        float currentAvgHue = avgHue;

        packet.put("=== COLOR SENSORS ===", "");
        packet.put("CS1 Hue", currentCS1Hue);
        packet.put("CS2 Hue", currentCS2Hue);
        packet.put("Average Hue", currentAvgHue);
        packet.put("Detected Color", currentDetectedColor);

        packet.put("Erik is a ", "Big Dumb");

        packet.put("THIS IS FOR OLD ROBOT DO NOT USE", "je");

        dashboard.sendTelemetryPacket(packet);

        // --- Phone Telemetry ---
        telemetry.addData("Wheel Brake Active", wheelBreak);
        telemetry.addData("Robot X (in)", xPos);
        telemetry.addData("Robot Y (in)", yPos);
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("Slow Mode", slow_mode);
        telemetry.addData("Battery Voltage (V)", batteryVoltage);

        // Vision data on phone
        telemetry.addLine("\n=== VISION ===");
        telemetry.addData("Green Balls", currentGreenCount);
        telemetry.addData("Purple Balls", currentPurpleCount);
        telemetry.addData("Closest Color", currentClosestColor);

        // Color sensor data on phone
        telemetry.addLine("\n=== COLOR SENSORS ===");
        telemetry.addData("CS1 Hue", currentCS1Hue);
        telemetry.addData("CS2 Hue", currentCS2Hue);
        telemetry.addData("Avg Hue", currentAvgHue);
        telemetry.addData("Detected", currentDetectedColor);


        telemetry.addData("THIS IS FOR OLD ROBOT DO NOT USE", "asdf");

        telemetry.addLine("\nErik is a Big Dumb");
    }
}