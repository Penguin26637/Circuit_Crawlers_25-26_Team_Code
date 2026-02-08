package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Rebuilt.MainPrograms.MotorPowerRegulator_New;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "ErikandBradyBrokeEverythingMTVision", group = "Main")
@Config
public class ErikandBradyBrokeEverythingMTVision extends LinearOpMode {

    // Hardware
    private DriveControlClassGregMT drive;
    private DcMotor odoLeft, odoRight, odoPerp;
    private ServoClassMT intakeLeft, intakeRight, spindexer, flipper;
    private MotorPowerRegulator_New shooter;
    private MultipleTelemetry multitelemetry;
    private CameraVisionOuttakeDetection cvos;
    private FtcDashboard dashboard;

    // Configuration
    private static final double NORMAL_SPEED = 0.75, SLOW_SPEED = 0.1;
    private static final boolean ENABLE_ODOMETRY = true;
    private static final double SHOOTER_ACTIVE_RPM = 1400.0, SHOOTER_IDLE_RPM = 0;
    private static final double TICKS_PER_INCH = 337.2, TRACK_WIDTH = 13.5, PERP_OFFSET = 8.0;

    // Vision configuration
    private static final double SHOOTER_READY_RPM_THRESHOLD = 120.0;
    private static final double FLIPPER_UP_POSITION = 0.5;
    private static final double FLIPPER_DOWN_POSITION = 0.25;

    // State
    private double xPos = 0, yPos = 0, odoHeading = 0;
    private int prevLeft = 0, prevRight = 0, prevPerp = 0;
    private boolean odometryInitialized = false;
    private boolean lastGamepad1A = false, lastGamepad1X = false, lastGamepad1RightBumper = false;
    private boolean lastGamepad1Start = false, lastGamepad1Back = false;
    private boolean lastGamepad2Y = false, lastGamepad2X = false;

    private boolean auto_shoot = false;
    private boolean vision_auto_shoot = false;
    private boolean shooteractive = false;
    private boolean visionInitialized = false;

    @Override
    public void runOpMode() {
        multitelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();

        // CREATE TELEMETRY PACKET ONCE - REUSE IT (like test program)
        TelemetryPacket packet = new TelemetryPacket();

        multitelemetry.addData("Status", "Initializing...");
        multitelemetry.update();

        try {
            // Initialize drive
            drive = new DriveControlClassGregMT(hardwareMap, multitelemetry, true, true, false);
            drive.nerf = NORMAL_SPEED;
            drive.useFieldCentric = false;
            drive.useImuForFieldCentric = true;

            // Initialize servos
            intakeLeft = new ServoClassMT("leftintake", ServoClassMT.ServoType.CONTINUOUS_SERVO);
            intakeRight = new ServoClassMT("rightintake", ServoClassMT.ServoType.CONTINUOUS_SERVO);
            spindexer = new ServoClassMT("spindexer", ServoClassMT.ServoType.CONTINUOUS_SERVO);
            flipper = new ServoClassMT("flipper", ServoClassMT.ServoType.STANDARD_SERVO);

            intakeLeft.init(hardwareMap);
            intakeRight.init(hardwareMap);
            spindexer.init(hardwareMap);
            flipper.init(hardwareMap);

            intakeLeft.stop();
            intakeRight.stop();
            spindexer.stop();
            flipper.stop();

            // Initialize shooter
            shooter = new MotorPowerRegulator_New(hardwareMap, multitelemetry, "shooter");
            shooter.setTicksPerRev(537.7);
            shooter.setMaxRpmUnderLoad(1400.0);
            shooter.setAllGains(0.00068, 0.06, 0.0004, 0.0002, 0.00005);
            shooter.setTargetRPM(SHOOTER_IDLE_RPM);

            // Initialize vision system - EXACT PATTERN AS WORKING TEST PROGRAM
            try {
                cvos = new CameraVisionOuttakeDetection(hardwareMap, dashboard);

                // Wait for camera to be ready
                while (cvos.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY && !isStopRequested()) {
                    packet.put("Camera State", "Not Ready");
                    dashboard.sendTelemetryPacket(packet);
                    multitelemetry.addData("Camera State", "Waiting...");
                    multitelemetry.update();
                    sleep(50);
                }
                packet.put("Camera State", "Ready");
                dashboard.sendTelemetryPacket(packet);

                visionInitialized = true;
                multitelemetry.addData("Vision System", "✓");
            } catch (Exception e) {
                visionInitialized = false;
                multitelemetry.addData("Vision System", "✗ - " + e.getMessage());
            }

            multitelemetry.addData("Status", "Initialized!");
            multitelemetry.addData("Drive Motors", drive.driveMotorsInitialized ? "✓" : "✗");
            multitelemetry.addData("IMU", drive.imuInitialized ? "✓" : "✗");
            multitelemetry.addData("Intake L/R", (intakeLeft.getCRServo() != null ? "✓" : "✗") + "/" + (intakeRight.getCRServo() != null ? "✓" : "✗"));
            multitelemetry.addData("Spindexer", spindexer.getCRServo() != null ? "✓" : "✗");
            multitelemetry.addData("Flipper", flipper.getServo() != null ? "✓" : "✗");
            multitelemetry.addData("Shooter", shooter.getMotor() != null ? "✓" : "✗");

            // Initialize Odometry
            if (ENABLE_ODOMETRY) {
                try {
                    odoLeft = hardwareMap.get(DcMotor.class, "ol");
                    odoRight = hardwareMap.get(DcMotor.class, "or");
                    odoPerp = hardwareMap.get(DcMotor.class, "perp");

                    odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Thread.sleep(100);

                    odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    odoPerp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    prevLeft = odoLeft.getCurrentPosition();
                    prevRight = odoRight.getCurrentPosition();
                    prevPerp = odoPerp.getCurrentPosition();

                    odometryInitialized = true;
                    multitelemetry.addData("Odometry", "✓");
                } catch (Exception e) {
                    odometryInitialized = false;
                    multitelemetry.addData("Odometry", "✗ - " + e.getMessage());
                }
            }
        } catch (Exception e) {
            multitelemetry.addData("ERROR", e.getMessage());
            multitelemetry.addData("Status", "Initialization Failed!");
        }

        multitelemetry.update();
        waitForStart();

        // Start camera stream and configure AFTER waitForStart (like test program)
        if (visionInitialized) {
            cvos.startCameraStreamSystem();
            packet.clearLines();
            dashboard.sendTelemetryPacket(packet);
            cvos.configureCameraControls();
        }

        // Main loop
        while (opModeIsActive()) {
            // Get inputs
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Toggle controls with debouncing - Gamepad 1
            if (gamepad1.a && !lastGamepad1A) drive.useFieldCentric = !drive.useFieldCentric;
            if (gamepad1.x && !lastGamepad1X) drive.useImuForFieldCentric = !drive.useImuForFieldCentric;
            if (gamepad1.right_bumper && !lastGamepad1RightBumper) drive.nerf = (drive.nerf == NORMAL_SPEED) ? SLOW_SPEED : NORMAL_SPEED;
            if (gamepad1.start && !lastGamepad1Start && drive.imuInitialized) drive.imu.resetYaw();
            if (gamepad1.back && !lastGamepad1Back && odometryInitialized) resetOdometry();

            lastGamepad1A = gamepad1.a;
            lastGamepad1X = gamepad1.x;
            lastGamepad1RightBumper = gamepad1.right_bumper;
            lastGamepad1Start = gamepad1.start;
            lastGamepad1Back = gamepad1.back;

            // Toggle controls with debouncing - Gamepad 2
            if (gamepad2.y && !lastGamepad2Y) auto_shoot = !auto_shoot;
            if (gamepad2.x && !lastGamepad2X) vision_auto_shoot = !vision_auto_shoot;

            lastGamepad2Y = gamepad2.y;
            lastGamepad2X = gamepad2.x;

            // Update odometry and drive
            if (odometryInitialized) updateOdometry();
            drive.update(true, forward, strafe, turn, odoHeading);

            // Intake controls
            if (gamepad2.left_bumper) {
                intakeLeft.goToPosition(-1.0);
                intakeRight.goToPosition(-1.0);
            } else if (gamepad2.right_bumper) {
                intakeLeft.goToPosition(1.0);
                intakeRight.goToPosition(1.0);
            } else {
                intakeLeft.stop();
                intakeRight.stop();
            }

            // Vision detection results (detect once per loop to avoid null pointer issues)
            String purpleDetected = "No Color";
            String greenDetected = "No Color";
            if (visionInitialized) {
                try {
                    purpleDetected = cvos.detectClosestObjectAndDecide("Purple");
                    greenDetected = cvos.detectClosestObjectAndDecide("Green");
                } catch (Exception e) {
                    // Ignore detection errors and continue
                }
            }

            // VISION AUTO-SHOOT MODE (Gamepad2.X toggles this)
            if (vision_auto_shoot && visionInitialized) {
                // Vision mode: Auto-control spindexer and flipper based on color detection
                boolean shooterReady = shooter.getCurrentRPM() >= SHOOTER_READY_RPM_THRESHOLD;

                // Check if either purple or green is detected in ROI
                boolean colorDetected = !purpleDetected.equals("No Color") || !greenDetected.equals("No Color");

                if (colorDetected && shooterReady) {
                    // Ball detected in ROI and shooter ready - SHOOT!
                    flipper.goToPosition(FLIPPER_UP_POSITION);
                    spindexer.stop();
                } else {
                    // No ball or shooter not ready - rotate spindexer to find ball
                    flipper.goToPosition(FLIPPER_DOWN_POSITION);
                    spindexer.goToPosition(1.0);
                }
            } else {
                // MANUAL MODE (original controls)

                // Manual Spindexer controls
                if (gamepad2.dpad_left) spindexer.goToPosition(-1);
                else if (gamepad2.dpad_right) spindexer.goToPosition(1);
                else spindexer.stop();

                // Flipper controls - Manual or Auto (non-vision)
                if (auto_shoot) {
                    // Auto shoot mode - automatically flip when shooter is ready
                    if (shooter.getCurrentRPM() >= SHOOTER_READY_RPM_THRESHOLD) {
                        flipper.goToPosition(FLIPPER_UP_POSITION);
                    } else {
                        flipper.goToPosition(FLIPPER_DOWN_POSITION);
                    }
                } else {
                    // Manual mode - only flip on button press
                    if (shooter.getCurrentRPM() >= 110 && gamepad2.dpad_up) {
                        flipper.goToPosition(FLIPPER_UP_POSITION);
                    } else if (gamepad2.dpad_down) {
                        flipper.goToPosition(FLIPPER_DOWN_POSITION);
                    }
                }
            }

            // Shooter controls
            if (gamepad2.a && !shooteractive){
                shooteractive = true;
                sleep(20);
            } else if (gamepad2.b && shooteractive) {
                shooteractive = false;
                sleep(20);
            }

            if (shooteractive)
                shooter.setTargetRPM((SHOOTER_ACTIVE_RPM));
            else{
                shooter.setTargetRPM(SHOOTER_IDLE_RPM);
            }
            shooter.loop();

            // Telemetry
            multitelemetry.addData("Drive Mode", drive.useFieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            if (drive.useFieldCentric) {
                multitelemetry.addData("Heading Source", drive.useImuForFieldCentric ? "IMU" : "ODOMETRY");
            }
            multitelemetry.addData("Speed", drive.nerf == SLOW_SPEED ? "SLOW" : "NORMAL");

            if (drive.useImuForFieldCentric && drive.imuInitialized) {
                multitelemetry.addData("IMU Heading", "%.2f°", Math.toDegrees(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
            }

            if (odometryInitialized) {
                multitelemetry.addData("Odo Position", "X:%.2f Y:%.2f", xPos, yPos);
                multitelemetry.addData("Odo Heading", "%.2f°", Math.toDegrees(odoHeading));
            }

            // Vision telemetry - REUSE SAME PACKET (like test program)
            if (visionInitialized) {
                try {
                    // Update blob data in the SAME packet
                    if (!cvos.getPurpleBlobs().isEmpty()) {
                        packet.put("Largest Purple Blob", cvos.getPurpleBlobs().get(0).getContourArea());
                    }
                    if (!cvos.getGreenBlobs().isEmpty()) {
                        packet.put("Largest Green Blob", cvos.getGreenBlobs().get(0).getContourArea());
                    }

                    packet.put("Purple Detected", purpleDetected);
                    packet.put("Green Detected", greenDetected);
                } catch (Exception e) {
                    // Ignore
                }

                dashboard.sendTelemetryPacket(packet);

                // Driver station telemetry
                multitelemetry.addData("Vision Auto-Shoot", vision_auto_shoot ? "ON 🟢" : "OFF 🔴");
                multitelemetry.addData("Purple Detected", purpleDetected);
                multitelemetry.addData("Green Detected", greenDetected);

                try {
                    if (cvos.getPurpleBlobs() != null) {
                        multitelemetry.addData("Purple Blobs", cvos.getPurpleBlobs().size());
                        if (!cvos.getPurpleBlobs().isEmpty()) {
                            multitelemetry.addData("Largest Purple Area", cvos.getPurpleBlobs().get(0).getContourArea());
                        }
                    }
                    if (cvos.getGreenBlobs() != null) {
                        multitelemetry.addData("Green Blobs", cvos.getGreenBlobs().size());
                        if (!cvos.getGreenBlobs().isEmpty()) {
                            multitelemetry.addData("Largest Green Area", cvos.getGreenBlobs().get(0).getContourArea());
                        }
                    }
                } catch (Exception e) {
                    multitelemetry.addData("Blob Error", e.getMessage());
                }
            }

            // Component status
            String intakeStatus = gamepad2.left_bumper ? "IN" : (gamepad2.right_bumper ? "OUT" : "STOP");
            String spindexerStatus;
            if (vision_auto_shoot && visionInitialized) {
                boolean colorDetected = !purpleDetected.equals("No Color") || !greenDetected.equals("No Color");
                boolean shooterReady = shooter.getCurrentRPM() >= SHOOTER_READY_RPM_THRESHOLD;
                spindexerStatus = (colorDetected && shooterReady) ? "STOP (SHOOTING)" : "ROTATING (SEARCHING)";
            } else {
                spindexerStatus = gamepad2.dpad_left ? "LEFT" : (gamepad2.dpad_right ? "RIGHT" : "STOP");
            }

            multitelemetry.addData("Intake", intakeStatus);
            multitelemetry.addData("Spindexer", spindexerStatus);
            multitelemetry.addData("Flipper Pos", "%.2f", flipper.getCurrentPosition());
            multitelemetry.addData("Auto Shoot", auto_shoot ? "ON 🟢" : "OFF 🔴");
            multitelemetry.addData("Shooter Target", "%.0f RPM", shooter.getTargetRPM());
            multitelemetry.addData("Shooter Current", "%.0f RPM %s", shooter.getCurrentRPM(), shooter.isAtTarget(50) ? "✓" : "✗");

            multitelemetry.addData("", "=== Controls GM 1 ===");
            multitelemetry.addData("GP1 A", "Toggle Field/Robot");
            multitelemetry.addData("GP1 X", "Toggle IMU/Odo");
            multitelemetry.addData("GP1 RB", "Toggle Speed");
            multitelemetry.addData("GP1 Start/Back", "Reset IMU/Odo");
            multitelemetry.addData("", "=== Controls GM2 ===");
            multitelemetry.addData("GP2 Bumpers", "Intake");
            multitelemetry.addData("GP2 DPad L/R", "Spindexer (Manual)");
            multitelemetry.addData("GP2 DPad U/D", "Flipper (Manual)");
            multitelemetry.addData("GP2 X", "Toggle Vision Auto-Shoot");
            multitelemetry.addData("GP2 Y", "Toggle Auto Shoot");
            multitelemetry.addData("GP2 A/B", "Shooter On/Off");
            multitelemetry.update();

            sleep(30);
        }

        // Cleanup vision system
        if (visionInitialized) {
            cvos.stopCameraStreamandVision();
        }
    }

    private void updateOdometry() {
        int leftPos = -1 * odoLeft.getCurrentPosition();
        int rightPos = odoRight.getCurrentPosition();
        int perpPos = odoPerp.getCurrentPosition();

        int deltaLeft = leftPos - prevLeft;
        int deltaRight = rightPos - prevRight;
        int deltaPerp = perpPos - prevPerp;

        prevLeft = leftPos;
        prevRight = rightPos;
        prevPerp = perpPos;

        double dLeft = deltaLeft / TICKS_PER_INCH;
        double dRight = deltaRight / TICKS_PER_INCH;
        double dPerp = deltaPerp / TICKS_PER_INCH;

        double dHeading = (dRight - dLeft) / TRACK_WIDTH;
        double dForward = (dLeft + dRight) / 2.0;
        double dSide = dPerp - (dHeading * PERP_OFFSET);

        double sinH = Math.sin(odoHeading);
        double cosH = Math.cos(odoHeading);

        xPos += dForward * cosH - dSide * sinH;
        yPos += dForward * sinH + dSide * cosH;
        odoHeading += dHeading;
    }

    private void resetOdometry() {
        xPos = yPos = odoHeading = 0;

        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        try { Thread.sleep(100); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }

        odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        prevLeft = odoLeft.getCurrentPosition();
        prevRight = odoRight.getCurrentPosition();
        prevPerp = odoPerp.getCurrentPosition();
    }
}