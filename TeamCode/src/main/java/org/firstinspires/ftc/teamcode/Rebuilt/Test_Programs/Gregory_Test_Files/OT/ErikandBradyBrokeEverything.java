package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Rebuilt.MainPrograms.MotorPowerRegulator_New;

/**
 * ErikandBradyBrokeEverything - Simplified TeleOp
 * Robot-Centric and Field-Centric drive control with Odometry support
 * Uses DriveControlClassGreg for drive, odometry handled directly in file
 */
@TeleOp(name = "ErikandBradyBrokeEverything", group = "Main")
public class ErikandBradyBrokeEverything extends LinearOpMode {

    // ========== HARDWARE ==========
    private DriveControlClassGreg drive;

    // Odometry encoders - handled directly in this file
    private DcMotor odoLeft;
    private DcMotor odoRight;
    private DcMotor odoPerp;

    // Intake servos
    private org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass intakeLeft;
    private org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass intakeRight;

    // Shooter motor
    private MotorPowerRegulator_New shooter;

    // ========== CONFIGURATION ==========
    private static final double NORMAL_SPEED = 0.75;
    private static final double SLOW_SPEED = 0.1;
    private static final boolean ENABLE_ODOMETRY = true;  // Set to false to disable odometry

    // Shooter constants
    private static final double SHOOTER_ACTIVE_RPM = 1400.0;
    private static final double SHOOTER_IDLE_RPM = 700.0;

    // Odometry constants
    private static final double TICKS_PER_INCH = 337.2;
    private static final double TRACK_WIDTH = 13.5;
    private static final double PERP_OFFSET = 8.0;

    // ========== STATE VARIABLES ==========
    // Odometry position variables
    private double xPos = 0;
    private double yPos = 0;
    private double odoHeading = 0;
    private int prevLeft = 0;
    private int prevRight = 0;
    private int prevPerp = 0;
    private boolean odometryInitialized = false;

    // ========== BUTTON DEBOUNCING ==========
    private boolean lastGamepad1A = false;
    private boolean lastGamepad1X = false;
    private boolean lastGamepad1RightBumper = false;
    private boolean lastGamepad1Start = false;
    private boolean lastGamepad1Back = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // ========== INITIALIZE HARDWARE ==========
        try {
            // Initialize drive using DriveControlClassGreg
            drive = new DriveControlClassGreg(hardwareMap, telemetry, true, true, false);

            // Set initial drive states
            drive.nerf = NORMAL_SPEED;
            drive.useFieldCentric = false;
            drive.useImuForFieldCentric = true;

            // Initialize intake servos
            intakeLeft = new org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass("intakeLeft", org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass.ServoType.CONTINUOUS_SERVO);
            intakeRight = new org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass("intakeRight", org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass.ServoType.CONTINUOUS_SERVO);

            intakeLeft.init(hardwareMap);
            intakeRight.init(hardwareMap);

            // Start with intakes stopped
            intakeLeft.stop();
            intakeRight.stop();

            // Initialize shooter motor
            shooter = new MotorPowerRegulator_New(hardwareMap, telemetry, "shooter");
            shooter.setTicksPerRev(537.7);
            shooter.setMaxRpmUnderLoad(300.0);
            shooter.setAllGains(0.00068, 0.06, 0.0004, 0.0002, 0.00005);
            shooter.setTargetRPM(SHOOTER_IDLE_RPM);

            telemetry.addData("Status", "Initialized!");
            telemetry.addData("Drive Motors", drive.driveMotorsInitialized ? "✓" : "✗");
            telemetry.addData("IMU", drive.imuInitialized ? "✓" : "✗");
            telemetry.addData("Intake Left", intakeLeft.getCRServo() != null ? "✓" : "✗");
            telemetry.addData("Intake Right", intakeRight.getCRServo() != null ? "✓" : "✗");
            telemetry.addData("Shooter", shooter.getMotor() != null ? "✓" : "✗");

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
                    telemetry.addData("Odometry", "✓");
                } catch (Exception e) {
                    odometryInitialized = false;
                    telemetry.addData("Odometry", "✗ - " + e.getMessage());
                }
            }

        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
            telemetry.addData("Status", "Initialization Failed!");
        }

        telemetry.update();
        waitForStart();

        // ========== MAIN CONTROL LOOP ==========
        while (opModeIsActive()) {

            // ========== GET JOYSTICK INPUTS ==========
            double forward = -gamepad1.left_stick_y;   // Forward/Backward
            double strafe = gamepad1.left_stick_x;      // Left/Right
            double turn = gamepad1.right_stick_x;       // Rotation

            // ========== TOGGLE CONTROLS ==========

            // Toggle Field-Centric / Robot-Centric (A button)
            if (gamepad1.a && !lastGamepad1A) {
                drive.useFieldCentric = !drive.useFieldCentric;
            }
            lastGamepad1A = gamepad1.a;

            // Toggle IMU / Odometry for heading (X button)
            if (gamepad1.x && !lastGamepad1X) {
                drive.useImuForFieldCentric = !drive.useImuForFieldCentric;
            }
            lastGamepad1X = gamepad1.x;

            // Toggle Slow Mode (Right Bumper)
            if (gamepad1.right_bumper && !lastGamepad1RightBumper) {
                drive.nerf = (drive.nerf == NORMAL_SPEED) ? SLOW_SPEED : NORMAL_SPEED;
            }
            lastGamepad1RightBumper = gamepad1.right_bumper;

            // Reset IMU Heading (Start button)
            if (gamepad1.start && !lastGamepad1Start) {
                if (drive.imuInitialized) {
                    drive.imu.resetYaw();
                }
            }
            lastGamepad1Start = gamepad1.start;

            // Reset Odometry (Back button)
            if (gamepad1.back && !lastGamepad1Back && odometryInitialized) {
                resetOdometry();
            }
            lastGamepad1Back = gamepad1.back;

            // ========== UPDATE ODOMETRY ==========
            if (odometryInitialized) {
                updateOdometry();
            }

            // ========== UPDATE DRIVE ==========
            // Update drive (pass odometry heading for field centric option)
            // Note: nerf is already applied inside DriveControlClass
            drive.update(true, forward, strafe, turn, odoHeading);

            // ========== INTAKE CONTROLS (Gamepad 2) ==========
            // Left Bumper = both intakes reverse (intake in)
            // Right Bumper = both intakes forward (intake out)
            // Neither = stop
            if (gamepad2.left_bumper) {
                intakeLeft.goToPosition(-1.0);   // Reverse
                intakeRight.goToPosition(-1.0);  // Reverse
            } else if (gamepad2.right_bumper) {
                intakeLeft.goToPosition(1.0);    // Forward
                intakeRight.goToPosition(1.0);   // Forward
            } else {
                intakeLeft.stop();               // Stop
                intakeRight.stop();              // Stop
            }

            // ========== SHOOTER CONTROLS (Gamepad 2) ==========
            // Right Trigger = shooter active (1400 RPM)
            // Released = shooter idle (700 RPM)
            if (gamepad2.right_trigger >= 0.2) {
                shooter.setTargetRPM(SHOOTER_ACTIVE_RPM);
            } else {
                shooter.setTargetRPM(SHOOTER_IDLE_RPM);
            }

            // Update shooter motor controller
            shooter.loop();

            // ========== TELEMETRY ==========
            telemetry.addData("Drive Mode", drive.useFieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            if (drive.useFieldCentric) {
                telemetry.addData("Heading Source", drive.useImuForFieldCentric ? "IMU" : "ODOMETRY");
            }
            telemetry.addData("Speed Mode", (drive.nerf == SLOW_SPEED) ? "SLOW (" + SLOW_SPEED + ")" : "NORMAL (" + NORMAL_SPEED + ")");

            if (drive.useImuForFieldCentric && drive.imuInitialized) {
                telemetry.addData("IMU Heading", "%.2f°", Math.toDegrees(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
            }

            if (odometryInitialized) {
                telemetry.addData("", "");
                telemetry.addData("Odometry Position", "X: %.2f, Y: %.2f", xPos, yPos);
                telemetry.addData("Odometry Heading", "%.2f°", Math.toDegrees(odoHeading));
            }

            telemetry.addData("", "");
            telemetry.addData("Intake Status", "");
            String intakeStatus = "STOPPED";
            if (gamepad2.left_bumper) {
                intakeStatus = "IN (Reverse)";
            } else if (gamepad2.right_bumper) {
                intakeStatus = "OUT (Forward)";
            }
            telemetry.addData("  Intakes", intakeStatus);

            telemetry.addData("", "");
            telemetry.addData("Shooter Status", "");
            telemetry.addData("  Target RPM", "%.0f", shooter.getTargetRPM());
            telemetry.addData("  Current RPM", "%.0f", shooter.getCurrentRPM());
            telemetry.addData("  At Target", shooter.isAtTarget(50) ? "✓" : "✗");

            telemetry.addData("", "");
            telemetry.addData("Controls", "");
            telemetry.addData("  A Button", "Toggle Field/Robot Centric");
            telemetry.addData("  X Button", "Toggle IMU/Odometry Heading");
            telemetry.addData("  Right Bumper", "Toggle Slow Mode");
            telemetry.addData("  Start Button", "Reset IMU Heading");
            if (odometryInitialized) {
                telemetry.addData("  Back Button", "Reset Odometry");
            }
            telemetry.addData("", "");
            telemetry.addData("Gamepad 2:", "");
            telemetry.addData("  Left Bumper", "Intake In");
            telemetry.addData("  Right Bumper", "Intake Out");
            telemetry.addData("  Right Trigger", "Shooter Active");
            telemetry.update();

            sleep(20);
        }
    }

    // ========== ODOMETRY METHODS ==========

    /**
     * Update odometry position
     */
    private void updateOdometry() {
        // Read current encoder positions
        int leftPos = -1 * odoLeft.getCurrentPosition();
        int rightPos = odoRight.getCurrentPosition();
        int perpPos = odoPerp.getCurrentPosition();

        // Calculate deltas
        int deltaLeft = leftPos - prevLeft;
        int deltaRight = rightPos - prevRight;
        int deltaPerp = perpPos - prevPerp;

        prevLeft = leftPos;
        prevRight = rightPos;
        prevPerp = perpPos;

        // Convert to inches
        double dLeft = deltaLeft / TICKS_PER_INCH;
        double dRight = deltaRight / TICKS_PER_INCH;
        double dPerp = deltaPerp / TICKS_PER_INCH;

        // Calculate changes
        double dHeading = (dRight - dLeft) / TRACK_WIDTH;
        double dForward = (dLeft + dRight) / 2.0;
        double dSide = dPerp - (dHeading * PERP_OFFSET);

        // Update position
        double sinH = Math.sin(odoHeading);
        double cosH = Math.cos(odoHeading);

        xPos += dForward * cosH - dSide * sinH;
        yPos += dForward * sinH + dSide * cosH;
        odoHeading += dHeading;
    }

    /**
     * Reset odometry position
     */
    private void resetOdometry() {
        xPos = 0;
        yPos = 0;
        odoHeading = 0;

        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        prevLeft = odoLeft.getCurrentPosition();
        prevRight = odoRight.getCurrentPosition();
        prevPerp = odoPerp.getCurrentPosition();
    }
}