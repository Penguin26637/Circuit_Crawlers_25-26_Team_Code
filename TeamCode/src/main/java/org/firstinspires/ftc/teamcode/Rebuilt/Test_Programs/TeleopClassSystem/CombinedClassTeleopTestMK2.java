package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * CombinedClassTeleopTest - Main control hub
 * ALL variables are controlled from here
 * Classes only handle HOW variables interact, not WHAT values they have
 */
@TeleOp(name = "Combined Class Teleop Test", group = "Main")
@Config
public class CombinedClassTeleopTestMK2 extends LinearOpMode {

    // ========== ENABLE/DISABLE FLAGS ==========
    // Drive components
    public static boolean driveMotorsAttached = true;
    public static boolean imuAttached = true;
    public static boolean backMotorPidAttached = true;

    // Intake configuration
    public static boolean IntakeAttached = true;
    public static String intake1Type = "servo";  // "servo", "crservo", "motor", or "none"
    public static String intake2Type = "servo";  // "servo", "crservo", "motor", or "none"

    // Shooter components
    public static boolean shooterAttached = false;
    public static boolean shooterHinge1Attached = false;
    public static boolean shooterHinge2Attached = false;


    // Magazine configuration
    public static boolean magazineAttached = false;
    public static boolean shooterHingeAttached = false;
    public static String magazine1Type = "crservo";  // "servo", "crservo", "motor", or "none"
    public static String magazine2Type = "crservo";  // "servo", "crservo", "motor", or "none
    public static String magazine3Type = "crservo";  // "servo", "crservo", "motor", or "none"
    public static String magazine4Type = "crservo";  // "servo", "crservo", "motor", or "none"

    public static String shooterHinge1Type = "crservo";  // "servo", "crservo", "motor", or "none"
    public static String shooterHinge2Type = "crservo";  // "servo", "crservo", "motor", or "none"

    // Other subsystems
    public static boolean OdometryAttached = true;
    public static boolean TelemetryEnabled = true;
    public static boolean DashboardEnabled = true;



    // ========== DRIVE CONFIGURATION ==========
    public static double normalSpeed = 0.75;
    public static double slowSpeed = 0.1;
    public static boolean startInSlowMode = false;
    public static boolean startInFieldCentric = false;
    public static boolean startWithImu = true;  // true = use IMU, false = use odometry

    // ========== INTAKE CONFIGURATION ==========
    public static double intakeInPosition = 0.5;
    public static double intakeOutPosition = 0;

    // ========== SHOOTER CONFIGURATION ==========
    public static double shooterActiveRPM = 1400;
    public static double shooterIdleRPM = 0;
    public static double magazineActivePower = 0.5;
    public static double hinge1UpPosition = 1.0;
    public static double hinge2UpPosition = 1.0;
    public static double hinge1DownPosition = 0.0;
    public static double hinge2DownPosition = 0.0;



    // ========== CLASS INSTANCES ==========
    private DriveControlClass drive;
    private IntakeClass intake;
    private ShooterClassMK2 shooter;
    private OdometryClass odometry;
    private TelemetryClass telem;

    // ========== BUTTON DEBOUNCING ==========
    private boolean lastGamepad1A = false;
    private boolean lastGamepad1B = false;
    private boolean lastGamepad1X = false;  // For IMU toggle
    private boolean lastGamepad1RightBumper = false;
    private boolean lastGamepad1BothSticks = false;
    private boolean lastGamepad1Start = false;
    private boolean lastGamepad2A = false;
    private boolean lastGamepad2LeftBumper = false;

    // ========== STATE VARIABLES (controlled here, not in classes) ==========
    private boolean slowModeActive = startInSlowMode;
    private boolean intakeIsIn = false;
    private boolean hinge1IsUp = false;
    private boolean hinge2IsUp = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // ========== INITIALIZE CLASSES ==========
        drive = new DriveControlClass(hardwareMap, telemetry, driveMotorsAttached, imuAttached, backMotorPidAttached);

        if (IntakeAttached) {
            intake = new IntakeClass(hardwareMap, telemetry, intake1Type, intake2Type);
        }

        if (shooterAttached || shooterHingeAttached || magazineAttached) {
            shooter = new ShooterClassMK2(hardwareMap, telemetry,
                    shooterAttached, shooterHinge1Type, shooterHinge2Type, magazine1Type, magazine2Type, magazine3Type, magazine4Type);
        }

        if (OdometryAttached) {
            odometry = new OdometryClass(hardwareMap, telemetry);
        }

//        telem = new TelemetryClass(telemetry, hardwareMap);
//
//        // Link classes to telemetry
//        telem.drive = drive;
//        telem.intake = intake;
//        telem.shooter = shooter;
//        telem.odometry = odometry;

        // Set initial states in classes
        drive.nerf = slowModeActive ? slowSpeed : normalSpeed;
        drive.useFieldCentric = startInFieldCentric;
        drive.useImuForFieldCentric = startWithImu;
        drive.useBackMotorPid = backMotorPidAttached;  // Set initial back motor PID state

        if (IntakeAttached) {
            intake.targetPosition = intakeOutPosition;
        }


        // Display status
        telemetry.addData("Status", "Initialized!");
        telemetry.addData("Drive Motors", drive.driveMotorsInitialized ? "✓" : "✗");
        if (imuAttached) telemetry.addData("IMU", drive.imuInitialized ? "✓" : "✗");
        if (backMotorPidAttached) telemetry.addData("Back PID", drive.backPidInitialized ? "✓" : "✗");
        if (IntakeAttached) {
            if (!intake1Type.equals("none")) telemetry.addData("Intake 1", intake.getIntake1Initialized() ? "✓" : "✗");
            if (!intake2Type.equals("none")) telemetry.addData("Intake 2", intake.getIntake2Initialized() ? "✓" : "✗");
        }
        if (shooterAttached) telemetry.addData("Shooter", shooter.getShooterInitialized() ? "✓" : "✗");
        if (magazineAttached) {
            if (!magazine1Type.equals("none")) telemetry.addData("Mag 1", shooter.getMagazine1Initialized() ? "✓" : "✗");
            if (!magazine2Type.equals("none")) telemetry.addData("Mag 2", shooter.getMagazine2Initialized() ? "✓" : "✗");
            if (!magazine3Type.equals("none")) telemetry.addData("Mag 3", shooter.getMagazine3Initialized() ? "✓" : "✗");
            if (!magazine4Type.equals("none")) telemetry.addData("Mag 4", shooter.getMagazine4Initialized() ? "✓" : "✗");
        }
        if (shooterHingeAttached) {
            if (!shooterHinge1Type.equals("none")) telemetry.addData("Hinge 1", shooter.getHinge1Initialized() ? "✓" : "✗");
            if (!shooterHinge2Type.equals("none")) telemetry.addData("Hinge 2", shooter.getHinge1Initialized() ? "✓" : "✗");
        if (OdometryAttached) telemetry.addData("Odometry", odometry.getInitialized() ? "✓" : "✗");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ==================== GAMEPAD 1 - DRIVER ====================

            // Get raw inputs
//            double rawForward = -gamepad1.left_stick_y;
//            double rawStrafe = -gamepad1.left_stick_x;
//            double rawTurn = -gamepad1.right_stick_x;

            // Apply speed multiplier (controlled here, not in class)
//            double forward = rawForward * drive.nerf;
//            double strafe = rawStrafe * drive.nerf;
//            double turn = rawTurn * drive.nerf;

            double forward = -gamepad1.left_stick_y * drive.nerf;
            double strafe = -gamepad1.left_stick_x * drive.nerf;
            double turn = -gamepad1.right_stick_x * drive.nerf;

            // ========== DRIVE CONTROL TOGGLES ==========

            // Toggle field/robot centric (A button)
            if (gamepad1.a && !lastGamepad1A) {
                drive.useFieldCentric = !drive.useFieldCentric;
            }
            lastGamepad1A = gamepad1.a;

            // Toggle back motor PID (B button) - only if BackMotorPidAttached is true
            if (backMotorPidAttached && gamepad1.b && !lastGamepad1B) {
                drive.useBackMotorPid = !drive.useBackMotorPid;
            }
            lastGamepad1B = gamepad1.b;

            // Toggle IMU vs Odometry for field centric (X button)
            if (gamepad1.x && !lastGamepad1X) {
                drive.useImuForFieldCentric = !drive.useImuForFieldCentric;
            }
            lastGamepad1X = gamepad1.x;

            // Toggle slow mode (Right Bumper)
            if (gamepad1.right_bumper && !lastGamepad1RightBumper) {
                slowModeActive = !slowModeActive;
                drive.nerf = slowModeActive ? slowSpeed : normalSpeed;
            }
            lastGamepad1RightBumper = gamepad1.right_bumper;

            // Toggle wheel brake (Both stick buttons)
            boolean bothSticks = gamepad1.left_stick_button && gamepad1.right_stick_button;
            if (bothSticks && !lastGamepad1BothSticks) {
                drive.useWheelBrake = !drive.useWheelBrake;

                if (drive.useWheelBrake) {
                    drive.initWheelBrake();  // Initialize brake targets
                }
            }
            lastGamepad1BothSticks = bothSticks;

            // Reset IMU (Start button)
            if (gamepad1.start && !lastGamepad1Start) {
                drive.imu.resetYaw();
            }
            lastGamepad1Start = gamepad1.start;

            // Reset odometry (Back button)
            if (gamepad1.back && OdometryAttached) {
                odometry.resetPosition();
            }

            // Update drive (pass odometry heading for field centric option)
            double odoHeading = (OdometryAttached && odometry.getInitialized()) ? odometry.heading : 0;
            drive.update(driveMotorsAttached, forward, strafe, turn, odoHeading);


            // ==================== GAMEPAD 2 - OPERATOR ====================

            // ========== INTAKE CONTROL ==========
            if (IntakeAttached) {
                // Toggle intake (Left Bumper)
                if (gamepad2.left_bumper && !lastGamepad2LeftBumper) {
                    intakeIsIn = !intakeIsIn;
                    // Set both position and power - class will use the right one based on type
                    intake.targetPosition = intakeIsIn ? intakeInPosition : intakeOutPosition;
                    intake.targetPower = intakeIsIn ? 1.0 : 0.0;  // For CR servos/motors
                }
                lastGamepad2LeftBumper = gamepad2.left_bumper;

                intake.update(IntakeAttached);
            }


            // ========== SHOOTER CONTROL ==========
            if (shooterAttached || shooterHingeAttached || magazineAttached) {

                // Shooter motor (Right Trigger) - only if shooter attached
                if (shooterAttached) {
                    if (gamepad2.right_trigger >= 0.2) {
                        shooter.shooterTargetRPM = shooterActiveRPM;
                    } else {
                        shooter.shooterTargetRPM = shooterIdleRPM;
                    }
                }

                // Magazine (Right Trigger) - only if magazine attached
                if (magazineAttached) {
                    if (gamepad2.right_trigger >= 0.2) {
                        // Set both - class will use the right one based on type
                        shooter.magazineTargetPower = magazineActivePower;
                        shooter.magazineTargetPosition = 1.0;  // For regular servos
                    } else {
                        shooter.magazineTargetPower = 0;
                        shooter.magazineTargetPosition = 0;
                    }
                }

                // Toggle hinge (A button) - only if hinge attached
                if (shooterHingeAttached) {
                    if (gamepad2.right_trigger >= 0.2) {
                        // Set both - class will use the right one based on type
                        shooter.magazineTargetPower = magazineActivePower;
                        shooter.magazineTargetPosition = 1.0;  // For regular servos
                    } else {
                        shooter.magazineTargetPower = 0;
                        shooter.magazineTargetPosition = 0;
                    }
                }

                // Update shooter (pass individual enable flags)
                shooter.update(shooterAttached, shooterHingeAttached, magazineAttached);
            }


            // ==================== UPDATE OTHER SYSTEMS ====================

            // Update odometry
            if (OdometryAttached) {
                odometry.update(OdometryAttached);
            }

            // Update telemetry
            telem.update(TelemetryEnabled, DashboardEnabled);

            sleep(20);
            }
        }
    }
}