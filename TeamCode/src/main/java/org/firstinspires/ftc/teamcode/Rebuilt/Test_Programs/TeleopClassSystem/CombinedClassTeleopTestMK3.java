package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



// TODO: There will be some refactoring required when you make changes
//       to the other classes, but nothing structurally wrong here.

/**
 * CombinedClassTeleopTest MK3 - Main control hub
 * ALL variables are controlled from here
 * Classes only handle HOW variables interact, not WHAT values they have
 * Added spindexer and flipper servos
 *
 * FIXED: Set shooterActiveRPM to 1400 (was 600)
 */
@TeleOp(name = "Combined Class Teleop Test MK3 .java", group = "Main")
@Config
public class CombinedClassTeleopTestMK3 extends LinearOpMode {

    // ========== ENABLE/DISABLE FLAGS ==========
    // Drive components
    public static boolean driveMotorsAttached = true;
    public static boolean imuAttached = true;
    public static boolean backMotorPidAttached = false;

    // Intake configuration
    public static boolean IntakeAttached = false;
    public static String intake1Type = "crservo";  // "servo", "crservo", "motor", or "none"
    public static String intake2Type = "crservo";  // "servo", "crservo", "motor", or "none"

    // Shooter components
    public static boolean shooterAttached = true;

    // Magazine configuration
    public static boolean magazineAttached = false;
    public static boolean shooterHingeAttached = false;
    public static boolean spindexerAttached = false;
    public static boolean flipperAttached = false;
    public static String magazine1Type = "none";  // "servo", "crservo", "motor", or "none"
    public static String magazine2Type = "none";  // "servo", "crservo", "motor", or "none
    public static String magazine3Type = "none";  // "servo", "crservo", "motor", or "none"
    public static String magazine4Type = "none";  // "servo", "crservo", "motor", or "none"

    public static String shooterHinge1Type = "none";  // "servo", "crservo", "motor", or "none"
    public static String shooterHinge2Type = "none";  // "servo", "crservo", "motor", or "none"

    public static String spindexerType = "crservo";  // "servo", "crservo", "motor", or "none"
    public static String flipperType = "servo";  // "servo", "crservo", "motor", or "none"

    // Other subsystems
    public static boolean OdometryAttached = false;
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
    public static double shooterActiveRPM = 1400;  // ✓ FIXED: Changed from 600 to 1400
    public static double shooterIdleRPM = 700;
    public static double magazineActivePower = 0.5;

    // ========== SPINDEXER CONFIGURATION ==========
    public static double spindexerActivePosition = 0.5;
    public static double spindexerIdlePosition = 0;
    public static double spindexerActivePower = 1.0;

    // ========== FLIPPER CONFIGURATION ==========
    public static double flipperActivePosition = 0.2;
    public static double flipperIdlePosition = 0;
    public static double flipperActivePower = 1.0;


    private IntakeClass intake;
    private ShooterClassMK3 shooter;
    private OdometryClass odometry;

    // ========== BUTTON DEBOUNCING ==========
    private boolean lastGamepad1A = false;
    private boolean lastGamepad1B = false;
    private boolean lastGamepad1X = false;  // For IMU toggle
    private boolean lastGamepad1RightBumper = false;
    private boolean lastGamepad1BothSticks = false;
    private boolean lastGamepad1Start = false;
    private boolean lastGamepad2A = false;

    // ========== STATE VARIABLES (controlled here, not in classes) ==========
    private boolean slowModeActive = startInSlowMode;
    private boolean intakeIsIn = false;
    private boolean hinge1IsUp = false;
    private boolean hinge2IsUp = false;

    private CRServo spindexerCRServo;
    private Servo flipperServo;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        IntakeAttached = !intake1Type.equals("none") || !intake2Type.equals("none");
        magazineAttached = !magazine1Type.equals("none") || !magazine2Type.equals("none") || !magazine3Type.equals("none") || !magazine4Type.equals("none");
        shooterHingeAttached = !shooterHinge1Type.equals("none") || !shooterHinge2Type.equals("none");
        spindexerAttached = !spindexerType.equals("none");
        flipperAttached = !flipperType.equals("none");

        telemetry.addData("IntakeAttached", IntakeAttached);
        telemetry.addData("Mag attached", magazineAttached);
        telemetry.addData("shooter hinge attached", shooterHingeAttached);
        telemetry.addData("spindexer attached", spindexerAttached);
        telemetry.addData("flipper attached", flipperAttached);
        // ========== INITIALIZE CLASSES ==========
        // ========== CLASS INSTANCES ==========
        DriveControlClass drive = new DriveControlClass(hardwareMap, telemetry, driveMotorsAttached, imuAttached, backMotorPidAttached);

        if (IntakeAttached) {
            intake = new IntakeClass(hardwareMap, telemetry, intake1Type, intake2Type);
        }

        spindexerCRServo = hardwareMap.get(CRServo.class, 'spin')
        flipperServo = hardwareMap.get(Servo.class, 'flip')

        if (shooterAttached) {
            shooter = new ShooterClassMK3(spindexerCRServo, flipperServo, telemetry, hardwareMap, telemetry);
        }

        if (OdometryAttached) {
            odometry = new OdometryClass(hardwareMap, telemetry);
        }

        TelemetryClassMK3 telem = new TelemetryClassMK3(telemetry, hardwareMap, shooter);

        // Link classes to telemetry
        telem.drive = drive;
        telem.intake = intake;
        telem.shooter = shooter;
        telem.odometry = odometry;

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
            if (!shooterHinge1Type.equals("none"))
                telemetry.addData("Hinge 1", shooter.getHinge1Initialized() ? "✓" : "✗");
            if (!shooterHinge2Type.equals("none"))
                telemetry.addData("Hinge 2", shooter.getHinge2Initialized() ? "✓" : "✗");
        }
        if (spindexerAttached) {
            if (!spindexerType.equals("none"))
                telemetry.addData("Spindexer", shooter.getSpindexerInitialized() ? "✓" : "✗");
        }
        if (flipperAttached) {
            if (!flipperType.equals("none"))
                telemetry.addData("Flipper", shooter.getFlipperInitialized() ? "✓" : "✗");
        }
        if (OdometryAttached) telemetry.addData("Odometry", odometry.getInitialized() ? "✓" : "✗");
        telemetry.addData("Troubleshooting: ", hardwareMap.tryGet(DcMotor.class, "frontl"));

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
                // Intake control - Hold buttons to run
                // Left Bumper = intake in (forward power/position)
                // Right Bumper = intake out (reverse power/position)
                // Neither = stop (CRServo/Motor only, servos hold position)
                if (gamepad2.left_bumper) {
                    // Intake in - forward
                    intake.targetPosition = intakeInPosition;
                    intake.targetPower = 1.0;
                } else if (gamepad2.right_bumper) {
                    // Intake out - reverse
                    intake.targetPosition = intakeOutPosition;
                    intake.targetPower = -1.0;
                } else {
                    // Stop (only affects CRServo and Motor, servos ignore power)
                    intake.targetPower = 0;
                }

                intake.update(IntakeAttached);
            }


            // ========== SHOOTER CONTROL ==========
            if (shooterAttached || shooterHingeAttached || magazineAttached || spindexerAttached || flipperAttached) {

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

                // Toggle hinge (Left Trigger) - only if hinge attached
                if (shooterHingeAttached) {
                    if (gamepad2.left_trigger >= 0.2) {
                        // Set both - class will use the right one based on type
                        shooter.hingeTargetPower = 1.0;
//                        shooter.hingeTargetPower = 1.0;  // For regular servos
                    } else {
                        shooter.hingeTargetPower = 0;
                        shooter.hingeTargetPosition = 0;
                    }
                }

                // Flipper control (D-pad Up/Down) - only if flipper attached
                // Flipper is a positional servo
                if (flipperAttached) {
                    if (gamepad2.dpad_up && (shooter.getShooterCurrentRPM() > 550 && shooter.getShooterCurrentRPM() < 650)) {
                        // Set both - class will use the right one based on type
                        shooter.flipperTargetPosition = flipperActivePosition;
                        shooter.flipperTargetPower = flipperActivePower;
                    } else if (gamepad2.dpad_down) {
                        shooter.flipperTargetPosition = flipperIdlePosition;
                        shooter.flipperTargetPower = 0;
                    }
                }

                // Spindexer control (D-pad Left/Right) - only if spindexer attached
                // Spindexer is a continuous rotation servo (left = negative power, right = positive power)
                if (spindexerAttached) {
                    if (gamepad2.dpad_right) {
                        // Spin right - positive power
                        shooter.spindexerTargetPosition = spindexerActivePosition;
                        shooter.spindexerTargetPower = spindexerActivePower;
                    } else if (gamepad2.dpad_left) {
                        // Spin left - negative power
                        shooter.spindexerTargetPosition = spindexerIdlePosition;
                        shooter.spindexerTargetPower = -spindexerActivePower;
                    } else {
                        // No button pressed - stop
                        shooter.spindexerTargetPosition = 0;
                        shooter.spindexerTargetPower = 0;
                    }
                }

                // Update shooter (pass individual enable flags)
                shooter.update(shooterAttached, shooterHingeAttached, magazineAttached, spindexerAttached, flipperAttached);
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

    //TODO: accessing fields without a getter/setter is fine when it is from this class,
    //      so these can be removed.

    public boolean isLastGamepad2A() {
        return lastGamepad2A;
    }

    public void setLastGamepad2A(boolean lastGamepad2A) {
        this.lastGamepad2A = lastGamepad2A;
    }

    public boolean isIntakeIsIn() {
        return intakeIsIn;
    }

    public void setIntakeIsIn(boolean intakeIsIn) {
        this.intakeIsIn = intakeIsIn;
    }

    public boolean isHinge1IsUp() {
        return hinge1IsUp;
    }

    public void setHinge1IsUp(boolean hinge1IsUp) {
        this.hinge1IsUp = hinge1IsUp;
    }

    public boolean isHinge2IsUp() {
        return hinge2IsUp;
    }

    public void setHinge2IsUp(boolean hinge2IsUp) {
        this.hinge2IsUp = hinge2IsUp;
    }
}