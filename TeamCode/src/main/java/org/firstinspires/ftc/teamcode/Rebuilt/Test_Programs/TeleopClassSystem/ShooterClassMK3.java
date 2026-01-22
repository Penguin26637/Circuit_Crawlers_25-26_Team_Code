package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Rebuilt.MainPrograms.MotorPowerRegulator_New;

import Math

/**
 * ShooterClassMK3 - Handles shooter logic
 * Supports flexible magazine: up to 4, each can be Servo, CRServo, or DcMotor
 * Added spindexer and flipper servos
 * All variables are public and controlled from main teleop

 * FIXED: Always calls loop() to keep RPM updated even when target is 0
 */
public class ShooterClassMK3 {

    // Hardware - Public - Only initialized ones will be non-null
    public MotorPowerRegulator_New shooterMotor;
    private static final double SHOOTER_KV = 0.0006785714285714286;
    private static final double SHOOTER_KS = 0.06;
    private static final double SHOOTER_KP = 0.0004;
    private static final double SHOOTER_KI = 0.0002;
    private static final double SHOOTER_KD = 0.00005;

    //TODO: Now we know explicitly what types of motors/servos to expect, so
    //      these conditional fields aren't really necessary.

    // New servos for MK3
    private CRServo spindexerCRServo;
    private Servo flipperServo;
    private Telemetry telemetry;

    // Control variables - Public - Set these from main teleop
    private double shooterTargetRPM = 0;

    // TODO: instead of exposing spindexerTargetPower, maybe factor this into a method...
    // New control variables for spindexer and flipper
    public double spindexerTargetPower = 0;      // For CR servos and motors (-1 to 1)
    private double flipperTargetPosition = 0;     // For regular servos (0-1)

    // TODO: Combine these booleans with the 'attached' booleans outside.
    //       Erik thinks the combined booleans should all be in main (only
    //       call update if it's attached). I agree, it will make this class simpler.

    /**
     * Constructor taking servos of spindexer, flipper
     * @param spindexerCRServo continuous rotational servo for the spindexer
     * @param flipperServo positional servo to control flipper
     * @param telemetry overall telemetry object
     * @param hardwareMap hardware map for the robot needed for MotorPowerRegulator
     */
    public ShooterClassMK3(CRServo spindexerCRServo, Servo flipperServo, Telemetry telemetry, HardwareMap hardwareMap) {
        this.spindexerCRServo = spindexerCRServo;
        this.flipperServo = flipperServo;
        this.telemetry = telemetry;
        
        // TODO: refactor MotorPowerRegulator so that it takes a motor rather than the hardware map
        shooterMotor = new MotorPowerRegulator_New(hardwareMap, telemetry, "s");
        shooterMotor.setTicksPerRev(112.0);
        shooterMotor.setMaxRpmUnderLoad(1400.0);
        shooterMotor.setAllGains(SHOOTER_KV, SHOOTER_KS, SHOOTER_KP, SHOOTER_KI, SHOOTER_KD);
    }
    
    public enum ShootingMode {
        NONE,
        CLOSE,
        FAR,
        SUPER_FAR
    }

    public setShootingMode(ShootingMode choice) {
        if (choice == ShootingMode.CLOSE) {
            shooterMotor.setTargetRPM(200.0);
        }
        else if (choice == ShootingMode.FAR) {
            shooterMotor.setTargetRPM(400.0);
        } 
        else if (choice == ShootingMode.SUPER_FAR) {
            shooterMotor.setTargetRPM(600.0);
        }
        else { //NONE
            shooterMotor.setTargetRPM(0.0);
        }
    }

    private boolean shootingMode = false;

    public shoot() {
        shootingMode = true;
    }

    // TODO: Don't do this, this is dumb. I don't have time to come up with something better.
    public unshoot() {
        shootingMode = false;
    }

    /**
     * Main update - reads public variables and applies them
     * FIXED: Always calls loop() to keep RPM updated, even when target is 0
     */
    public void update() {

        // Update shooter motor - ALWAYS call loop() to keep RPM measurement updated
        shooterMotor.loop();  // Always call loop to update RPM

        // Update flipper
        double LIMIT = 0.05;
        double SHOOT_POSITION = 0.2;
        double REST_POSITION = 0.1;
        if (shootingMode && Math.abs(shooterMotor.lastError) < LIMIT) {
            flipperServo.setPosition(SHOOT_POSITION);
        } else if (!shootingMode) {
            flipperServo.setPosition(REST_POSITION);
        }

        // Update spindexer
        spindexerCRServo.setPower(spindexerTargetPower);
    }
}