package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Rebuilt.MainPrograms.MotorPowerRegulator_New;

//TODO: This class should be refactored so that it takes on more responsibility.
//      What Erik and I were thinking is it keeps track of all internal details of the
//      shooter (like motor positions, speeds, etc.) and exposes methods like:
//      .setShootingMode()    close, far, etc.
//      .shoot()              set internal state to shoot when ready
//      .update()             basically the same as is, updates everything behind the scenes

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

    //TODO: Now we know explicitly what types of motors/servos to expect, so
    //      these conditional fields aren't really necessary.
    public Servo hinge1Servo, hinge2Servo;
    public CRServo hinge1CRservo, hinge2CRservo;
    public DcMotor hinge1Motor, hinge2Motor;
    public CRServo magazine1CRServo, magazine2CRServo, magazine3CRServo, magazine4CRServo;
    public Servo magazine1Servo, magazine2Servo, magazine3Servo, magazine4Servo;
    public DcMotor magazine1Motor, magazine2Motor, magazine3Motor, magazine4Motor;

    // New servos for MK3
    public CRServo spindexerCRServo;
    public Servo spindexerServo;
    public DcMotor spindexerMotor;
    public CRServo flipperCRServo;
    public Servo flipperServo;
    public DcMotor flipperMotor;

    public Telemetry telemetry;

    // Control variables - Public - Set these from main teleop
    public double shooterTargetRPM = 0;
    public double hingeTargetPosition = 0;
    public double hingeTargetPower = 0;
    public double magazineTargetPower = 0;      // For CR servos and motors (-1 to 1)
    public double magazineTargetPosition = 0;   // For regular servos (0-1)

    // New control variables for spindexer and flipper
    public double spindexerTargetPower = 0;      // For CR servos and motors (-1 to 1)
    public double spindexerTargetPosition = 0;   // For regular servos (0-1)
    public double flipperTargetPower = 0;        // For CR servos and motors (-1 to 1)
    public double flipperTargetPosition = 0;     // For regular servos (0-1)

    // TODO: Combine these booleans with the 'attached' booleans outside.
    //       Erik thinks the combined booleans should all be in main (only
    //       call update if it's attached). I agree, it will make this class simpler.

    // Component status - Public
    public boolean shooterEnabled = true;
    public boolean hingeEnabled = false;
    public boolean magazineEnabled = false;
    public boolean spindexerEnabled = false;
    public boolean flipperEnabled = false;
    public boolean shooterInitialized = false;
    public boolean hinge1Initialized = false;
    public boolean hinge2Initialized = false;
    public boolean magazine1Initialized = false;
    public boolean magazine2Initialized = false;
    public boolean magazine3Initialized = false;
    public boolean magazine4Initialized = false;
    public boolean spindexerInitialized = false;
    public boolean flipperInitialized = false;
    public boolean magazineInitialized;
    public boolean hingeInitialized;

    // Magazine type tracking - Public - Set from teleop constructor parameters
    public String magazine1Type;
    public String magazine2Type;
    public String magazine3Type;
    public String magazine4Type;
    public String hinge1Type;
    public String hinge2Type;
    public String spindexerType;
    public String flipperType;

    /**
     * Constructor with flexible magazine types and new servos
     * @param initShooter Initialize shooter motor
     * @param initHinge1Type Initialize hinge servo
     * @param initHinge2Type Initialize hinge servo
     * @param initMag1Type "servo", "crservo", "motor", or "none"
     * @param initMag2Type "servo", "crservo", "motor", or "none"
     * @param initMag3Type "servo", "crservo", "motor", or "none"
     * @param initMag4Type "servo", "crservo", "motor", or "none"
     * @param initSpindexerType "servo", "crservo", "motor", or "none"
     * @param initFlipperType "servo", "crservo", "motor", or "none"
     */
    public ShooterClassMK3(HardwareMap hardwareMap, Telemetry telemetry,
                           boolean initShooter, String initHinge1Type, String initHinge2Type,
                           String initMag1Type, String initMag2Type, String initMag3Type, String initMag4Type,
                           String initSpindexerType, String initFlipperType) {
        this.telemetry = telemetry;

        // Initialize shooter motor
        if (initShooter) {
            try {
                shooterMotor = new MotorPowerRegulator_New(hardwareMap, telemetry, "s");
                shooterMotor.setTicksPerRev(112.0);
                shooterMotor.setMaxRpmUnderLoad(1400.0);
                shooterMotor.setAllGains(0.0006785714285714286, 0.06, 0.0004, 0.0002, 0.00005);
                shooterInitialized = true;
            } catch (Exception e) {
                shooterInitialized = false;
                telemetry.addData("Shooter Error", e.getMessage());
            }
        }

        // Initialize hinge servo
        if (!initHinge1Type.equals("none")) {
            try {
                switch (initHinge1Type) {
                    case "servo" -> {
                        hinge1Servo = hardwareMap.get(Servo.class, "hinge1");
                        hinge1Servo.setPosition(0);
                        hinge1Type = "servo";
                        hinge1Initialized = true;
                    }
                    case "crservo" -> {
                        hinge1CRservo = hardwareMap.get(CRServo.class, "hinge1");
                        hinge1CRservo.setPower(0);
                        hinge1Type = "crservo";
                        hinge1Initialized = true;
                    }
                    case "motor" -> {
                        hinge1Motor = hardwareMap.get(DcMotor.class, "hinge1");
                        hinge1Motor.setPower(0);
                        hinge1Type = "motor";
                        hinge1Initialized = true;
                    }
                }
            } catch (Exception e) {
                hinge1Initialized = false;
                telemetry.addData("hinge 1 Error", e.getMessage());
            }
        }
        if (!initHinge2Type.equals("none")) {
            try {
                switch (initHinge2Type) {
                    case "servo" -> {
                        hinge2Servo = hardwareMap.get(Servo.class, "hinge2");
                        hinge2Servo.setPosition(0);
                        hinge2Type = "servo";
                        hinge2Initialized = true;
                    }
                    case "crservo" -> {
                        hinge2CRservo = hardwareMap.get(CRServo.class, "hinge2");
                        hinge2CRservo.setPower(0);
                        hinge2Type = "crservo";
                        hinge2Initialized = true;
                    }
                    case "motor" -> {
                        hinge2Motor = hardwareMap.get(DcMotor.class, "hinge2");
                        hinge2Motor.setPower(0);
                        hinge2Type = "motor";
                        hinge2Initialized = true;
                    }
                }
            } catch (Exception e) {
                hinge2Initialized = false;
                telemetry.addData("hinge 2 Error", e.getMessage());
            }
        }


        // Initialize magazine 1
        if (!initMag1Type.equals("none")) {
            try {
                switch (initMag1Type) {
                    case "servo" -> {
                        magazine1Servo = hardwareMap.get(Servo.class, "mag1");
                        magazine1Servo.setPosition(0);
                        magazine1Type = "servo";
                        magazine1Initialized = true;
                    }
                    case "crservo" -> {
                        magazine1CRServo = hardwareMap.get(CRServo.class, "mag1");
                        magazine1CRServo.setPower(0);
                        magazine1Type = "crservo";
                        magazine1Initialized = true;
                    }
                    case "motor" -> {
                        magazine1Motor = hardwareMap.get(DcMotor.class, "mag1");
                        magazine1Motor.setPower(0);
                        magazine1Type = "motor";
                        magazine1Initialized = true;
                    }
                }
            } catch (Exception e) {
                magazine1Initialized = false;
                telemetry.addData("Magazine 1 Error", e.getMessage());
            }
        }

        // Initialize magazine 2
        if (!initMag2Type.equals("none")) {
            try {
                switch (initMag2Type) {
                    case "servo" -> {
                        magazine2Servo = hardwareMap.get(Servo.class, "mag2");
                        magazine2Servo.setPosition(0);
                        magazine2Type = "servo";
                        magazine2Initialized = true;
                    }
                    case "crservo" -> {
                        magazine2CRServo = hardwareMap.get(CRServo.class, "mag2");
                        magazine2CRServo.setPower(0);
                        magazine2Type = "crservo";
                        magazine2Initialized = true;
                    }
                    case "motor" -> {
                        magazine2Motor = hardwareMap.get(DcMotor.class, "mag2");
                        magazine2Motor.setPower(0);
                        magazine2Type = "motor";
                        magazine2Initialized = true;
                    }
                }
            } catch (Exception e) {
                magazine2Initialized = false;
                telemetry.addData("Magazine 2 Error", e.getMessage());
            }
        }
        if (!initMag3Type.equals("none")) {
            try {
                switch (initMag3Type) {
                    case "servo" -> {
                        magazine3Servo = hardwareMap.get(Servo.class, "mag3");
                        magazine3Servo.setPosition(0);
                        magazine3Type = "servo";
                        magazine3Initialized = true;
                    }
                    case "crservo" -> {
                        magazine3CRServo = hardwareMap.get(CRServo.class, "mag3");
                        magazine3CRServo.setPower(0);
                        magazine3Type = "crservo";
                        magazine3Initialized = true;
                    }
                    case "motor" -> {
                        magazine3Motor = hardwareMap.get(DcMotor.class, "mag3");
                        magazine3Motor.setPower(0);
                        magazine3Type = "motor";
                        magazine3Initialized = true;
                    }
                }
            } catch (Exception e) {
                magazine3Initialized = false;
                telemetry.addData("Magazine 3 Error", e.getMessage());
            }
        }
        if (!initMag4Type.equals("none")) {
            try {
                switch (initMag4Type) {
                    case "servo" -> {
                        magazine4Servo = hardwareMap.get(Servo.class, "mag4");
                        magazine4Servo.setPosition(0);
                        magazine4Type = "servo";
                        magazine4Initialized = true;
                    }
                    case "crservo" -> {
                        magazine4CRServo = hardwareMap.get(CRServo.class, "mag4");
                        magazine4CRServo.setPower(0);
                        magazine4Type = "crservo";
                        magazine4Initialized = true;
                    }
                    case "motor" -> {
                        magazine4Motor = hardwareMap.get(DcMotor.class, "mag4");
                        magazine4Motor.setPower(0);
                        magazine4Type = "motor";
                        magazine4Initialized = true;
                    }
                }
            } catch (Exception e) {
                magazine4Initialized = false;
                telemetry.addData("Magazine 4 Error", e.getMessage());
            }
        }

        // Initialize spindexer (hardware name: "spin")
        if (!initSpindexerType.equals("none")) {
            try {
                switch (initSpindexerType) {
                    case "servo" -> {
                        spindexerServo = hardwareMap.get(Servo.class, "spin");
                        spindexerServo.setPosition(0);
                        spindexerType = "servo";
                        spindexerInitialized = true;
                    }
                    case "crservo" -> {
                        spindexerCRServo = hardwareMap.get(CRServo.class, "spin");
                        spindexerCRServo.setPower(0);
                        spindexerType = "crservo";
                        spindexerInitialized = true;
                    }
                    case "motor" -> {
                        spindexerMotor = hardwareMap.get(DcMotor.class, "spin");
                        spindexerMotor.setPower(0);
                        spindexerType = "motor";
                        spindexerInitialized = true;
                    }
                }
            } catch (Exception e) {
                spindexerInitialized = false;
                telemetry.addData("Spindexer Error", e.getMessage());
            }
        }

        // Initialize flipper (hardware name: "flipper")
        if (!initFlipperType.equals("none")) {
            try {
                switch (initFlipperType) {
                    case "servo" -> {
                        flipperServo = hardwareMap.get(Servo.class, "flipper");
                        flipperServo.setPosition(0);
                        flipperType = "servo";
                        flipperInitialized = true;
                    }
                    case "crservo" -> {
                        flipperCRServo = hardwareMap.get(CRServo.class, "flipper");
                        flipperCRServo.setPower(0);
                        flipperType = "crservo";
                        flipperInitialized = true;
                    }
                    case "motor" -> {
                        flipperMotor = hardwareMap.get(DcMotor.class, "flipper");
                        flipperMotor.setPower(0);
                        flipperType = "motor";
                        flipperInitialized = true;
                    }
                }
            } catch (Exception e) {
                flipperInitialized = false;
                telemetry.addData("Flipper Error", e.getMessage());
            }
        }

        magazineInitialized = magazine1Initialized || magazine2Initialized || magazine3Initialized || magazine4Initialized;
        hingeInitialized = hinge1Initialized || hinge2Initialized;
    }

    //TODO: Add methods to modify the shooting mode and to set the current goal to shoot
    //      (or whatever you decide to do here.)

    /**
     * Main update - reads public variables and applies them
     * FIXED: Always calls loop() to keep RPM updated, even when target is 0
     */
    public void update(boolean shooterEnabled, boolean hingeEnabled, boolean magazineEnabled,
                       boolean spindexerEnabled, boolean flipperEnabled) {
        this.shooterEnabled = shooterEnabled;
        this.hingeEnabled = hingeEnabled;
        this.magazineEnabled = magazineEnabled;
        this.spindexerEnabled = spindexerEnabled;
        this.flipperEnabled = flipperEnabled;

        // Update shooter motor - ALWAYS call loop() to keep RPM measurement updated
        if (shooterEnabled && shooterInitialized) {
            shooterMotor.setTargetRPM(shooterTargetRPM);
            shooterMotor.loop();  // Always call loop to update RPM

            // If target is 0, force power to 0 after loop() runs
            if (shooterTargetRPM <= 0) {
                shooterMotor.getMotor().setPower(0);
            }
        } else if (shooterInitialized) {
            shooterMotor.setTargetRPM(0);
            shooterMotor.loop();  // Keep updating RPM even when disabled
            shooterMotor.getMotor().setPower(0);
        }

        // Update hinge
        if (hingeEnabled && hinge1Initialized) {
            switch (hinge1Type) {
                case "servo" -> hinge1Servo.setPosition(hingeTargetPosition);
                case "crservo" -> hinge1CRservo.setPower(hingeTargetPower);
                case "motor" -> hinge1Motor.setPower(hingeTargetPower);
            }
        } else if (hinge1Initialized) {
            stophinge1();
        }
        if (hingeEnabled && hinge2Initialized) {
            switch (hinge2Type) {
                case "servo" -> hinge2Servo.setPosition(hingeTargetPosition);
                case "crservo" -> hinge2CRservo.setPower(hingeTargetPower);
                case "motor" -> hinge2Motor.setPower(hingeTargetPower);
            }
        } else if (hinge2Initialized) {
            stophinge2();
        }

        // Update magazine 1
        if (magazineEnabled && magazine1Initialized) {
            switch (magazine1Type) {
                case "servo" -> magazine1Servo.setPosition(magazineTargetPosition);
                case "crservo" -> magazine1CRServo.setPower(magazineTargetPower);
                case "motor" -> magazine1Motor.setPower(magazineTargetPower);
            }
        } else if (magazine1Initialized) {
            stopMagazine1();
        }

        // Update magazine 2
        if (magazineEnabled && magazine2Initialized) {
            switch (magazine2Type) {
                case "servo" -> magazine2Servo.setPosition(magazineTargetPosition);
                case "crservo" -> magazine2CRServo.setPower(magazineTargetPower);
                case "motor" -> magazine2Motor.setPower(magazineTargetPower);
            }
        } else if (magazine2Initialized) {
            stopMagazine2();
        }
        if (magazineEnabled && magazine3Initialized) {
            switch (magazine3Type) {
                case "servo" -> magazine3Servo.setPosition(magazineTargetPosition);
                case "crservo" -> magazine3CRServo.setPower(magazineTargetPower);
                case "motor" -> magazine3Motor.setPower(magazineTargetPower);
            }
        } else if (magazine3Initialized) {
            stopMagazine3();
        }
        if (magazineEnabled && magazine4Initialized) {
            switch (magazine4Type) {
                case "servo" -> magazine4Servo.setPosition(magazineTargetPosition);
                case "crservo" -> magazine4CRServo.setPower(magazineTargetPower);
                case "motor" -> magazine4Motor.setPower(magazineTargetPower);
            }
        } else if (magazine4Initialized) {
            stopMagazine4();
        }

        // Update spindexer
        if (spindexerEnabled && spindexerInitialized) {
            switch (spindexerType) {
                case "servo" -> spindexerServo.setPosition(spindexerTargetPosition);
                case "crservo" -> spindexerCRServo.setPower(spindexerTargetPower);
                case "motor" -> spindexerMotor.setPower(spindexerTargetPower);
            }
        } else if (spindexerInitialized) {
            stopSpindexer();
        }

        // Update flipper
        if (flipperEnabled && flipperInitialized) {
            switch (flipperType) {
                case "servo" -> flipperServo.setPosition(flipperTargetPosition);
                case "crservo" -> flipperCRServo.setPower(flipperTargetPower);
                case "motor" -> flipperMotor.setPower(flipperTargetPower);
            }
        } else if (flipperInitialized) {
            stopFlipper();
        }
    }

    /**
     * Stop magazine 1
     */
    public void stopMagazine1() {
        switch (magazine1Type) {
            case "servo" -> magazine1Servo.setPosition(0);
            case "crservo" -> magazine1CRServo.setPower(0);
            case "motor" -> magazine1Motor.setPower(0);
        }
    }

    /**
     * Stop magazine 2
     */
    public void stopMagazine2() {
        switch (magazine2Type) {
            case "servo" -> magazine2Servo.setPosition(0);
            case "crservo" -> magazine2CRServo.setPower(0);
            case "motor" -> magazine2Motor.setPower(0);
        }
    }
    public void stopMagazine3() {
        switch (magazine3Type) {
            case "servo" -> magazine3Servo.setPosition(0);
            case "crservo" -> magazine3CRServo.setPower(0);
            case "motor" -> magazine3Motor.setPower(0);
        }
    }
    public void stopMagazine4() {
        switch (magazine4Type) {
            case "servo" -> magazine4Servo.setPosition(0);
            case "crservo" -> magazine4CRServo.setPower(0);
            case "motor" -> magazine4Motor.setPower(0);
        }
    }
    public void stophinge1() {
        switch (hinge1Type) {
            case "servo" -> hinge1Servo.setPosition(0);
            case "crservo" -> hinge1CRservo.setPower(0);
            case "motor" -> hinge1Motor.setPower(0);
        }
    }
    public void stophinge2() {
        switch (hinge2Type) {
            case "servo" -> hinge2Servo.setPosition(0);
            case "crservo" -> hinge2CRservo.setPower(0);
            case "motor" -> hinge2Motor.setPower(0);
        }
    }

    /**
     * Stop spindexer
     */
    public void stopSpindexer() {
        switch (spindexerType) {
            case "servo" -> spindexerServo.setPosition(0);
            case "crservo" -> spindexerCRServo.setPower(0);
            case "motor" -> spindexerMotor.setPower(0);
        }
    }

    /**
     * Stop flipper
     */
    public void stopFlipper() {
        switch (flipperType) {
            case "servo" -> flipperServo.setPosition(0);
            case "crservo" -> flipperCRServo.setPower(0);
            case "motor" -> flipperMotor.setPower(0);
        }
    }

    // ==================== GETTERS ====================

    //TODO: After refactor, these will probably be unnecessary. Remove them if
    //      they are. Generally, you only need getters/setters if it is necessary
    //      to expose something internal.

    public boolean getShooterInitialized() { return shooterInitialized; }
    public boolean getMagazineInitialized() { return magazineInitialized; }
    public boolean getMagazine1Initialized() { return magazine1Initialized; }
    public boolean getMagazine2Initialized() { return magazine2Initialized; }
    public boolean getMagazine3Initialized() { return magazine3Initialized; }
    public boolean getMagazine4Initialized() { return magazine4Initialized; }
    public boolean getSpindexerInitialized() { return spindexerInitialized; }
    public boolean getFlipperInitialized() { return flipperInitialized; }
    public boolean getShooterEnabled() { return shooterEnabled; }
    public boolean getHinge1Initialized() { return hinge1Initialized; }
    public boolean getHinge2Initialized() { return hinge2Initialized; }

    public boolean getMagazineEnabled() { return magazineEnabled; }
    public boolean getSpindexerEnabled() { return spindexerEnabled; }
    public boolean getFlipperEnabled() { return flipperEnabled; }

    public double getShooterCurrentRPM() {
        return shooterMotor != null ? shooterMotor.getCurrentRPM() : 0;
    }

    public boolean getShooterAtTarget(double tolerance) {
        return shooterMotor != null && shooterMotor.isAtTarget(tolerance);
    }
}