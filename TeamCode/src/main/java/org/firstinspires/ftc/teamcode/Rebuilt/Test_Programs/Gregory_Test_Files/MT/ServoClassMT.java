package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Represents an FTC servo that can be positioned at any value.
 * Supports both standard Servo and CRServo types.
 */
class ServoClassMT {
    public enum ServoType {
        STANDARD_SERVO,
        CONTINUOUS_SERVO
    }

    private String hardwareMapName;
    private ServoType type;
    private Servo servo;           // For standard servos
    private CRServo crServo;       // For continuous rotation servos

    public ServoClassMT(String name, ServoType type) {
        this.hardwareMapName = name;
        this.type = type;
    }

    /**
     * Initializes the servo hardware from the hardwareMap.
     */
    public void init(HardwareMap hwMap) {
        if (type == ServoType.STANDARD_SERVO) {
            servo = hwMap.get(Servo.class, hardwareMapName);
        } else {
            crServo = hwMap.get(CRServo.class, hardwareMapName);
        }
    }

    /**
     * Sets the servo to a specific position (0.0 to 1.0) or power (-1.0 to 1.0 for CRServo).
     * @param position The position/power to set
     */
    public void goToPosition(double position) {
        if (type == ServoType.STANDARD_SERVO && servo != null) {
            servo.setPosition(position);
        } else if (type == ServoType.CONTINUOUS_SERVO && crServo != null) {
            crServo.setPower(position);
        }
    }

    /**
     * Stops a continuous rotation servo (sets power to 0).
     * Has no effect on standard servos.
     */
    public void stop() {
        if (type == ServoType.CONTINUOUS_SERVO && crServo != null) {
            crServo.setPower(0.0);
        }
    }

    /**
     * Gets the current position (standard servo only).
     * @return Current position, or -1 if not a standard servo
     */
    public double getCurrentPosition() {
        if (type == ServoType.STANDARD_SERVO && servo != null) {
            return servo.getPosition();
        }
        return -1;
    }

    // Getters
    public String getName() { return hardwareMapName; }
    public ServoType getType() { return type; }
    public Servo getServo() { return servo; }
    public CRServo getCRServo() { return crServo; }
}