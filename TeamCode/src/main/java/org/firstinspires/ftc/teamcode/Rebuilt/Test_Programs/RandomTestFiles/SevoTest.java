package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.RandomTestFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SevoTest", group = "Main")
@Config
public class SevoTest extends LinearOpMode {

    private Servo s6, s5;
    private CRServo s1, s2, s3, s4;

    /* ===== Dashboard Tunables ===== */
    public static double CR_POWER = 0.5;
    public static double SERVO_POS_0 = 0.5;
    public static double SERVO_POS_1 = 1.0;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s1 = hardwareMap.get(CRServo.class, "mag1");
        s2 = hardwareMap.get(CRServo.class, "mag2");
        s3 = hardwareMap.get(CRServo.class, "mag3");
        s4 = hardwareMap.get(CRServo.class, "mag4");
        s5 = hardwareMap.get(Servo.class, "hinge1");
        s6 = hardwareMap.get(Servo.class, "hinge2");
//
//        s5.setPosition(SERVO_POS_0);
//        s2.setPosition(SERVO_POS_0);
//        s3.setPosition(SERVO_POS_0);

        waitForStart();

        while (opModeIsActive()) {

            s1.setPower(CR_POWER);
            s4.setPower(CR_POWER);
//            s6.setPower(CR_POWER);
//            s5.setPower(CR_POWER);
            s2.setPower(CR_POWER);
            s3.setPower(CR_POWER);

            if (gamepad1.a) {
//                s5.setPosition(SERVO_POS_0);
//                s2.setPosition(SERVO_POS_0);
//                s3.setPosition(SERVO_POS_0);
//                s4.setPosition(SERVO_POS_0);
                s6.setPosition(SERVO_POS_0);
                s5.setPosition(SERVO_POS_0);
                telemetry.addLine("Button A → 0");
            }

            if (gamepad1.b) {
//                s5.setPosition(SERVO_POS_1);
//                s2.setPosition(SERVO_POS_1);
//                s3.setPosition(SERVO_POS_1);
//                s4.setPosition(SERVO_POS_1);
                s5.setPosition(SERVO_POS_1);
                s6.setPosition(SERVO_POS_1);

                telemetry.addLine("Button B → 1");
            }

//            telemetry.addData("s2 position", s2.getPosition());
//            telemetry.addData("s3 position", s3.getPosition());
//            telemetry.addData("s5 position", s5.getPosition());
            telemetry.addData("s6 Position", s6.getPosition());
            telemetry.addData("s4 power", s4.getPower());
            telemetry.addData("s1 power", s1.getPower());
            telemetry.addData("s2 power", s2.getPower());
            telemetry.addData("s3 power", s3.getPower());
            telemetry.addData("s5 power", s5.getPosition());

            telemetry.update();
            sleep(20);
        }
    }
}
