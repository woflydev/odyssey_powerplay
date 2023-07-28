package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class Slide extends OpMode {

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor frontLMotor = null;
    DcMotor frontRMotor = null;
    DcMotorEx ArmMotor = null;
    Servo claw;
    boolean clawOpen = false;

    int setPos;

    //INTRODUCE VARIABLES HERE

    public void init() {
        claw = hardwareMap.get(Servo.class, "Servo");

        leftMotor = hardwareMap.get(DcMotor.class, "backL");
        rightMotor = hardwareMap.get(DcMotor.class, "backR");

        frontLMotor = hardwareMap.get(DcMotor.class, "frontL");
        frontRMotor = hardwareMap.get(DcMotor.class, "frontR");
        frontRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ArmMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setTargetPosition(0);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setAutoClear(false);
    }

    public void loop() {
        telemetry.clear();
        if (setPos < 4000) {
            setPos += 8 * Math.round(-gamepad2.left_stick_y); // adds value of joystick
        }
        int ArmPos0 = 0;
        int ArmPos1 = 1650;
        int ArmPos2 = 2700;
        int ArmPos3 = 3800;

        // Claw Code
        if(gamepad2.left_bumper) {
            // closed
            clawOpen = false;
            claw.setPosition(1);
        } else if (gamepad2.right_bumper) {
            // open
            clawOpen = true;
            claw.setPosition(0.43);
        }
        telemetry.addData("Claw Open:", clawOpen);

        int speedModA = 1;
        if (gamepad2.x) {
            speedModA = 2;
        }

        int slidePos = ArmMotor.getCurrentPosition();
        ArmMotor.setVelocity(1800 / speedModA);
        telemetry.addData("Current Position", slidePos);
        if (gamepad2.dpad_down) {
            setPos = ArmPos0;
            telemetry.addData("target pos", setPos);
        } else if (gamepad2.dpad_left) {
            setPos = ArmPos1;
            telemetry.addData("target pos", setPos);
        } else if (gamepad2.dpad_right) {
            setPos = ArmPos2;
            telemetry.addData("target pos", setPos);
        } else if (gamepad2.dpad_up) {
            setPos = ArmPos3;
            telemetry.addData("target pos", setPos);
        }

        telemetry.addData("Setpos", setPos);

        ArmMotor.setTargetPosition(setPos); // joystick position or key

        // Drive --------------------------------------------------------------------
        // assign speed modifier
        int speedModB = 2;

        if (gamepad1.right_bumper) {
            speedModB = 1;
        }
        if (gamepad1.left_bumper) {
            speedModB = 3;
        }

        // Mecanum Drive
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.right_stick_x);
        double robotAngle = Math.atan2(- 1 * gamepad1.right_stick_x, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_y;
        final double v1 = r * Math.cos(-robotAngle) + rightX; //back left
        final double v2 = r * Math.sin(robotAngle) - rightX; //front right
        final double v3 = r * Math.sin(robotAngle) + rightX; //front left
        final double v4 = r * Math.cos(-robotAngle) - rightX; //back right

        frontLMotor.setPower(v3 / speedModB);
        frontRMotor.setPower(v2 / speedModB);
        leftMotor.setPower(v1 / speedModB);
        rightMotor.setPower(v4 / speedModB);
    }
}
