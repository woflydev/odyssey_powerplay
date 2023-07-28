package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class Manual extends OpMode {

    private DcMotor backLM = null;
    private DcMotor backRM = null;
    private DcMotor frontLM = null;
    private DcMotor frontRM = null;
    private DcMotorEx armM = null;
    private Servo claw;
    private boolean clawOpen = false;

    private int setPos;

    private static final float MAX_ACCELERATION_DEVIATION = 0.1f;

    // --------------------------------------------------------------

    private static final String FRONT_LEFT = "frontL";
    private static final String FRONT_RIGHT = "frontR";
    private static final String BACK_LEFT = "backL";
    private static final String BACK_RIGHT = "backR";
    private static final String SERVO_CLAW = "Servo";
    private static final String ARM_MOTOR = "armMotor";

    // --------------------------------------------------------------

    public void init() {
        claw = hardwareMap.get(Servo.class, SERVO_CLAW);

        backLM = hardwareMap.get(DcMotor.class, BACK_LEFT);
        backRM = hardwareMap.get(DcMotor.class, BACK_RIGHT);

        frontLM = hardwareMap.get(DcMotor.class, FRONT_LEFT);
        frontRM = hardwareMap.get(DcMotor.class, FRONT_RIGHT); frontRM.setDirection(DcMotorSimple.Direction.REVERSE);

        armM = hardwareMap.get(DcMotorEx.class, ARM_MOTOR);
        armM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armM.setTargetPosition(0);
        armM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setAutoClear(false);
    }

    private double stabilize(double new_accel, double current_accel, double max_dev) {
        double dev = new_accel - current_accel;
        return Math.abs(dev) > max_dev ? current_accel + max_dev * dev / Math.abs(dev) : new_accel;
    }

    float currentAccel1;
    float currentAccel2;
    float currentAccel3;
    float currentAccel4;

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
        if (gamepad2.left_bumper) {
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

        int slidePos = armM.getCurrentPosition();
        armM.setVelocity((double)1800 / speedModA);
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

        armM.setTargetPosition(setPos); // joystick position or key

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

        double stable_v1 = stabilize(v1, currentAccel1, MAX_ACCELERATION_DEVIATION);
        double stable_v2 = stabilize(v2, currentAccel2, MAX_ACCELERATION_DEVIATION);
        double stable_v3 = stabilize(v3, currentAccel3, MAX_ACCELERATION_DEVIATION);
        double stable_v4 = stabilize(v4, currentAccel4, MAX_ACCELERATION_DEVIATION);

        frontLM.setPower(stable_v3 / speedModB);
        frontRM.setPower(stable_v2 / speedModB);
        backLM.setPower(stable_v1 / speedModB);
        backRM.setPower(stable_v4 / speedModB);
    }
}
