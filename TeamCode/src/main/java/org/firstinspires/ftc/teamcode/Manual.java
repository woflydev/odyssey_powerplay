package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class Manual extends OpMode {
    // -------------------------------------------------------------- VAR CONFIG
    private DcMotor backLM = null;
    private DcMotor backRM = null;
    private DcMotor frontLM = null;
    private DcMotor frontRM = null;
    private DcMotorEx armM = null;
    private Servo claw;

    private int currentArmPosition = 0;
    private int targetArmPosition = 0;

    private double current_v1 = 0;
    private double current_v2 = 0;
    private double current_v3 = 0;
    private double current_v4 = 0;

    private boolean ADJUSTMENT_ALLOWED = true;
    private boolean clawOpen = true;

    private static final float MAX_ACCELERATION_DEVIATION = 0.1f; // higher it is, the less smoothing

    // -------------------------------------------------------------- MOTOR CONFIG

    private static final String FRONT_LEFT = "frontL";
    private static final String FRONT_RIGHT = "frontR";
    private static final String BACK_LEFT = "backL";
    private static final String BACK_RIGHT = "backR";
    private static final String SERVO_CLAW = "Servo";
    private static final String ARM_MOTOR = "armMotor";

    private static final double CLAW_CLOSE = 0.62;
    private static final double CLAW_OPEN = 0.43;

    // -------------------------------------------------------------- JUNCTION PRESETS

    private static final int JUNCTION_OFF = 0;
    private static final int JUNCTION_LOW = 1650;
    private static final int JUNCTION_STANDBY = 2000;
    private static final int JUNCTION_MID = 2700;
    private static final int JUNCTION_HIGH = 3800;

    private static final int ARM_ADJUSTMENT_INCREMENT = 10;
    private static final int ARM_BOOST_MODIFIER = 2;

    // -------------------------------------------------------------- MAIN INIT

    private double Stabilize(double new_accel, double current_accel) {
        double dev = new_accel - current_accel;
        return Math.abs(dev) > MAX_ACCELERATION_DEVIATION ? current_accel + MAX_ACCELERATION_DEVIATION * dev / Math.abs(dev) : new_accel;
    }

    private void Move(double power, int timeout, boolean forward) {
        if (forward) {
            frontLM.setPower(-power);
            frontRM.setPower(power);
            backLM.setPower(-power);
            backRM.setPower(power);
        }
        else {
            frontLM.setPower(power);
            frontRM.setPower(-power);
            backLM.setPower(power);
            backRM.setPower(-power);
        }

        try { sleep(timeout); } catch (Exception e) { System.out.println("interrupted"); }

        frontLM.setPower(0);
        frontRM.setPower(0);
        backLM.setPower(0);
        backRM.setPower(0);
    }

    private void Turn(double power, int timeout, boolean right) {
        if (right) {
            frontLM.setPower(power);
            frontRM.setPower(-power);
            backLM.setPower(power);
            backRM.setPower(-power);
        }
        else {
            frontLM.setPower(-power);
            frontRM.setPower(power);
            backLM.setPower(-power);
            backRM.setPower(power);
        }

        try { sleep(timeout); } catch (Exception e) { System.out.println("interrupted"); }

        frontLM.setPower(0);
        frontRM.setPower(0);
        backLM.setPower(0);
        backRM.setPower(0);
    }

    public void init() {
        claw = hardwareMap.get(Servo.class, SERVO_CLAW);
        clawOpen = true;
        claw.setPosition(CLAW_OPEN);

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

    public void loop() {
        // -------------------------------------------------------------- PRELIMINARIES
        telemetry.clear();
        currentArmPosition = armM.getCurrentPosition();

        // -------------------------------------------------------------- MACROS

        if (gamepad1.left_bumper) {
            if (clawOpen) { // obtain cone
                ADJUSTMENT_ALLOWED = false;

                Move(0.9, 250, true); // TODO: should be the length of the arm and front of robot

                claw.setPosition(CLAW_CLOSE); // close claw
                clawOpen = false;
                armM.setTargetPosition(JUNCTION_LOW); // lift cone clear of stack

                Move(0.75, 300, false); // TODO: tune this to clear cone stack
                Turn(0.95, 800, true); // TODO: 180 turn, timeout needs tweaking

                armM.setTargetPosition(JUNCTION_STANDBY);
            }
            else { // drop off cone
                claw.setPosition(CLAW_OPEN); // open claw
                clawOpen = true;

                Move(0.85, 150, false); // TODO: move back, tune timeout

                armM.setTargetPosition(JUNCTION_OFF);

                ADJUSTMENT_ALLOWED = true;
            }
        }

        else if (gamepad1.right_bumper) { // manual close without auto
            if (clawOpen) {
                claw.setPosition(CLAW_CLOSE);
                clawOpen = false;
            }
            else {
                claw.setPosition(CLAW_OPEN);
                clawOpen = true;
            }
        }

        // -------------------------------------------------------------- ARM ADJUSTMENT

        // best used for lining up arm for the topmost cone
        if (gamepad1.dpad_down) {
            targetArmPosition = JUNCTION_OFF;
        }

        if (ADJUSTMENT_ALLOWED) {
            if (gamepad1.b && armM.getCurrentPosition() < 4000 - ARM_ADJUSTMENT_INCREMENT) {
                targetArmPosition += ARM_ADJUSTMENT_INCREMENT;
            }

            else if (gamepad1.a && armM.getCurrentPosition() > ARM_ADJUSTMENT_INCREMENT) {
                targetArmPosition -= ARM_ADJUSTMENT_INCREMENT;
            }
        }

        armM.setVelocity((double)1800 / ARM_BOOST_MODIFIER); armM.setTargetPosition(targetArmPosition);

        // -------------------------------------------------------------- DRIVE

        // assign speed modifier
        int driveSpeedModifier = 1;

        // Mecanum Drive
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.right_stick_x);
        double robotAngle = Math.atan2(- 1 * gamepad1.right_stick_x, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_y;
        final double v1 = r * Math.cos(-robotAngle) + rightX; //back left
        final double v2 = r * Math.sin(robotAngle) - rightX; //front right
        final double v3 = r * Math.sin(robotAngle) + rightX; //front left
        final double v4 = r * Math.cos(-robotAngle) - rightX; //back right

        double stable_v1 = Stabilize(v1, current_v1);
        double stable_v2 = Stabilize(v2, current_v2);
        double stable_v3 = Stabilize(v3, current_v3);
        double stable_v4 = Stabilize(v4, current_v4);

        current_v1 = stable_v1;
        current_v2 = stable_v2;
        current_v3 = stable_v3;
        current_v4 = stable_v4;

        frontLM.setPower(stable_v3 / driveSpeedModifier);
        frontRM.setPower(stable_v2 / driveSpeedModifier);
        backLM.setPower(stable_v1 / driveSpeedModifier);
        backRM.setPower(stable_v4 / driveSpeedModifier);

        // -------------------------------------------------------------- TELEMETRY

        telemetry.addData("Claw Open: ", clawOpen);
        telemetry.addData("Current Position: ", currentArmPosition);
        telemetry.addData("Target Arm Position: ", targetArmPosition);
        telemetry.addData("Arm Adjustment Allowed: ", ADJUSTMENT_ALLOWED);
    }
}
