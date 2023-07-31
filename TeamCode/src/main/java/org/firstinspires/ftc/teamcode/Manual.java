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
    // -------------------------------------------------------------- SYSTEM VAR
    private DcMotorEx backLM = null;
    private DcMotorEx backRM = null;
    private DcMotorEx frontLM = null;
    private DcMotorEx frontRM = null;
    private DcMotorEx armM = null;
    private Servo claw;

    private int targetArmPosition = 0;

    private double current_v1 = 0;
    private double current_v2 = 0;
    private double current_v3 = 0;
    private double current_v4 = 0;

    private boolean ADJUSTMENT_ALLOWED = true;
    private boolean clawOpen = true;

    // -------------------------------------------------------------- ROBOT CONFIG

    private static final String FRONT_LEFT = "frontL";
    private static final String FRONT_RIGHT = "frontR";
    private static final String BACK_LEFT = "backL";
    private static final String BACK_RIGHT = "backR";
    private static final String SERVO_CLAW = "Servo";
    private static final String ARM_MOTOR = "armMotor";

    private static final double CLAW_CLOSE = 0.62;
    private static final double CLAW_OPEN = 0.43;

    private static final int ARM_ADJUSTMENT_INCREMENT = 45;
    private static final int ARM_BOOST_MODIFIER = 1;

    private static final boolean LEFT_STACK = false; // if left, must turn right

    //private static final double ENCODER_TICKS = 537.7; // gobuilda motor 85203 Series
    //private static final double DRIVE_SPEED_MODIFIER = 1; // formula: ENCODER_TICKS * BASE_SPEED ticks per sec. 1 means motor is spinning 1 time per sec.
    private static final double MAX_ACCELERATION_DEVIATION = 0.2; // higher = less smoothing

    // -------------------------------------------------------------- JUNCTION PRESETS

    private static final int JUNCTION_OFF = 0;
    private static final int JUNCTION_LOW = 1650;
    private static final int JUNCTION_MID = 2700;
    private static final int JUNCTION_STANDBY = 3200;
    private static final int JUNCTION_HIGH = 4100;

    // -------------------------------------------------------------- MAIN INIT

    private void Mecanum() {
        // assign speed modifier
        int driveSpeedModifier = 1;

        // mecanum
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
    }

    private void MotorMode(boolean auto) {
        if (auto) {
            backLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // motor tries to use encoder to run at constant velocity
            backRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        else {
            backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void Delay(double time) {
        try { sleep((long)time); } catch (Exception e) { System.out.println("interrupted"); }
    }

    private double Stabilize(double new_accel, double current_accel) {
        double dev = new_accel - current_accel;
        return Math.abs(dev) > MAX_ACCELERATION_DEVIATION ? current_accel + MAX_ACCELERATION_DEVIATION * dev / Math.abs(dev) : new_accel;
    }

    private void Move(double power, int timeout, boolean forward) {
        // going at .5 power will take twice as long, hence timeout / power = distance
        timeout /= timeout > 0 ? power : 1;

        if (forward) {
            frontLM.setPower(-power);
            frontRM.setPower(power);
            backLM.setPower(-power);
            backRM.setPower(power);

            /*frontLM.setVelocity(-power);
            frontRM.setVelocity(power);
            backLM.setVelocity(-power);
            backRM.setVelocity(power);*/
        }
        else {
            frontLM.setPower(power);
            frontRM.setPower(-power);
            backLM.setPower(power);
            backRM.setPower(-power);

            /*frontLM.setVelocity(power);
            frontRM.setVelocity(-power);
            backLM.setVelocity(power);
            backRM.setVelocity(-power);*/
        }

        try { sleep(timeout); } catch (Exception e) { System.out.println("interrupted"); }

        frontLM.setPower(0);
        frontRM.setPower(0);
        backLM.setPower(0);
        backRM.setPower(0);

        /*frontLM.setVelocity(0);
        frontRM.setVelocity(0);
        backLM.setVelocity(0);
        backRM.setVelocity(0);*/
    }

    private void Turn(double power, int timeout, boolean right) {
        timeout /= timeout > 0 ? power : 1;

        if (right) {
            frontLM.setPower(-power); // left motors are inverted
            frontRM.setPower(-power);
            backLM.setPower(-power);
            backRM.setPower(-power);

            /*frontLM.setVelocity(-power);
            frontRM.setVelocity(-power);
            backLM.setVelocity(-power);
            backRM.setVelocity(-power);*/
        }
        else {
            frontLM.setPower(power); // see above
            frontRM.setPower(power);
            backLM.setPower(power);
            backRM.setPower(power);

            /*frontLM.setVelocity(power);
            frontRM.setVelocity(power);
            backLM.setVelocity(power);
            backRM.setVelocity(power);*/
        }

        try { sleep(timeout); } catch (Exception e) { System.out.println("interrupted"); }

        frontLM.setPower(0);
        frontRM.setPower(0);
        backLM.setPower(0);
        backRM.setPower(0);

        /*frontLM.setVelocity(0);
        frontRM.setVelocity(0);
        backLM.setVelocity(0);
        backRM.setVelocity(0);*/
    }

    public void init() {
        claw = hardwareMap.get(Servo.class, SERVO_CLAW);
        clawOpen = true;
        claw.setPosition(CLAW_OPEN);

        backLM = hardwareMap.get(DcMotorEx.class, BACK_LEFT);
        backRM = hardwareMap.get(DcMotorEx.class, BACK_RIGHT);

        frontLM = hardwareMap.get(DcMotorEx.class, FRONT_LEFT);
        frontRM = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT); frontRM.setDirection(DcMotorSimple.Direction.REVERSE); // weird workaround Stanley put in

        backLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

        // -------------------------------------------------------------- MACROS

        if (gamepad1.left_bumper) {
            /*if (clawOpen) { // obtain cone
                MotorMode(true);
                ADJUSTMENT_ALLOWED = false;

                Move(0.5, 180, true); // TODO: should be the length of the arm and front of robot

                claw.setPosition(CLAW_CLOSE); // close claw
                clawOpen = false;

                Delay(300); // claw needs time to close

                armM.setVelocity((double)2300 / ARM_BOOST_MODIFIER);
                armM.setTargetPosition(JUNCTION_HIGH); targetArmPosition = JUNCTION_HIGH;

                Delay(400);

                Move(0.9, 650, false); // TODO: tune this to clear cone stack
                Turn(0.95, 1050, true); // TODO: 180 turn, timeout needs tweaking
                MotorMode(false);
            }*/

            if (clawOpen) { // obtain cone
                MotorMode(true);
                ADJUSTMENT_ALLOWED = false;

                Move(0.5, 180, true); // TODO: should be the length of the arm and front of robot

                claw.setPosition(CLAW_CLOSE); // close claw
                clawOpen = false;

                Delay(300); // claw needs time to close

                armM.setVelocity((double)2300 / ARM_BOOST_MODIFIER);
                armM.setTargetPosition(JUNCTION_HIGH); targetArmPosition = JUNCTION_HIGH;

                Delay(400);

                Move(0.9, 1000, false);
                Turn(0.95, 525, LEFT_STACK); // turns left
                MotorMode(false);
            }

            else { // drop off cone
                MotorMode(true);
                claw.setPosition(CLAW_OPEN); // open claw
                clawOpen = true;

                Delay(350); // need time for cone to drop

                Move(0.85, 150, false); // TODO: move back, tune timeout

                armM.setVelocity((double)2300 / ARM_BOOST_MODIFIER);
                armM.setTargetPosition(JUNCTION_STANDBY); targetArmPosition = JUNCTION_STANDBY;

                Turn(0.95, 1050, false);

                ADJUSTMENT_ALLOWED = true;
                MotorMode(false);
            }
        }

        else if (gamepad1.right_bumper) { // manual close without auto
            if (clawOpen) {
                claw.setPosition(CLAW_CLOSE);
                clawOpen = false;
                Delay(200); // needs delay to register button press
            }
            else {
                claw.setPosition(CLAW_OPEN);
                clawOpen = true;
                Delay(200);
            }
        }

        // -------------------------------------------------------------- ARM ADJUSTMENT

        // best used for lining up arm for the topmost cone
        if (gamepad1.dpad_down) {
            targetArmPosition = JUNCTION_OFF;
        }

        else if (gamepad1.dpad_up) {
            targetArmPosition = JUNCTION_HIGH;
        }

        if (gamepad1.dpad_right) {
            MotorMode(true);
            Move(0.5, 500, true);
            Turn(0.5, 500, true);
            MotorMode(false);
        }

        if (ADJUSTMENT_ALLOWED) {
            if (gamepad1.b && armM.getCurrentPosition() < 4000 - ARM_ADJUSTMENT_INCREMENT) {
                targetArmPosition += ARM_ADJUSTMENT_INCREMENT;
            }

            else if (gamepad1.a && armM.getCurrentPosition() > ARM_ADJUSTMENT_INCREMENT) {
                targetArmPosition -= ARM_ADJUSTMENT_INCREMENT;
            }
        }

        armM.setVelocity((double)2300 / ARM_BOOST_MODIFIER); armM.setTargetPosition(targetArmPosition); //velocity was 1800

        // -------------------------------------------------------------- DRIVE

        Mecanum();

        // -------------------------------------------------------------- TELEMETRY

        telemetry.addData("Claw Open: ", clawOpen);
        telemetry.addData("Current Position: ", armM.getCurrentPosition());
        telemetry.addData("Target Arm Position: ", targetArmPosition);
        telemetry.addData("Arm Adjustment Allowed: ", ADJUSTMENT_ALLOWED);
        telemetry.update();
    }
}
