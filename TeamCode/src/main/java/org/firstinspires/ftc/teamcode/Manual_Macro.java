package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp()
public class Manual_Macro extends OpMode {
    // -------------------------------------------------------------- SYSTEM VAR
    private DcMotorEx backLM = null;
    private DcMotorEx backRM = null;
    private DcMotorEx frontLM = null;
    private DcMotorEx frontRM = null;
    private DcMotorEx armM = null;
    private IMU imu = null;
    private Servo claw;

    private final ElapsedTime encoderRuntime = new ElapsedTime();
    private final ElapsedTime armRuntime = new ElapsedTime();

    private int targetArmPosition = 0;
    private int runtimeArmMinimum = 0;
    private boolean armCanReset = false;
    private boolean clawOpen = true;

    private boolean fieldCentricRed = true;

    private double current_v1 = 0;
    private double current_v2 = 0;
    private double current_v3 = 0;
    private double current_v4 = 0;

    private boolean ADJUSTMENT_ALLOWED = true;

    private boolean SCORING_BEHAVIOUR_LEFT = true; // turns left on score macro

    // -------------------------------------------------------------- ROBOT CONFIG

    private static final String FRONT_LEFT = "frontL";
    private static final String FRONT_RIGHT = "frontR";
    private static final String BACK_LEFT = "backL";
    private static final String BACK_RIGHT = "backR";
    private static final String SERVO_CLAW = "Servo";
    private static final String ARM_MOTOR = "armMotor";
    private static final String HUB_IMU = "imu";

    private static final double CLAW_CLOSE = 0.6;
    private static final double CLAW_OPEN = 0.43;

    private static final int MAX_ARM_HEIGHT = 4150;
    private static final int MIN_ARM_HEIGHT = 0;

    private static final int ARM_ADJUSTMENT_INCREMENT = 45;
    private static final int ARM_BOOST_MODIFIER = 1;
    private static final int ARM_RESET_TIMEOUT = 3;
    private static final int ARM_RESET_THRESHOLD = 800; // will only reset if the arm has previously gone above this threshold

    private static final double MAX_ACCELERATION_DEVIATION = 0.3; // higher = less smoothing

    private static final double PPR = 537.7; // gobuilda motor 85203 Series

    // -------------------------------------------------------------- JUNCTION PRESETS

    private static final int JUNCTION_OFF = 0;
    private static final int JUNCTION_LOW = 1650;
    private static final int JUNCTION_MID = 2700;
    private static final int JUNCTION_STANDBY = 3200;
    private static final int JUNCTION_HIGH = 4100;

    // -------------------------------------------------------------- ROBOT OPERATION

    private void Mecanum() {
        if (ADJUSTMENT_ALLOWED) { // only take manual when not macro control
            // IF GETTING RID OF MECANUM, GET RID OF REVERSE DIRECTION
            double yAxis;
            double xAxis;
            double rotateAxis;

            int dir = fieldCentricRed ? 1 : -1;

            // all negative when field centric red
            yAxis = gamepad1.left_stick_y * dir;
            xAxis = -gamepad1.left_stick_x * 1.1 * dir;
            rotateAxis = -gamepad1.right_stick_x * dir;

            if (gamepad1.back) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = xAxis * Math.cos(-botHeading) - yAxis * Math.sin(-botHeading);
            double rotY = xAxis * Math.sin(-botHeading) + yAxis * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotateAxis), 1);
            double frontLeftPower = (rotY + rotX + rotateAxis) / denominator;
            double backLeftPower = (rotY - rotX + rotateAxis) / denominator;
            double frontRightPower = (rotY - rotX - rotateAxis) / denominator;
            double backRightPower = (rotY + rotX - rotateAxis) / denominator;

            int driveSpeedModifier = 1;

            double stable_v1 = Stabilize(backLeftPower, current_v1);
            double stable_v2 = Stabilize(frontRightPower, current_v2);
            double stable_v3 = Stabilize(frontLeftPower, current_v3);
            double stable_v4 = Stabilize(backRightPower, current_v4);

            current_v1 = stable_v1;
            current_v2 = stable_v2;
            current_v3 = stable_v3;
            current_v4 = stable_v4;

            frontLM.setPower(stable_v3 / driveSpeedModifier);
            frontRM.setPower(stable_v2 / driveSpeedModifier);
            backLM.setPower(stable_v1 / driveSpeedModifier);
            backRM.setPower(stable_v4 / driveSpeedModifier);

            /* // assign speed modifier
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
            backRM.setPower(stable_v4 / driveSpeedModifier);*/
        }
    }

    private void Overrides() {
        // lining up arm for topmost cone
        if (ADJUSTMENT_ALLOWED) {
            if (gamepad1.b && armM.getCurrentPosition() < MAX_ARM_HEIGHT - ARM_ADJUSTMENT_INCREMENT) {
                targetArmPosition += ARM_ADJUSTMENT_INCREMENT;
                UpdateArm();
            }

            else if (gamepad1.a && armM.getCurrentPosition() > MIN_ARM_HEIGHT + ARM_ADJUSTMENT_INCREMENT) {
                targetArmPosition -= ARM_ADJUSTMENT_INCREMENT;
                UpdateArm();
            }
        }
    }

    private void RuntimeConfig() {
        if (gamepad1.dpad_down) {
            targetArmPosition = JUNCTION_OFF;
            UpdateArm();
        }

        else if (gamepad1.dpad_up) {
            targetArmPosition = JUNCTION_HIGH;
            UpdateArm();
        }

        if (gamepad1.dpad_left) {
            SCORING_BEHAVIOUR_LEFT = true;
        }

        else if (gamepad1.dpad_right) {
            SCORING_BEHAVIOUR_LEFT = false;
        }

        if (gamepad1.y && gamepad1.x) {
            fieldCentricRed = !fieldCentricRed;
            Delay(200);
        }
    }

    private void Macros() {
        int direction = SCORING_BEHAVIOUR_LEFT ? -1 : 1;

        if (gamepad1.left_bumper) {
            if (clawOpen) { // obtain cone
                MotorMode(true);
                ADJUSTMENT_ALLOWED = false;

                EncoderMove(0.5, 1, 1, 5); // TODO: should be the length of the arm and front of robot

                claw.setPosition(CLAW_CLOSE); // close
                clawOpen = false;

                Delay(150); // claw needs time

                targetArmPosition = JUNCTION_HIGH;
                UpdateArm();

                Delay(250);

                EncoderMove(0.5, -8, -8, 5); // TODO: tune this to clear cone stack
                EncoderMove(0.5, 2 * direction, -2 * direction, 4);

                ADJUSTMENT_ALLOWED = true;
                MotorMode(false);
            }

            else { // drop off cone
                MotorMode(true);
                ADJUSTMENT_ALLOWED = false;
                claw.setPosition(CLAW_OPEN); // open
                clawOpen = true;

                Delay(250); // need time to drop

                EncoderMove(0.85, -0.5, -0.5, 5); // move back for clearance TODO: move back, tune timeout

                Delay(150);

                targetArmPosition = JUNCTION_MID;
                UpdateArm();

                EncoderMove(0.9, -2 * direction, 2 * direction, 4); // turn left or right
                EncoderMove(0.9, 1, 1, 5); // move forward to line up

                ADJUSTMENT_ALLOWED = true;
                MotorMode(false);
            }
        }

        else if (gamepad1.right_bumper) { // manual close and open
            if (clawOpen) {
                claw.setPosition(CLAW_CLOSE);
                clawOpen = false;
                Delay(200);
            }
            else {
                claw.setPosition(CLAW_OPEN);
                clawOpen = true;
                Delay(200);
            }
        }
    }

    // -------------------------------------------------------------- USER FUNCTIONS

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

    private void EncoderMove(double power, double left, double right, double safetyTimeout) {
        int newLeftTarget = backLM.getCurrentPosition() + (int)(left * PPR);
        int newRightTarget = backRM.getCurrentPosition() + (int)(right * PPR);

        backLM.setTargetPosition(newLeftTarget);
        frontLM.setTargetPosition(newLeftTarget);
        backRM.setTargetPosition(newRightTarget);
        frontRM.setTargetPosition(newRightTarget);

        backLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        encoderRuntime.reset();
        backLM.setPower(Math.abs(power));
        frontLM.setPower(Math.abs(power));
        backRM.setPower(Math.abs(power));
        frontRM.setPower(Math.abs(power));

        while ((encoderRuntime.seconds() <= safetyTimeout) && (backRM.isBusy() && backLM.isBusy())) {
            telemetry.addData("TARGET COORDINATE: ",  "%7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("CURRENT COORDINATE: ",  "%7d :%7d", backLM.getCurrentPosition(), backRM.getCurrentPosition());
            telemetry.update();
        }

        backLM.setPower(0);
        frontLM.setPower(0);
        backRM.setPower(0);
        frontRM.setPower(0);

        backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void UpdateArm() { // after updating target pos, must run this to make arm move
        armM.setTargetPosition(Math.abs(targetArmPosition + 100));

        if ((targetArmPosition <= JUNCTION_OFF || targetArmPosition <= runtimeArmMinimum) && armCanReset) {
            armCanReset = false;
            armRuntime.reset();
            armM.setVelocity((double)2100 / ARM_BOOST_MODIFIER); // velocity used to be 1800

            while (armM.getCurrentPosition() <= 100 || armRuntime.seconds() <= ARM_RESET_TIMEOUT) {
                telemetry.update();
            }

            armM.setVelocity(0);
            runtimeArmMinimum = armM.getCurrentPosition();
            telemetry.addData("ARM RESET AT: ", runtimeArmMinimum);
            telemetry.update();
        }

        else {
            armRuntime.reset();
            armM.setVelocity((double)2100 / ARM_BOOST_MODIFIER); // velocity used to be 1800

            if (targetArmPosition >= ARM_RESET_THRESHOLD) { // if the arm has been lifted up, it can be reset
                armCanReset = true;
            }
        }
    }

    // -------------------------------------------------------------- MAIN INIT & LOOP

    public void init() {
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "ROBOT INITIALIZING...");    //
        telemetry.update();

        claw = hardwareMap.get(Servo.class, SERVO_CLAW);
        clawOpen = true;
        claw.setPosition(CLAW_OPEN);

        backLM = hardwareMap.get(DcMotorEx.class, BACK_LEFT);
        backRM = hardwareMap.get(DcMotorEx.class, BACK_RIGHT);

        frontLM = hardwareMap.get(DcMotorEx.class, FRONT_LEFT);
        frontRM = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT); //frontRM.setDirection(DcMotorSimple.Direction.REVERSE); // weird workaround Stanley put in

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

        frontRM.setDirection(DcMotorSimple.Direction.REVERSE);
        backRM.setDirection(DcMotorSimple.Direction.REVERSE);

        armM = hardwareMap.get(DcMotorEx.class, ARM_MOTOR);
        armM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armM.setTargetPosition(0);
        armM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // -------------------------------------------------------------- IMU INIT

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));

        imu = hardwareMap.get(IMU.class, HUB_IMU);
        imu.initialize(parameters);

        telemetry.addData("Status", "INITIALIZING IMU...");
        telemetry.update();

        Delay(500);

        // -------------------------------------------------------------- TELEMETRY INIT

        telemetry.setAutoClear(false);
        telemetry.addData("Status", "INITIALIZATION COMPLETE!");
        telemetry.update();
    }

    public void loop() {
        // -------------------------------------------------------------- PRELIMINARIES
        telemetry.clear();

        // -------------------------------------------------------------- DRIVE AND MANUAL OVERRIDES

        Macros();
        Overrides();
        RuntimeConfig();
        Mecanum();

        // -------------------------------------------------------------- TELEMETRY

        telemetry.addData("Claw Open: ", clawOpen);
        telemetry.addData("Current Arm Position: ", armM.getCurrentPosition());
        telemetry.addData("Target Arm Position: ", targetArmPosition);
        telemetry.addData("Arm Adjustment Allowed: ", ADJUSTMENT_ALLOWED);
        telemetry.addData("Score Behaviour: ", SCORING_BEHAVIOUR_LEFT);
        telemetry.addData("Current Alliance Mode : ", fieldCentricRed ? "RED" : "BLUE");
        telemetry.addData("FrontRM Encoder Value: ", frontRM.getCurrentPosition());
        telemetry.addData("FrontLM Encoder Value: ", frontLM.getCurrentPosition());
        telemetry.addData("BackRM Encoder Value: ", backRM.getCurrentPosition());
        telemetry.addData("BackLM Encoder Value: ", backLM.getCurrentPosition());
        telemetry.update();
    }
}
