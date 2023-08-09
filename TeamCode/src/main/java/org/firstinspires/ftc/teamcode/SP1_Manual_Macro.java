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

// TODO: IMPORTANT! SNAPSHOT CODE, SCORING WORKS INCLUDING MACROS, BUT NEXT VERSION WILL INCLUDE IMU CALCULATIONS FOR MACROS FOR MORE ACCURACY
// TODO: THIS VERSION INCLUDES ARM CONTROLS FOR A SECONDARY GAMEPAD. ROBOT IS STILL OPERATIONAL THROUGH SINGLE GAMEPAD.

@TeleOp()
public class SP1_Manual_Macro extends OpMode {
    // -------------------------------------------------------------- SYSTEM VAR
    private DcMotorEx backLM = null;
    private DcMotorEx backRM = null;
    private DcMotorEx frontLM = null;
    private DcMotorEx frontRM = null;
    private DcMotorEx armM = null;
    private IMU imu = null;
    private Servo claw = null;

    private final ElapsedTime encoderRuntime = new ElapsedTime();
    private final ElapsedTime armRuntime = new ElapsedTime();

    private int targetArmPosition = 0;
    private int runtimeArmMinimum = 0;
    private boolean armCanReset = false;
    private boolean clawOpen = true;

    private double current_v1 = 0;
    private double current_v2 = 0;
    private double current_v3 = 0;
    private double current_v4 = 0;

    private double driveSpeedModifier = 1;

    private boolean adjustmentAllowed = true;

    private boolean scoringBehaviourRight = false; // turns left on score macro
    private boolean fieldCentricRed = true;
    private boolean fieldCentricDrive = true;

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

    private static final int MAX_ARM_HEIGHT = 4050;
    private static final int MIN_ARM_HEIGHT = 0;

    private static final int ARM_ADJUSTMENT_INCREMENT = 45;
    private static final int ARM_BOOST_MODIFIER = 1;
    private static final int ARM_RESET_TIMEOUT = 3;
    private static final int ARM_RESET_THRESHOLD = 400; // will only reset if the arm has previously gone above this threshold

    private static final double MAX_ACCELERATION_DEVIATION = 0.3; // higher = less smoothing
    private static final double BASE_DRIVE_SPEED_MODIFIER = 1.5; // higher = less speed
    private static final double PRECISION_DRIVE_SPEED_MODIFIER = 3;

    private static final double PPR = 537.7; // gobuilda motor 85203 Series

    // -------------------------------------------------------------- JUNCTION PRESETS

    private static final int JUNCTION_OFF = 30; // will change
    private static final int JUNCTION_LOW = 1650;
    private static final int JUNCTION_MID = 2700;
    private static final int JUNCTION_STANDBY = 3200;
    private static final int JUNCTION_HIGH = 4100;

    // -------------------------------------------------------------- ROBOT OPERATION

    private void Mecanum() {
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        if (fieldCentricDrive) {
            // FIELD CENTRIC DRIVE REQUIRES RIGHT MOTORS TO BE REVERSED!!
            double yAxis;
            double xAxis;
            double rotateAxis;

            int dir = fieldCentricRed ? 1 : -1;

            // all negative when field centric red
            yAxis = gamepad1.left_stick_y * dir;
            xAxis = -gamepad1.left_stick_x * 1.1 * dir;
            rotateAxis = -gamepad1.right_stick_x * dir;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = xAxis * Math.cos(-heading) - yAxis * Math.sin(-heading);
            double rotY = xAxis * Math.sin(-heading) + yAxis * Math.cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotateAxis), 1);
            frontLeftPower = (rotY + rotX + rotateAxis) / denominator;
            backLeftPower = (rotY - rotX + rotateAxis) / denominator;
            frontRightPower = (rotY - rotX - rotateAxis) / denominator;
            backRightPower = (rotY + rotX - rotateAxis) / denominator;
        }

        else {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.right_stick_x);
            double robotAngle = Math.atan2(- 1 * gamepad1.right_stick_x, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.left_stick_y;
            frontLeftPower = r * Math.sin(robotAngle) + rightX; //front left
            backLeftPower = r * Math.cos(-robotAngle) + rightX; //back left
            frontRightPower = r * Math.sin(robotAngle) - rightX; //front right
            backRightPower = r * Math.cos(-robotAngle) - rightX; //back right
        }

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

    private void RuntimeConfig() {
        // -------------------------------------------------------------- MANUAL ARM CONTROL

        if (adjustmentAllowed) { // lining up arm for topmost cone
            if ((gamepad1.right_trigger >= 0.5 || gamepad2.right_trigger >= 0.5) && armM.getCurrentPosition() < MAX_ARM_HEIGHT - ARM_ADJUSTMENT_INCREMENT) {
                targetArmPosition += ARM_ADJUSTMENT_INCREMENT;
                NewUpdateArm(false);
            }

            else if ((gamepad1.left_trigger >= 0.5 || gamepad2.left_trigger >= 0.5) && armM.getCurrentPosition() > MIN_ARM_HEIGHT + ARM_ADJUSTMENT_INCREMENT) {
                targetArmPosition -= ARM_ADJUSTMENT_INCREMENT;
                NewUpdateArm(false);
            }

            else if (gamepad1.dpad_down || gamepad2.dpad_down) { // off
                targetArmPosition = JUNCTION_OFF;
                NewUpdateArm(true);
            }
        }

        else if (gamepad1.dpad_up || gamepad2.dpad_up) { // high junction
            targetArmPosition = JUNCTION_HIGH;
            NewUpdateArm(false);
        }

        // -------------------------------------------------------------- CONFIGURATION

        if (gamepad1.dpad_left) { // goes left on macro
            scoringBehaviourRight = true;
            Delay(50);
        }

        else if (gamepad1.dpad_right) { // goes right on macro
            scoringBehaviourRight = false;
            Delay(50);
        }

        if (gamepad1.x && gamepad1.back) { // toggle red / blue alliance for FCD
            fieldCentricRed = !fieldCentricRed;
            Delay(50);
        }

        if (gamepad1.start) {
            armM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armM.setTargetPosition(0);
            armM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad1.back) {
            imu.resetYaw();
        }

        if (clawOpen) {
            driveSpeedModifier = BASE_DRIVE_SPEED_MODIFIER;
        }

        else {
            driveSpeedModifier = PRECISION_DRIVE_SPEED_MODIFIER;
        }
    }

    private void Macros() {
        int direction = scoringBehaviourRight ? 1 : -1;

        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            if (clawOpen) { // obtain cone
                adjustmentAllowed = false;

                EncoderMove(0.5, 0.6, 0.6, 5); // TODO: should be the length of the arm and front of robot

                claw.setPosition(CLAW_CLOSE); // close
                clawOpen = false;

                Delay(200); // claw needs time

                targetArmPosition = JUNCTION_HIGH;
                NewUpdateArm(false);

                Delay(300);

                EncoderMove(0.5, -1.3, -1.3, 4); // TODO: tune this to clear cone stack
                EncoderMove(0.5, 1.9 * direction, -1.9 * direction, 4);

                adjustmentAllowed = true;
                driveSpeedModifier = PRECISION_DRIVE_SPEED_MODIFIER;
            }

            else { // drop off cone
                adjustmentAllowed = false;
                claw.setPosition(CLAW_OPEN); // open
                clawOpen = true;

                Delay(300); // need time to drop

                EncoderMove(0.85, -0.4, -0.4, 3); // move back for clearance TODO: move back, tune timeout

                Delay(250);

                targetArmPosition = JUNCTION_OFF;
                NewUpdateArm(true);

                EncoderMove(0.5, -2 * direction, 2 * direction, 4);
                EncoderMove(0.4, 1, 1, 5); // move forward to line up

                adjustmentAllowed = true;
                driveSpeedModifier = BASE_DRIVE_SPEED_MODIFIER;
            }
        }

        else if (gamepad1.dpad_left && gamepad1.x) {
            adjustmentAllowed = false;

            claw.setPosition(CLAW_OPEN);
            clawOpen = true;

            EncoderMove(0.5, 0.1, 0.1, 3);
            EncoderMove(0.5, -0.5 * direction, -.5 * direction, 3);
            EncoderMove(0.5, 0.6, 0.6, 4);
        }

        else if (gamepad1.right_bumper || gamepad2.right_bumper) { // manual close and open
            if (clawOpen) {
                claw.setPosition(CLAW_CLOSE);
                clawOpen = false;

                driveSpeedModifier = BASE_DRIVE_SPEED_MODIFIER;

                Delay(200);
            }
            else {
                claw.setPosition(CLAW_OPEN);
                clawOpen = true;

                driveSpeedModifier = PRECISION_DRIVE_SPEED_MODIFIER;

                Delay(200);
            }
        }
    }

    // -------------------------------------------------------------- USER FUNCTIONS

    private void Delay(double time) {
        try { sleep((long)time); } catch (Exception e) { System.out.println("interrupted"); }
    }

    private double Stabilize(double new_accel, double current_accel) {
        double dev = new_accel - current_accel;
        return Math.abs(dev) > MAX_ACCELERATION_DEVIATION ? current_accel + MAX_ACCELERATION_DEVIATION * dev / Math.abs(dev) : new_accel;
    }

    private void EncoderMove(double power, double left, double right, double safetyTimeout) {
        int backLMTarget = backLM.getCurrentPosition() - (int)(left * PPR); // should theoretically make it go backwards
        int frontLMTarget = frontLM.getCurrentPosition() - (int)(left * PPR);
        int backRMTarget = backRM.getCurrentPosition() - (int)(right * PPR);
        int frontRMTarget = frontRM.getCurrentPosition() - (int)(right * PPR);

        backLM.setTargetPosition(backLMTarget);
        frontLM.setTargetPosition(frontLMTarget);
        backRM.setTargetPosition(backRMTarget);
        frontRM.setTargetPosition(frontRMTarget);

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
            telemetry.clear();
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

        Delay(50);
    }

    /*private void UpdateArm() { // after updating target pos, must run this to make arm move
        armM.setTargetPosition(targetArmPosition);

        if ((targetArmPosition <= JUNCTION_OFF || targetArmPosition <= runtimeArmMinimum) && armCanReset) {
            armCanReset = false;
            armRuntime.reset();
            armM.setVelocity((double)2100 / ARM_BOOST_MODIFIER);

            while (armM.getCurrentPosition() <= 80 || armRuntime.seconds() <= ARM_RESET_TIMEOUT) {
                telemetry.update();
            }

            armM.setVelocity(0);
            //runtimeArmMinimum = armM.getCurrentPosition();
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
    }*/

    private void NewUpdateArm(boolean reset) { // test new function
        armM.setTargetPosition(targetArmPosition);

        if (reset) {
            armM.setTargetPosition(30);
            targetArmPosition = 30;
            armCanReset = false;
            armRuntime.reset();

            while (armM.getCurrentPosition() <= 50 || armRuntime.seconds() <= ARM_RESET_TIMEOUT) {
                armM.setVelocity((double)2100 / ARM_BOOST_MODIFIER);

                if (armM.getCurrentPosition() <= 50 || armRuntime.seconds() <= ARM_RESET_TIMEOUT) {
                    break;
                }
            }

            if (armM.getCurrentPosition() <= 50) {
                armM.setVelocity(0);
            }

            //armM.setVelocity(0);
            //armM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //armM.setTargetPosition(0);
            //armM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //runtimeArmMinimum = armM.getCurrentPosition();
            telemetry.addData("ARM RESET AT: ", runtimeArmMinimum);
            telemetry.update();
        }

        else {
            armRuntime.reset();
            armM.setVelocity((double)2500 / ARM_BOOST_MODIFIER); // velocity used to be 1800

            if (targetArmPosition >= ARM_RESET_THRESHOLD) { // if the arm has been lifted up, it can be reset
                armCanReset = true;
            }
        }
    }

    // -------------------------------------------------------------- MAIN INIT & LOOP

    public void init() {
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "INITIALIZING ROBOT...");    //
        telemetry.update();

        driveSpeedModifier = BASE_DRIVE_SPEED_MODIFIER;

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

        telemetry.addData("Status", "CALIBRATING IMU...");
        telemetry.addData("Important Information", "PLACE ROBOT FACING AWAY FROM ALLIANCE BOX!");
        telemetry.update();

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));

        imu = hardwareMap.get(IMU.class, HUB_IMU);
        imu.initialize(parameters);
        imu.resetYaw();

        Delay(1000);

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
        RuntimeConfig();
        Mecanum();

        // -------------------------------------------------------------- TELEMETRY

        telemetry.addData("Claw Open: ", clawOpen);
        telemetry.addData("Current Arm Position: ", armM.getCurrentPosition());
        telemetry.addData("Target Arm Position: ", targetArmPosition);
        telemetry.addData("Adjustment Allowed: ", adjustmentAllowed);
        telemetry.addData("Score Behaviour: ", scoringBehaviourRight ? "LEFT" : "RIGHT");
        telemetry.addData("Current Alliance Mode : ", fieldCentricRed ? "RED" : "BLUE");
        telemetry.addData("Current Drive Mode: ", fieldCentricDrive ? "FIELD CENTRIC" : "ROBOT CENTRIC");
        telemetry.addData("Current Speed Mode: ", driveSpeedModifier == BASE_DRIVE_SPEED_MODIFIER ? "BASE SPEED" : "PRECISION MODE");
        telemetry.addData("FrontRM Encoder Value: ", frontRM.getCurrentPosition());
        telemetry.addData("FrontLM Encoder Value: ", frontLM.getCurrentPosition());
        telemetry.addData("BackRM Encoder Value: ", backRM.getCurrentPosition());
        telemetry.addData("BackLM Encoder Value: ", backLM.getCurrentPosition());
        telemetry.update();
    }
}
