package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// TODO: test encoderMove function for forward/backward + imu calibration (which axes?)

@TeleOp()
public class LocalTesting extends OpMode {
    // -------------------------------------------------------------- SYSTEM VAR
    private DcMotorEx backLM = null;
    private DcMotorEx backRM = null;
    private DcMotorEx frontLM = null;
    private DcMotorEx frontRM = null;
    private BNO055IMU imu = null;

    private double globalAngle = 0.30;

    private double current_v1 = 0;
    private double current_v2 = 0;
    private double current_v3 = 0;
    private double current_v4 = 0;

    private final ElapsedTime encoderRuntime = new ElapsedTime();
    private Orientation lastAngles = new Orientation();

    // -------------------------------------------------------------- ROBOT CONFIG

    private static final String FRONT_LEFT = "frontL";
    private static final String FRONT_RIGHT = "frontR";
    private static final String BACK_LEFT = "backL";
    private static final String BACK_RIGHT = "backR";

    private static final double MAX_ACCELERATION_DEVIATION = 0.3; // higher = less smoothing

    private static final double PPR = 537.7; // gobuilda motor 85203 Series

    // -------------------------------------------------------------- ROBOT OPERATION

    private void Mecanum() {
        // assign speed modifier
        int driveSpeedModifier = 1;

        // mecanum
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.right_stick_x);
        double robotAngle = Math.atan2(- 1 * gamepad1.right_stick_x, gamepad1.left_stick_x) - Math.PI / 4;
        robotAngle -= GetRobotAngle(); // in testing
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

    private double GetRobotAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
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
        int newLeftTarget = backLM.getCurrentPosition() + (int)(left * PPR);
        int newRightTarget = backRM.getCurrentPosition() + (int)(right * PPR);

        backLM.setTargetPosition(-newLeftTarget); // used to be pos
        frontLM.setTargetPosition(-newLeftTarget);
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

    // -------------------------------------------------------------- MAIN INIT & LOOP

    public void init() {
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "ROBOT INITIALIZING...");    //
        telemetry.update();

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

        // -------------------------------------------------------------- IMU INIT

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "CALIBRATING IMU...");
        telemetry.update();

        Delay(500);

        telemetry.addData("IMU Calibration Status", imu.getCalibrationStatus().toString());
        telemetry.update();

        Delay(2000);

        // -------------------------------------------------------------- TELEMETRY INIT

        telemetry.setAutoClear(false);

        telemetry.addData("Status", "INITIALIZATION COMPLETE!");
        telemetry.update();
    }

    public void loop() {
        // -------------------------------------------------------------- PRELIMINARIES
        telemetry.clear();

        // -------------------------------------------------------------- DRIVE AND MANUAL OVERRIDES

        Mecanum();

        if (gamepad1.dpad_right) {
            EncoderMove(0.5, 3, 3, 3);
            EncoderMove(0.5, -3, -3, 3);
            EncoderMove(0.5, 3, -3, 3);
        }

        // -------------------------------------------------------------- TELEMETRY

        telemetry.addData("FrontRM Encoder Value: ", frontRM.getCurrentPosition());
        telemetry.addData("FrontLM Encoder Value: ", frontLM.getCurrentPosition());
        telemetry.addData("BackRM Encoder Value: ", backRM.getCurrentPosition());
        telemetry.addData("BackLM Encoder Value: ", backLM.getCurrentPosition());
        telemetry.update();
    }
}
