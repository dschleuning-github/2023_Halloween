
package org.firstinspires.ftc.teamcode.Robo1_3;
//updated code 8/20/2023
// located at:  C:\Users\dschl\Documents\Davidfold2\DavidBot\2022_10_08 Robo1\
// 2023_08_20 FTC_v8_2\FtcRobotController-8.2\TeamCode\src\main\java\org\firstinspires\ftc\teamcode


//updated code on 1/8/2022
//this code is located at:
// C:\Users\dschl\Documents\Davidfold2\DavidBot\2021_07_04 Robo1\
// FtcRobot_06_29_2021\FtcRobotController-master\TeamCode\src\main\java\org\firstinspires\ftc\teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an opmode to control Robo1 -- robot with motor drives and a vaccuum cleaner
 */

@TeleOp(name="Robo1_3_4", group="Linear Opmode")
//@Disabled

public class Robo1_3_4 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();       //TBD....what is ElapsedTime()
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor vacBrush = null;
    private DcMotor vacFan = null;

    static final double COUNTS_PER_MOTOR_REV = 560;    // HD HEX (REV-41-1291) encoder with 20:1
    static final double WHEEL_DIAMETER_INCHES = 3.61;    // For figuring circumference
    static final double FUDGE_FACTOR = 79.25 / 74.84;   //meas 79.25;  encoder 74.84;  (Test 75.0 m; y=74.48)
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415 * FUDGE_FACTOR);   //  46.6 counts_per_in
    static final double WIDTH_WHEELS_IN = 18.625 * 362.11 / 345;  //fudge factor...off by 15deg

    static final double DRIVE_SPEED = 0.5;     //0.8 too fast

    double drive = 0.0;   //create "drive" has class variable
    double turn = 0.0;   //create "drive" has class variable

    boolean vac_on = false;
    double brushPower = 1.0;
    double vacPower = 1.0;
    double[] locationArray = new double[9];         // [L, R, x, y , H, T, V, Omega]
    double[] startArray = new double[9];
    double[] stopArray = new double[9];
//    double H_deg;

    // **************************
    private BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;
// *********************

    double TEST_DIST = 36.0;  //72.0;   //inches

    @Override
    public void runOpMode() {
        rightDrive = hardwareMap.get(DcMotor.class, "Davidmotor0");
        leftDrive = hardwareMap.get(DcMotor.class, "Davidmotor1");
        vacBrush = hardwareMap.get(DcMotor.class, "Davidmotor2");
        vacFan = hardwareMap.get(DcMotor.class, "Davidmotor3");

        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        vacBrush.setDirection(DcMotor.Direction.REVERSE);  //Note this makes Red-Red match
        vacFan.setDirection(DcMotor.Direction.REVERSE);

        locationArray[0] = leftDrive.getCurrentPosition() / COUNTS_PER_INCH;
        locationArray[1] = rightDrive.getCurrentPosition() / COUNTS_PER_INCH;
        locationArray[2] = 0.0;
        locationArray[3] = 0.0;
        locationArray[4] = 0.0;
        locationArray[5] = 0.0;
        locationArray[6] = 0.0;
        locationArray[7] = 0.0;
        locationArray[8] = 0.0;

        boolean dPad_left = false;
        boolean dPad_right = false;
        boolean bumper_left = false;
        boolean dPad_down = false;
        boolean dPad_up = false;
        boolean aButton = false;
        boolean yButton = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

// **********************
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("CODE update from MAC","--home2");
        telemetry.update();
//  ********************************************************

        // Wait for driver to press PLAY)
        waitForStart();
        runtime.reset();

        //***************** START THE RUN MODE ************************
        while (opModeIsActive()) {
            bumper_left = this.gamepad1.left_bumper;
            if (bumper_left) {
                locationArray[0] = leftDrive.getCurrentPosition() / COUNTS_PER_INCH;
                locationArray[1] = rightDrive.getCurrentPosition() / COUNTS_PER_INCH;
                locationArray[2] = 0.0;
                locationArray[3] = 0.0;
                locationArray[4] = 0.0;
                resetAngle();
            }
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.left_stick_x;
            driveTankmode();

            if (this.gamepad1.dpad_up){
                stopArray = getLocation(locationArray[0], locationArray[1], locationArray[2],
                        locationArray[3], locationArray[4], locationArray[5], locationArray[6],
                        locationArray[7], locationArray[8]);
            }
            if (this.gamepad1.dpad_down) {
                startArray = getLocation(locationArray[0], locationArray[1], locationArray[2],
                        locationArray[3], locationArray[4], locationArray[5], locationArray[6],
                        locationArray[7], locationArray[8]);
            }
            if (this.gamepad1.right_bumper) cleanRectangle();

            if (this.gamepad1.y) driveDistance(TEST_DIST);
            if (this.gamepad1.b) rotateAbsAngle(90);
            if (this.gamepad1.a) raster();
            if (this.gamepad1.x) rotateAbsAngle(-90);

            dPad_left = this.gamepad1.dpad_left;
            dPad_right = this.gamepad1.dpad_right;
            turnOnVac(dPad_left, dPad_right);

            //Update the telemetry
            locationArray = getLocation(locationArray[0], locationArray[1], locationArray[2],
                    locationArray[3], locationArray[4], locationArray[5], locationArray[6],
                    locationArray[7], locationArray[8]);
            // Location:  [0] L_in , [1] R_in , [2] X_in, [3] Y_in, [4] H_deg,
            //            [5] T_s, [6] dT_s, [7] v_inps, [8] omega_degps

//          telemetry.addData("Status", "Run Time: " + runtime.toString());
//          telemetry.addData("vacuum on:", "%b", vac_on);
//            telemetry.addData("encoder", "L=%.2f, R=%.2f",
//                    locationArray[0], locationArray[1]);
            telemetry.addData("Position", "x=%.2f, y=%.2f, H=%.2f",
                    locationArray[2], locationArray[3], locationArray[4]);
            telemetry.addData("Delta", "dT=%.4f, V=%.2f, Omega=%.2f",
                    locationArray[6], locationArray[7], locationArray[8]);
            telemetry.update();
        }
    }

    private void cleanRectangle(){
        turnOnVac(true, false);
        double deltaX = 12;

        locationArray = getLocation(locationArray[0], locationArray[1], locationArray[2],
                locationArray[3], locationArray[4], locationArray[5], locationArray[6],
                locationArray[7], locationArray[8]);
        // Location:  [0] L_in , [1] R_in , [2] X_in, [3] Y_in, [4] H_deg,
        //            [5] T_s, [6] dT_s, [7] v_inps, [8] omega_degps
        double startX = startArray[2];
        double startY = startArray[3];
        double stopX = stopArray[2];
        double stopY = stopArray[3];

        //go to start
        double moveX = locationArray[2] - startX;
        if (moveX > 0 ){
            rotateAbsAngle(-90);
            driveDistance(moveX);
        } else {
            rotateAbsAngle(90);
            driveDistance(-moveX);
        }

        double moveY = locationArray[3] - startY;
        if (moveY > 0 ){
            rotateAbsAngle(180);
            driveDistance(moveY);
        } else {
            rotateAbsAngle(0);
            driveDistance(-moveY);
        }
        rotateAbsAngle(0);

        double startT_s = runtime.seconds();
        double runT_s = 0;
        double timeout_s = 15;
        int Nx = (int) Math.abs(moveX / deltaX);
        //while (runT_s < timeout_s){
        //runT_s = runtime.seconds() - startT_s;
        for (int i = 0; i<=Nx; i++){
            rotateAbsAngle(0);
            driveDistance(moveY);
            rotateAbsAngle(90);
            driveDistance(deltaX);
            rotateAbsAngle(180);
            driveDistance(moveY);
        }

        turnOnVac(false, true);

    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;

//        telemetry.addData("angles:", "first=%.2f, second=%.2f, third=%.2f",
//                angles.firstAngle, angles.secondAngle, angles.thirdAngle);

        return -globalAngle;
    }

    public double[] getLocation(double oldL_in, double oldR_in, double oldX_in, double oldY_in, double oldH_deg,
                                double oldT_s, double oldDT_s, double oldV_inps, double oldOmega_degps) {
        // Location:  [0] L_in , [1] R_in , [2] X_in, [3] Y_in, [4] H_deg,
        //            [5] T_s, [6] dT_s, [7] v_inps, [8] omega_degps

        double newT_s = runtime.seconds();
        double[] locationNew = new double[9];
        double newL_in = leftDrive.getCurrentPosition() / COUNTS_PER_INCH;
        double newR_in = rightDrive.getCurrentPosition() / COUNTS_PER_INCH;
        double deltaL_in = newL_in - oldL_in;
        double deltaR_in = newR_in - oldR_in;

//        double theta_rad = (deltaL_in - deltaR_in) / WIDTH_WHEELS_IN;
//        double theta_deg = theta_rad * 180.0 / Math.PI;
        double gyroH_deg = getAngle();
        double gyroH_rad = gyroH_deg * Math.PI / 180.0;
        double deltaGyroH_deg = gyroH_deg - oldH_deg;
        double deltaGyroH_rad = deltaGyroH_deg * Math.PI / 180.0;

        double deltaLR_in = (deltaL_in + deltaR_in) / 2.0;
        double deltaI_in = deltaLR_in * Math.sin(deltaGyroH_rad);
        double deltaJ_in = deltaLR_in * Math.cos(deltaGyroH_rad);

//        double newH_rad = oldH_deg * Math.PI / 180.0 + theta_rad;
//        double newH_deg = newH_rad * 180.0 / Math.PI;
        double deltaX_in = deltaI_in * Math.cos(gyroH_rad) + deltaJ_in * Math.sin(gyroH_rad);
        double deltaY_in = -deltaI_in * Math.sin(gyroH_rad) + deltaJ_in * Math.cos(gyroH_rad);

        double newDT_s = newT_s - oldT_s;
        double v_inps = deltaLR_in / newDT_s;
        double omega_degps = deltaGyroH_deg / newDT_s;

        locationNew[0] = newL_in;
        locationNew[1] = newR_in;
        locationNew[2] = oldX_in + deltaX_in;
        locationNew[3] = oldY_in + deltaY_in;
        locationNew[4] = gyroH_deg;    //newH_deg;
        locationNew[5] = newT_s;
        locationNew[6] = newDT_s;
        locationNew[7] = v_inps;
        locationNew[8] = omega_degps;

        return locationNew;
    }

    public void driveDistance(double distance_in) {
        locationArray = getLocation(locationArray[0], locationArray[1], locationArray[2],
                locationArray[3], locationArray[4], locationArray[5], locationArray[6],
                locationArray[7], locationArray[8]);
        // Location:  [0] L_in , [1] R_in , [2] X_in, [3] Y_in, [4] H_deg,
        //            [5] T_s, [6] dT_s, [7] v_inps, [8] omega_degps
        double startT_s = locationArray[5];
        double driveTime_s = 0;
        double startL_in = locationArray[0];
        double startR_in = locationArray[1];
        double distTravelled_in = 0;

        double leftPower = 0;
        double rightPower = 0;
        double driveSpeedMax = 0.4;
        double driveSpeedMin = 0.2;
        double direction = 1.0;
        double leftCor = 1.0;
        double rightCor = 1.0;
        double omega_radpms = 0;
        double corGain = 1000.0;

        if (distance_in > 0) {
            direction = 1.0;
        } else {
            direction = -1.0;
        }

        while (opModeIsActive() && (Math.abs(distance_in - distTravelled_in) > 0.2) && (driveTime_s < 10.0)) {
            omega_radpms = locationArray[8] * Math.PI / 180.0 / 1000.0;
            leftCor = Range.clip(1 - omega_radpms * corGain, 0.8, 1.2);
            rightCor = Range.clip(1 + omega_radpms * corGain, 0.8, 1.2);
            if (Math.abs(distance_in - distTravelled_in) > 10) {
                leftPower = Range.clip(direction * leftCor * driveSpeedMax, -1.0, 1.0);   //DRIVE_SPEED = 0.8;    0.4
                rightPower = Range.clip(direction * rightCor * driveSpeedMax, -1.0, 1.0);
            } else {
                leftPower = Range.clip(direction * leftCor * driveSpeedMin, -1.0, 1.0);   //DRIVE_SPEED = 0.8;    0.4
                rightPower = Range.clip(direction * rightCor * driveSpeedMin, -1.0, 1.0);
            }

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            locationArray = getLocation(locationArray[0], locationArray[1], locationArray[2],
                    locationArray[3], locationArray[4], locationArray[5], locationArray[6],
                    locationArray[7], locationArray[8]);
            // Location:  [0] L_in , [1] R_in , [2] X_in, [3] Y_in, [4] H_deg,
            //            [5] T_s, [6] dT_s, [7] v_inps, [8] omega_degps
            distTravelled_in = ((locationArray[0] - startL_in) + (locationArray[1] - startR_in)) / 2.0;
            driveTime_s = locationArray[5] - startT_s;

//            telemetry.addData("angle", "omega_radpms = %.5f", omega_radpms);
//            telemetry.addData("Position", "L=%.2f, R=%.2f, x=%.2f, y=%.2f, H=%.2f",
//                    locationArray[0], locationArray[1], locationArray[2], locationArray[3], locationArray[4]);
//            telemetry.addData("Delta", "dT=%.4f, V=%.2f, Omega=%.2f",
//                    locationArray[6], locationArray[7], locationArray[8]);

            telemetry.addData("Position", "x=%.2f, y=%.2f, H=%.2f",
                    locationArray[2], locationArray[3], locationArray[4]);
            telemetry.addData("Delta", "dT=%.4f, V=%.2f, Omega=%.2f",
                    locationArray[6], locationArray[7], locationArray[8]);

            telemetry.addData("driveDistance", "distTravelled=%.1f  stillToGo=%.1f",
                    distTravelled_in, distance_in - distTravelled_in);
            telemetry.addData("corr", "leftCor=%.4f, rightCor=%.4f",
                    leftCor, rightCor);

            telemetry.update();
        }
    }

    public void rotateAbsAngle(double angleEndDeg) {
        locationArray = getLocation(locationArray[0], locationArray[1], locationArray[2],
                locationArray[3], locationArray[4], locationArray[5], locationArray[6],
                locationArray[7], locationArray[8]);
        // Location:  [0] L_in , [1] R_in , [2] X_in, [3] Y_in, [4] H_deg,
        //            [5] T_s, [6] dT_s, [7] v_inps, [8] omega_degps

        double startT_s = runtime.seconds();
        double runT_s = 0;
        double timeout_s = 10.0;

        double angleCurrentDeg = locationArray[4];
        double motorSpeed_fast = 0.4;
        double motorSpeed_slow = .2;
        double slowAngle = 5;
        double angleClose = 1.0


                ;
        double rotate;

        while (opModeIsActive() && (Math.abs(angleEndDeg - angleCurrentDeg) > angleClose) &&
                (runT_s < timeout_s) ) {

            locationArray = getLocation(locationArray[0], locationArray[1], locationArray[2],
                    locationArray[3], locationArray[4], locationArray[5], locationArray[6],
                    locationArray[7], locationArray[8]);
            // Location:  [0] L_in , [1] R_in , [2] X_in, [3] Y_in, [4] H_deg,
            //            [5] T_s, [6] dT_s, [7] v_inps, [8] omega_degps
            angleCurrentDeg = locationArray[4];

            if ((angleEndDeg - angleCurrentDeg) > 0.0) {
                rotate = 1.0;
            } else {
                rotate = -1.0;
            }

            if (Math.abs(angleEndDeg - angleCurrentDeg) > slowAngle){
                leftDrive.setPower(motorSpeed_fast * rotate );
                rightDrive.setPower(-motorSpeed_fast * rotate);
            } else {
                leftDrive.setPower(motorSpeed_slow * rotate );
                rightDrive.setPower(-motorSpeed_slow * rotate);
            }

            telemetry.addData("Position", "x=%.2f, y=%.2f, H=%.2f",
                    locationArray[2], locationArray[3], locationArray[4]);
            telemetry.addData("Delta", "dT=%.4f, V=%.2f, Omega=%.2f",
                    locationArray[6], locationArray[7], locationArray[8]);

//            telemetry.addData("stats", "angleCurrentDet=%.1f",
//                    locationArray[8]);
            telemetry.update();
            runT_s = runtime.seconds() - startT_s;
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(250);   // optional pause after each move
    }

    public void raster() {
//        driveDistance(36);
//        rotateAbsAngle(-90);
        driveDistance(48);
        rotateAbsAngle(90);
        driveDistance(12);
        rotateAbsAngle(180);
        driveDistance(48);
        rotateAbsAngle(90);
        driveDistance(12);
        rotateAbsAngle(0);
        driveDistance(48);
    }

    public void driveTankmode() {
        double leftPower = Range.clip(DRIVE_SPEED * (drive + 0.5 * turn), -1.0, 1.0);
        double rightPower = Range.clip(DRIVE_SPEED * (drive - 0.5 * turn), -1.0, 1.0);
        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void turnOnVac(boolean dLeft, boolean dRight) {
        if (dLeft) {
            vac_on = true;
        }
        if (dRight) {
            vac_on = false;
        }
        if (vac_on) {
            vacBrush.setPower(brushPower);
            vacFan.setPower(vacPower);
        }
        if (vac_on == false) {
            vacBrush.setPower(0);
            vacFan.setPower(0);
        }
    }

}

/*
    public double[] getLocation_OLD(double oldL_in, double oldR_in, double oldX_in, double oldY_in, double oldH_deg,
                                    double oldT_s, double oldDT_s, double oldV_inps, double oldOmega_degps) {
        // Location:  [0] L_in , [1] R_in , [2] X_in, [3] Y_in, [4] H_deg,
        //            [5] T_s, [6] dT_s, [7] v_inps, [8] omega_degps

        double newT_s = runtime.seconds();
        double[] locationNew = new double[9];
        double newL_in = leftDrive.getCurrentPosition() / COUNTS_PER_INCH;
        double newR_in = rightDrive.getCurrentPosition() / COUNTS_PER_INCH;
        double deltaL_in = newL_in - oldL_in;
        double deltaR_in = newR_in - oldR_in;

        double theta_rad = (deltaL_in - deltaR_in) / WIDTH_WHEELS_IN;
        double theta_deg = theta_rad * 180.0 / Math.PI;
        double deltaLR_in = (deltaL_in + deltaR_in) / 2.0;
        double deltaI_in = deltaLR_in * Math.sin(theta_rad);
        double deltaJ_in = deltaLR_in * Math.cos(theta_rad);

        double newH_rad = oldH_deg * Math.PI / 180.0 + theta_rad;
        double newH_deg = newH_rad * 180.0 / Math.PI;
        double deltaX_in = deltaI_in * Math.cos(newH_rad) + deltaJ_in * Math.sin(newH_rad);
        double deltaY_in = -deltaI_in * Math.sin(newH_rad) + deltaJ_in * Math.cos(newH_rad);

        double newDT_s = newT_s - oldT_s;
        double v_inps = deltaLR_in / newDT_s;
        double omega_degps = theta_deg / newDT_s;

        double gyroTheta_deg = getAngle();

        locationNew[0] = newL_in;
        locationNew[1] = newR_in;
        locationNew[2] = oldX_in + deltaX_in;
        locationNew[3] = oldY_in + deltaY_in;
        locationNew[4] = gyroTheta_deg;    //newH_deg;
        locationNew[5] = newT_s;
        locationNew[6] = newDT_s;
        locationNew[7] = v_inps;
        locationNew[8] = omega_degps;

        return locationNew;
    }

        public void rotateAngleOLD(double angleDeg) {
        locationArray = getLocation(locationArray[0], locationArray[1], locationArray[2],
                locationArray[3], locationArray[4], locationArray[5], locationArray[6],
                locationArray[7], locationArray[8]);  // L, R, X, Y, H, T, dT, V, Omega

        double startT_s = runtime.seconds();
        double currentT_s = runtime.seconds();
        double timeout_s = 5;

        double angleCurrentDeg = locationArray[4] * 180 / Math.PI;
        double angleStartDeg = angleCurrentDeg;
        double angleEndDeg = angleStartDeg + angleDeg;
        double angleClose = 0.1;
        double rotate;
        double leftPower;
        double rightPower;

        while (opModeIsActive() && (Math.abs(angleEndDeg - angleCurrentDeg) > angleClose) &&
                ((currentT_s-startT_s) < timeout_s) ) {

            if ((angleEndDeg - angleCurrentDeg) > 0.0) {
                rotate = 0.5;
            } else {
                rotate = -0.5;
            }

            leftPower = Range.clip(DRIVE_SPEED * (rotate), -1.0, 1.0);
            rightPower = Range.clip(DRIVE_SPEED * (-rotate), -1.0, 1.0);
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            locationArray = getLocation(locationArray[0], locationArray[1], locationArray[2],
                    locationArray[3], locationArray[4], locationArray[5], locationArray[6],
                    locationArray[7], locationArray[8]);  // L, R, X, Y, H, T, dT, V, Omega
            // Location:  [0] L_in , [1] R_in , [2] X_in, [3] Y_in, [4] H_deg,
            //            [5] T_s, [6] dT_s, [7] v_inps, [8] omega_degps
            angleCurrentDeg = locationArray[4];
            currentT_s = runtime.seconds();

            telemetry.addData("stats", "angleCurrentDet=%.1f",
                    locationArray[4]);
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(250);   // optional pause after each move
    }

*/