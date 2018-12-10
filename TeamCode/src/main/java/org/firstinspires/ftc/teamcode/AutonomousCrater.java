/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="Autonomous Crater", group="Pushbot")
public class AutonomousCrater extends LinearOpMode {

    /* Declare OpMode members. */
//    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.8;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    static final double SERVO_DOWN = 30/280.0;
    static final double SERVO_UP = 180/280.0;

    boolean gold = false;

    ElapsedTime timer = new ElapsedTime();

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    private DcMotor motorLeft = null;
    private DcMotor motorRight = null;
    private DcMotor motorLift = null;
    private DcMotor motorVertical = null;
    private DcMotor motorHorizontal = null;
    private DcMotor motorHDrive = null;
    private DcMotor motorIntake = null;
    private NormalizedColorSensor colorSensor = null;

    //  TODO Servos
    private Servo servoFlipper1 = null;
    private Servo servoFlipper2 = null;

    @Override
    public void runOpMode() {

        //*************************** IMU Setup ************************/

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //*************************** IMU Setup ************************/

        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorLift = hardwareMap.get(DcMotor.class, "motorLift");
        motorVertical = hardwareMap.get(DcMotor.class, "motorVertical");
        motorVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorHorizontal = hardwareMap.get(DcMotor.class, "motorHorizontal");
        motorHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorHDrive = hardwareMap.get(DcMotor.class, "motorHDrive");
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");


        servoFlipper1 = hardwareMap.get(Servo.class, "servoFlipper1");
        servoFlipper2 = hardwareMap.get(Servo.class, "servoFlipper2");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorHDrive.setDirection(DcMotor.Direction.REVERSE);
        motorLift.setDirection(DcMotor.Direction.FORWARD);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
//        unhook(26300);
//        motorHDrive.setPower(1);
//        gyroTurn(DRIVE_SPEED, -20);
//        pause();
//        fwd(DRIVE_SPEED,1000);
//        pause();
//        gyroTurn(TURN_SPEED, -4);
//        pause();
//        Hright(1,3000);
//        pause();
//        gyroTurn(TURN_SPEED,-4);
//        pause();
//        fwd(DRIVE_SPEED,1900);
//        gyroTurn(TURN_SPEED,-4);

        if (false) {
            fwd(DRIVE_SPEED, 5000);
            outtake(1.5);
            bwd(DRIVE_SPEED,1000);
            //outtake
            // move forward
        } else {
            bwd(DRIVE_SPEED, 1000);
            pause();
            gyroTurn(TURN_SPEED, -30);
            pause();
            fwd(DRIVE_SPEED,1900);
            pause();
            //slide out
            if (false) {
                fwd(DRIVE_SPEED,2500);
                pause();
                gyroTurn(TURN_SPEED,35);
                pause();
                fwd(DRIVE_SPEED, 3000);
                outtake(1.5);
                //outtake
                //slide in
                //move forward
                //turn left 70
                //fwd
            } else {
              bwd(DRIVE_SPEED,2500);
              pause();
              gyroTurn(TURN_SPEED, 30);
              pause();
              fwd(DRIVE_SPEED,6500);
              pause();
              gyroTurn(TURN_SPEED, -35);
              pause();
              fwd(DRIVE_SPEED,4300);
              outtake(1.5);
                //slide in
                //turn left 70
                //fwd
                //turn right 35
                //fwd
            }
        }

    }

    public void Hright(double power, int distance) {
        motorHDrive.setTargetPosition(-distance);
        motorHDrive.setPower(power);
        while (opModeIsActive() && motorHDrive.isBusy()) {
            telemetry.addData("Encoder", motorHDrive.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorHDrive.setPower(0);

        motorHDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void fwd(double power, int distance) {
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setTargetPosition(distance);
        motorRight.setTargetPosition(distance);
        motorLeft.setPower(power);
        motorRight.setPower(power);
        while (opModeIsActive() && motorLeft.isBusy() && motorRight.isBusy()) {
            telemetry.addData("Encoder", motorLeft.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorLeft.setPower(0);
        motorRight.setPower(0);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void bwd(double power, int distance) {
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setTargetPosition(-distance);
        motorRight.setTargetPosition(-distance);

        motorLeft.setPower(power);
        motorRight.setPower(power);
        while (opModeIsActive() && motorLeft.isBusy() && motorRight.isBusy()) {
            telemetry.addData("Encoder", motorLeft.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorLeft.setPower(0);
        motorRight.setPower(0);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void outtake(double time) {
        timer.reset();
        while(opModeIsActive() && (timer.time() < time)) {
            motorIntake.setPower(1);
        }
        motorIntake.setPower(0);
    }

    public boolean colorCheck() {
        int scale = 100;
        double red;
        double blue;
        double green;
        double maxRed = 0;
        double maxBlue = 0;
        double maxGreen = 0;
        for (int i = 0; i <= 10; i++){
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            red = scale*colors.red;
            blue = scale*colors.blue;
            green = scale*colors.green;
//            telemetry.addData("Red: ", red);
//            telemetry.addData("Blue: ", blue);
//            telemetry.addData("Green: ", green);
            if (red > maxRed) {
                maxRed = red;
            }
            if (blue > maxBlue) {
                maxBlue = blue;
            }
            if (green > maxGreen) {
                maxGreen = green;
            }
        }
        if (((maxRed+maxGreen)/3.5)>maxBlue) {
            return true;
        } else {
            return false;
        }
    }

    public void up() {
        servoFlipper1.setPosition(SERVO_DOWN);
        servoFlipper2.setPosition(SERVO_UP);
    }

    public void down() {
        servoFlipper1.setPosition(0);
        servoFlipper2.setPosition(210/280);
    }

    public void pause() {
        gyroHold(0,0,.3);
    }

    public void unhook(int value) {
        motorLift.setTargetPosition(value);
        motorLift.setPower(1);
        while (opModeIsActive() && motorLift.isBusy()) {
            telemetry.addData("Encoder", motorLift.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorLift.setPower(0);
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        if (opModeIsActive()) {

            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = motorLeft.getCurrentPosition() + moveCounts;
            newRightTarget = motorRight.getCurrentPosition() + moveCounts;

            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);

            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motorLeft.setPower(speed);
            motorRight.setPower(speed);

            while (opModeIsActive() &&
                    (motorLeft.isBusy() && motorRight.isBusy())) {

                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                motorLeft.setPower(leftSpeed);
                motorRight.setPower(rightSpeed);

                telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("LEFT POSITION", motorLeft.getCurrentPosition());
                telemetry.addData("RIGHT POSITION", motorRight.getCurrentPosition());
                telemetry.update();
            }

            motorLeft.setPower(0);
            motorRight.setPower(0);

            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroTurn (  double speed, double angle) {

        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
        }
        motorHDrive.setPower(0);
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);

    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        motorLeft.setPower(leftSpeed);
        motorRight.setPower(rightSpeed);

        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}