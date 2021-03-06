package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOP", group="Iterative Opmode")
public class TeleOP extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    //  TODO Constants
    static final double SERVO_DOWN = 0/280.0;
    static final double SERVO_UP = 210/280.0;

    //  TODO Motors
    private DcMotor motorLeft = null;
    private DcMotor motorRight = null;
    private DcMotor motorLift = null;
    private DcMotor motorVertical = null;
    private DcMotor motorHorizontal = null;
    private DcMotor motorIntake = null;
    private DcMotor motorHDrive = null;
    private ColorSensor colorSensor = null;

    //  TODO Servos
    private Servo servoFlipper1 = null;
    private Servo servoFlipper2 = null;

    //  TODO Joystick Mapping
    double factor = .8;
    double L = 2.10;
    double p = 3;
    double A = 2;
    double s = 3.3;
    double B = 7.8;
    double y = .30515;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

//      TODO Hardware Maps
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

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(false);

        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

//      TODO Controls
//      Drivetrain
        motorLeft.setPower(powerCurve(gamepad1.left_stick_y));
        motorRight.setPower(powerCurve(gamepad1.right_stick_y));
        if(gamepad1.left_bumper) {
            motorHDrive.setPower(-1);
        } else if (gamepad1.right_bumper) {
            motorHDrive.setPower(1);
        } else {
            motorHDrive.setPower(0);
        }

//      Lift TODO Encoders for lift
        if(gamepad2.x) {
            motorLift.setPower(1);
        } else if (gamepad2.y) {
            motorLift.setPower(-.8);
        } else {
            motorLift.setPower(0);
        }

//      Intake
        if(gamepad1.left_trigger != 0) {
            motorIntake.setPower(.8*Math.pow(gamepad1.left_trigger, 2));
        } else if (gamepad1.right_trigger != 0) {
            motorIntake.setPower(-8*Math.pow(gamepad1.right_trigger, 2));
        } else {
            motorIntake.setPower(0);
        }

//      Slides
        motorHorizontal.setPower(-gamepad2.right_stick_y);
        motorVertical.setPower(-gamepad2.left_stick_y);

//      Flipper
        if (gamepad2.right_bumper) {
            servoFlipper1.setPosition(SERVO_DOWN);
            servoFlipper2.setPosition(SERVO_UP);
        }
        if (gamepad2.left_bumper) {
            servoFlipper1.setPosition(SERVO_UP);
            servoFlipper2.setPosition(SERVO_DOWN);
        }

//      TODO Telemetry
        telemetry.addData("Diego is not very awesome", gamepad1.left_stick_y*1000);
        telemetry.addData("very very awesome", gamepad1.right_stick_y*1000);
        telemetry.update();
    }

    @Override
    public void stop() {
    }

    public double logisticGrowth(double x) {
        if(x > 0) {
            return (L/(p+(A*Math.exp(s-(B*x)))))+y;
        } else if (x < 0){
            return -((L/(p+(A*Math.exp(s+(B*x)))))+y);
        } else {
            return 0;
        }
    }

    public double powerCurve(double x) {
        if(x > 0) {
            return .8*Math.pow(x, 2);
        } else if (x < 0){
            return -.8*Math.pow(x, 2);
        } else {
            return 0;
        }

    }



}