package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MainTeleOp", group="Drive-Type OpModes")

public class MainTeleOp extends OpMode
{
    // Declare OpMode variables for use.
    //All servo variables are in DEGREES.
    //As a general rule, use uppercase first letters for hardware mapping,
    //and use lowercase first letters for variables.
    private ElapsedTime runtime = new ElapsedTime(); //We don't really need an elapsed time telemetry, but here it is.
    private DcMotor DriveLeftFront = null; //Left Front Motor
    private DcMotor DriveRightFront = null; //Right Front Motor
    private DcMotor DriveLeftRear = null; //Left Rear Motor
    private DcMotor DriveRightRear = null; //Right Rear Motor

    private DcMotor MotorGlyphGrabber = null;
    private DcMotor MotorRelicExtension = null; //Relic Extension Motor

    private Servo GlyphServoLeft = null; //Left half of glyph grabber
    private Servo GlyphServoRight = null; //Right half of glyph grabber
    private Servo RelicServoFront = null; //Front half of the Relic Grabber
    private Servo RelicServoBack = null; //Back half of the Relic Grabber
    private Servo RelicServoPitch = null; //Relic Grabber Rotation
    private Servo JewelWhackerServo = null; //Jewel Whacker Servo


    private DeviceInterfaceModule DeviceIM;
    private GyroSensor Gyro = null; // Das ist die Gyro

    //These outline the starting positions of all of the servos, as well as the range they're allowed to work in.
    //This is in degrees.
    private double servoGlyphLeftPosition = 180;
    private double servoGlyphRightPosition = 0;
    private double servoRelicServoFrontPosition = 45;
    private double servoRelicServoBackPosition = 80;
    private double servoRelicServoPitchPosition = 180;
    private double servoJewelWhackerServoPosition = 5;
    private final static double servoMinRange  = 1;
    private final static double servoMaxRange  = 180;
    private double motorSpeedMultiplier = .2;
    private String consoleOut = "Nothing Yet";

    @Override
    public void init() {
        // Code to run ONCE when the driver hits INIT

        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DriveLeftFront = hardwareMap.get(DcMotor.class, "DriveLeftFront");
        DriveRightFront = hardwareMap.get(DcMotor.class, "DriveRightFront");
        DriveRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveLeftRear = hardwareMap.get(DcMotor.class, "DriveLeftRear");
        DriveRightRear = hardwareMap.get(DcMotor.class, "DriveRightRear");
        DriveRightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //Init MotorRelicExtension a.k.a "Le Booper o' Death"
        MotorRelicExtension = hardwareMap.get(DcMotor.class, "MotorRelicExtension");
        MotorRelicExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorRelicExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Init MotorGlyphGrabber
        MotorGlyphGrabber = hardwareMap.get(DcMotor.class, "MotorGlyphGrabber");
        MotorGlyphGrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGlyphGrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //InitGlyphServos
        GlyphServoLeft = hardwareMap.get(Servo.class, "GlyphServoLeft");
        GlyphServoRight = hardwareMap.get(Servo.class, "GlyphServoRight");

        //InitRelicServos
        RelicServoPitch = hardwareMap.get(Servo.class, "RelicServoPitch");
        RelicServoFront = hardwareMap.get(Servo.class, "RelicServoFront");
        RelicServoBack = hardwareMap.get(Servo.class, "RelicServoBack");

        //Init Misc Devices

        DeviceIM = hardwareMap.get(DeviceInterfaceModule.class, "DIM");
        Gyro = hardwareMap.get(GyroSensor.class, "Gyro");

        JewelWhackerServo = hardwareMap.get(Servo.class, "JewelWhackerServo");


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
        //Code loops once you hit init
        telemetry.addData("Status:", "Armed");

    }


    @Override
    public void start() {
        //Code to run ONCE when the driver hits PLAY
        runtime.reset();
        Gyro.calibrate();
    }

    private boolean Extending = false;
    private boolean Retracting = false;
    private boolean UseServoStick = true;
    private boolean DriveMode = false; //False is for Glyphs, True is for Relic
    @Override
    public void loop() {
        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

        // Setup a variable for each drive wheel and servos to save power level for telemetry.
        double RelicGrabberExtensionPos = MotorRelicExtension.getCurrentPosition();
        double MotorGlyphGrabberPos = MotorGlyphGrabber.getCurrentPosition();

        // Init some local variables.

        //Gamepad 1 Controls
        // Original POV Mode written by dmssargent, sourced from the FTC Forum. Modified for use with our robot.
        //POV Mode. One stick controls translation and one controls rotation.
        double r = Math.hypot((gamepad1.left_stick_x * 2.5), gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, (gamepad1.left_stick_x * 2.5)) - Math.PI / 4;
        double rightX = (gamepad1.right_stick_x * .7);
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        if (gamepad1.a) {
            motorSpeedMultiplier = .2;
            DeviceIM.setLED(1, false);
        }

        if (gamepad1.b) {
            motorSpeedMultiplier = .55;
            DeviceIM.setLED(1, true);
        }

        //Gamepad 2 Controls
        //There will be a button to disable the stick for controlling the servo.
        if (UseServoStick) {
            servoGlyphLeftPosition = 180 - gamepad2.right_stick_x * 180;
            servoGlyphRightPosition = 0 - -gamepad2.right_stick_x * 180;
        }

        double v5 = gamepad2.left_stick_y * 0.15;

        if (gamepad2.dpad_up) {
            consoleOut = "Extending Relic Grabber To Maximum Length";
            servoRelicServoPitchPosition = 150;
            Extending = true;
            Retracting = false;
        }

        if (gamepad2.dpad_down) {
            consoleOut = "Stopped Relic Grabber Extension/Retraction";
            Extending = false;
            Retracting = false;
        }

        if (gamepad2.dpad_left) {
            consoleOut = "Returning Relic Extension";
            Extending = false;
            Retracting = true;
        }

        if (gamepad2.dpad_right) {
            if (DriveMode){
                DriveMode = true;
                consoleOut = "Set Drive Mode to Relic Mode";
            } else {
                DriveMode = false;
                consoleOut = "Set Drive Mode to Glyph Mode";
            }
        }

        //Toggle the Right Stick controlling the glyph grabber servos.
        if (gamepad2.right_stick_button) {
            if (UseServoStick) {
                UseServoStick = false;
            } else UseServoStick = true;
        }

        if (gamepad2.left_bumper) {
            servoGlyphLeftPosition = 90;
            servoGlyphRightPosition = 90;
        }

        if (gamepad2.right_bumper) {
            servoGlyphLeftPosition = 70;
            servoGlyphRightPosition = 110;
        }

        if (gamepad2.a) {
            servoRelicServoPitchPosition = 0;
        }

        if (gamepad2.b) {
            servoRelicServoPitchPosition = 180;
        }

        if (gamepad2.x) {
            servoRelicServoFrontPosition = 145;
            servoRelicServoBackPosition = 90;
        }

        if (gamepad2.y) {
            servoRelicServoFrontPosition = 35;
            servoRelicServoBackPosition = 90;
        }

        // Set Servo positions to variable "servoPosition"(s)
        servoGlyphLeftPosition = Range.clip(servoGlyphLeftPosition, 0, 180); //Clips servo range into usable area. Protects from over extension.
        GlyphServoLeft.setPosition(servoGlyphLeftPosition / 180); //This converts from degrees into 0-1 automagically.

        servoGlyphRightPosition = Range.clip(servoGlyphRightPosition, 0, 180);
        GlyphServoRight.setPosition(servoGlyphRightPosition / 180);

        servoRelicServoFrontPosition = Range.clip(servoRelicServoFrontPosition, servoMinRange, servoMaxRange);
        RelicServoFront.setPosition(servoRelicServoFrontPosition / 180);

        servoRelicServoBackPosition = Range.clip(servoRelicServoBackPosition, servoMinRange, servoMaxRange);
        RelicServoBack.setPosition(servoRelicServoBackPosition / 180);

        servoRelicServoPitchPosition = Range.clip(servoRelicServoPitchPosition, servoMinRange, servoMaxRange);
        RelicServoPitch.setPosition(servoRelicServoPitchPosition / 180);

        servoJewelWhackerServoPosition = Range.clip(servoJewelWhackerServoPosition, servoMinRange, servoMaxRange);
        JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180);

        // Send calculated power to wheels
        DriveLeftFront.setPower(v1 * motorSpeedMultiplier);
        DriveRightFront.setPower(v2 * motorSpeedMultiplier);
        DriveLeftRear.setPower(v3 * motorSpeedMultiplier);
        DriveRightRear.setPower(v4 * motorSpeedMultiplier);
        MotorGlyphGrabber.setPower(v5);

        //Read Positions Of Motors + Gyro
        double DriveLeftFrontPos = DriveLeftFront.getCurrentPosition();
        double DriveRightFrontPos = DriveRightFront.getCurrentPosition();
        double DriveLeftRearPos = DriveLeftRear.getCurrentPosition();
        double DriveRightRearPos = DriveRightRear.getCurrentPosition();
        int GyroPos = Gyro.getHeading();

        //Automated Relic Extension/Retraction
        if (Extending) {
            if (MotorRelicExtension.getCurrentPosition() > 7600) {
                MotorRelicExtension.setPower(0);
            } else {
                MotorRelicExtension.setPower(.25);
            }
        }

        if (Retracting) {
            if (MotorRelicExtension.getCurrentPosition() > 0) {
                MotorRelicExtension.setPower(-.15);
            } else {
                MotorRelicExtension.setPower(0);
            }
        }





        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Running, Run Time: " + runtime.toString());

        telemetry.addData("FrontMotors", "Left: " + DriveLeftFrontPos);
        telemetry.addData("FrontMotors", "Right: " + DriveRightFrontPos);
        telemetry.addData("RearMotors", "Left: "+ DriveLeftRearPos);
        telemetry.addData("RearMotors", "Right: " + DriveRightRearPos);

        telemetry.addData("RelicGrabberExtensionPos", RelicGrabberExtensionPos);
        telemetry.addData("MotorGlyphGrabberPos", MotorGlyphGrabberPos);
        telemetry.addData("GyroPos:", GyroPos);
        telemetry.addData( "Motor Speed","%.2f", motorSpeedMultiplier);
        telemetry.addData( "Console Out", consoleOut);
    }


    @Override
    public void stop() {
        //Code to run ONCE after the driver hits STOP.

        telemetry.addData("Status:", "Stopped");

    }

}