package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TankDrive", group="Drive-Type OpModes")

public class Tank_Drive extends OpMode
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

    private DcMotor MotorRelicExtension = null; //Relic Extension Motor

    private Servo GlyphServoLeft = null; //Left half of glyph grabber
    private Servo GlyphServoRight = null; //Right half of glyph grabber
    private Servo RelicServoFront = null; //Front half of the Relic Grabber
    private Servo RelicServoBack = null; //Back half of the Relic Grabber
    private Servo RelicServoPitch = null; //Relic Grabber Rotation

    private DeviceInterfaceModule DeviceIM;

    //These outline the starting positions of all of the servos, as well as the range they're allowed to work in.
    //This is in degrees.
    private double servoGlyphLeftPosition = 180 ;
    private double servoGlyphRightPosition = 0;
    private double servoRelicServoFrontPosition = 45;
    private double servoRelicServoBackPosition = 80;
    private double servoRelicServoPitchPosition = 0;
    private final static double servoMinRange  = 1;
    private final static double servoMaxRange  = 180;
    private double motorSpeedMultiplier = .6;


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

        GlyphServoLeft = hardwareMap.get(Servo.class, "GlyphServoLeft");
        GlyphServoRight = hardwareMap.get(Servo.class, "GlyphServoRight");

        RelicServoPitch = hardwareMap.get(Servo.class, "RelicServoPitch");
        RelicServoFront = hardwareMap.get(Servo.class, "RelicServoFront");
        RelicServoBack = hardwareMap.get(Servo.class, "RelicServoBack");

        DeviceIM = hardwareMap.get(DeviceInterfaceModule.class, "DIM");


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
    }


    @Override
    public void loop() {
        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

        // Setup a variable for each drive wheel and servos to save power level for telemetry.
        double RelicGrabberExtensionPos = MotorRelicExtension.getCurrentPosition();

        // Init some local variables.
        String consoleOut = "Nothing Yet";

        //Gamepad 1 Controls
        //POV Mode originally written by dmssargent, sourced from the FTC Forum.
        //However, there are multiple modifications that were made to make the translation more intuitive.
        //POV Mode. One stick controls translation and one controls rotation.
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        if (gamepad1.a) {
            motorSpeedMultiplier = .6;
            DeviceIM.setLED(1, false);
        }

        if (gamepad1.b) {
            motorSpeedMultiplier = .1;
            DeviceIM.setLED(1, true);
        }

        //Gamepad 2 Controls
        if (gamepad2.dpad_up) {
            consoleOut = "Extending Relic Grabber To Maximum Length";
            MotorRelicExtension.setTargetPosition(35);
            MotorRelicExtension.setPower(.2);
        }

        if (gamepad2.dpad_down) {
            consoleOut = "Stopped Relic Grabber Extension";
            MotorRelicExtension.setPower(0);
            RelicServoPitch.setPosition(30);
        }

        if (gamepad2.dpad_left) {
            consoleOut = "Returning Relic Extension";
            MotorRelicExtension.setTargetPosition(0);
            MotorRelicExtension.setPower(-.2);
        }

        if (gamepad2.x) {
            servoRelicServoFrontPosition = 160;
            servoRelicServoBackPosition = 15;
        }

        if (gamepad2.y) {
            servoRelicServoBackPosition = 15;
            servoRelicServoFrontPosition = 160;
        }

        if (gamepad2.b) {
            servoGlyphLeftPosition = 90;
            servoGlyphRightPosition = 90;
        }

        if (gamepad2.a) {
            servoGlyphLeftPosition = 80;
            servoGlyphRightPosition = 100;
        }


        // Set Servo position to variable "servoPosition"
        servoGlyphLeftPosition = Range.clip(servoGlyphLeftPosition, servoMinRange, servoMaxRange); //Clips servo range into usable area. Protects from over extension.
        GlyphServoLeft.setPosition(servoGlyphLeftPosition / 180); //This converts from degrees into 0-1 automagically.

        servoGlyphRightPosition = Range.clip(servoGlyphRightPosition, servoMinRange, servoMaxRange);
        GlyphServoRight.setPosition(servoGlyphRightPosition / 180);

        servoRelicServoFrontPosition = Range.clip(servoRelicServoFrontPosition, servoMinRange, servoMaxRange);
        RelicServoFront.setPosition(servoRelicServoFrontPosition / 180);

        servoRelicServoBackPosition = Range.clip(servoRelicServoBackPosition, servoMinRange, servoMaxRange);
        RelicServoBack.setPosition(servoRelicServoBackPosition / 180);

        servoRelicServoPitchPosition = Range.clip(servoRelicServoPitchPosition, servoMinRange, servoMaxRange);
        RelicServoPitch.setPosition(servoRelicServoPitchPosition / 180);


        // Send calculated power to wheels.
        DriveLeftFront.setPower(v1 * motorSpeedMultiplier);
        DriveRightFront.setPower(v2 * motorSpeedMultiplier);
        DriveLeftRear.setPower(v3 * motorSpeedMultiplier);
        DriveRightRear.setPower(v4 * motorSpeedMultiplier);

        //This code will prevent the Relic Extender from hurting itself.
        //if RelicGrabberExtensionPos = 720{

        //}

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Running, Run Time: " + runtime.toString());
        //telemetry.addData("FrontMotors:", "%.2f", DriveLeftFront, DriveRightFront);
        //telemetry.addData("RearMotors:", "%.2f", DriveLeftRear, DriveRightRear);
        //telemetry.addData("RelicGrabberExtensionPos", "Position: %.2f", RelicGrabberExtensionPos);
        telemetry.addData( "Motor Speed","%.2f", motorSpeedMultiplier);
        telemetry.addData( "Console Out", consoleOut);
    }


    @Override
    public void stop() {
        //Code to run ONCE after the driver hits STOP.

        telemetry.addData("Status:", "Stopped");

    }

}
