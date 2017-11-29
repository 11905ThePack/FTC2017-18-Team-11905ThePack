package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import static android.os.SystemClock.sleep;


@TeleOp(name="TankDrive", group="Drive-Type OpModes")

public class Tank_Drive extends OpMode
{
    // Declare OpMode variables for use.
    //All servo variables are in DEGREES.
    //As a general rule, use uppercase first letters for hardware mapping,
    //and use lowercase first letters for variables.
    private ElapsedTime runtime = new ElapsedTime(); //We don't really need an elapsedtime telemetry, but here it is.
    private DcMotor DriveMotorLeft = null; //Left Motor
    private DcMotor DriveMotorRight = null; //Right Motor
    //private DcMotor MotorRelicExtension = null; //Relic Extension Motor
    private Servo GlyphServoLeft = null; //Left half of glyph grabber
    private Servo GlyphServoRight = null; //Right half of glyph grabber
    private Servo RelicServoFront = null; //Front half of the Relic Grabber
    private Servo RelicServoBack = null; //Back half of the Relic Grabber


    //These outline the starting positions of all of the servos, as well as the range they're allowed to work in.
    //This is in degrees.
    private double servoGlyphLeftPosition = 180 ;
    private double servoGlyphRightPosition = 0;
    private double servoRelicServoFrontPosition = 45;
    private double servoRelicServoBackPosition = 80;
    private final static double servoMinRange  = 1;
    private final static double servoMaxRange  = 180;

    @Override
    public void init() {
        // Code to run ONCE when the driver hits INIT

        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DriveMotorLeft = hardwareMap.get(DcMotor.class, "DriveMotorLeft");
        DriveMotorRight = hardwareMap.get(DcMotor.class, "DriveMotorRight");
        GlyphServoLeft = hardwareMap.get(Servo.class, "GlyphServoLeft");
        GlyphServoRight = hardwareMap.get(Servo.class, "GlyphServoRight");

        //MotorRelicExtension = hardwareMap.get(DcMotor.class, "MotorRelicExtension");
        RelicServoFront = hardwareMap.get(Servo.class, "RelicServoFront");
        RelicServoBack = hardwareMap.get(Servo.class, "RelicServoBack");

        DriveMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveMotorRight.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
        //Code loops once you hit init.

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
        double DriveMotorLeftPower;
        double DriveMotorRightPower;

        // Init some local variables.
        double motorSpeedMultiplier = 1;
        String consoleOut = "Nothing Yet";


        // Tank Mode uses one stick to control each wheel. This is on Controller One.
         DriveMotorLeftPower  = -gamepad1.left_stick_y * motorSpeedMultiplier;
         DriveMotorRightPower = -gamepad1.right_stick_y * motorSpeedMultiplier ;

        // Use gamepad Y & A to set Servo's variables. This is on Controller Two.
        if (gamepad2.a) {
            servoGlyphLeftPosition = 10;
            servoGlyphRightPosition = 150;
        }
        if (gamepad2.y) {
            servoGlyphLeftPosition = 150;
            servoGlyphRightPosition = 10;

        }

        //Use "Up" on the DPad to trigger the full extension of the relic grabber. (This will later not work if runTime isn't high enough,
        //because you're not allowed  to move the relic outside of the field until endgame.)
        if (gamepad1.dpad_up)
            consoleOut = "Extending Relic Grabber To Maximum Length";



        // Use gamepad X & B to set Servo's Variables. This is on Controller Two.
        if (gamepad2.x) {
            servoRelicServoFrontPosition = 160;
            servoRelicServoBackPosition = 15;
        }

        if (gamepad2.b) {
            servoRelicServoBackPosition = 15;
            servoRelicServoFrontPosition = 160;
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


        // Send calculated power to wheels (There aren't any calculations done, this is pretty much extra at the moment.)
        DriveMotorLeft.setPower(DriveMotorLeftPower);
        DriveMotorRight.setPower(DriveMotorRightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Running, Run Time: " + runtime.toString());
        telemetry.addData("Motors", "Left: (%.2f), Right: (%.2f)", DriveMotorLeftPower, DriveMotorRightPower);
        //telemetry.addData("Servo Positions (Degrees)","%.2f", servoGlyphLeftPosition, servoGlyphRightPosition); (This is temporarily broken, but we'll fix it)
        telemetry.addData( "Motor Speed","%.2f", motorSpeedMultiplier);
        telemetry.addData( "Console Out", consoleOut);

        //sleep(50); //This saves battery by only running opMode 20 times a second. This is one Minecraft tick.
    }


    @Override
    public void stop() {
        //Code to run ONCE after the driver hits STOP.

        telemetry.addData("Status:", "Stopped");

    }

}
