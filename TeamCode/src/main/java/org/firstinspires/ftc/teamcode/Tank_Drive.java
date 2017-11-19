package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;


@TeleOp(name="TankDrive", group="Drive-Type OpModes")
//@Disabled
public class Tank_Drive extends OpMode
{
    // Declare OpMode variables for use.
    //All servo variables are in DEGREES.
    //As a general rule, use uppercase first letters for hardware mapping,
    //and use lowercase first letters for variables.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor DriveMotorLeft = null;
    private DcMotor DriveMotorRight = null;
    private Servo GlyphServoLeft = null; //Left half of glyph grabber
    private Servo GlyphServoRight = null; //Right half of glyph grabber

    private static double motorSpeedMultiplier = 1;

    private double servoGlyphLeftPosition = 180 ;  // Servo safe position
    private double servoGlyphRightPosition = 0;  //Servo safe position
    private final static double servoMinRange  = 1; //These should probably be removed at some point.
    private final static double servoMaxRange  = 180;

    private String consoleOut = "Nothing Yet";


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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor t//hat runs backwards when connected directly to the battery
        DriveMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveMotorRight.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
        //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

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

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        //DriveMotorLeftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //DriveMotorRightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
         DriveMotorLeftPower  = -gamepad1.left_stick_y * motorSpeedMultiplier;
         DriveMotorRightPower = -gamepad1.right_stick_y * motorSpeedMultiplier ;

        //MotorSpeed High/Low This is on controller one.
        if (gamepad1.a)
            motorSpeedMultiplier = 1;
        else if (gamepad1.b)
            motorSpeedMultiplier = .2; //20% Speed

        // Use gamepad Y & A set Servo's variables. This is on controller two.
        if (gamepad2.a) {
            servoGlyphLeftPosition = 10;
            servoGlyphRightPosition = 150;
        }
        if (gamepad2.y) {
            servoGlyphLeftPosition = 150;
            servoGlyphRightPosition = 10;
        }
        // Set Servo position to variable "servoPosition"
        servoGlyphLeftPosition = Range.clip(servoGlyphLeftPosition, servoMinRange, servoMaxRange); //Clips servo range into usable area. Protects from over extension.
        GlyphServoLeft.setPosition(servoGlyphLeftPosition / 180); //This converts from degrees into 0-1 automagically.
        servoGlyphRightPosition = Range.clip(servoGlyphRightPosition, servoMinRange, servoMaxRange);//Clips servo range into usable area. Protects from over extension.
        GlyphServoRight.setPosition(servoGlyphRightPosition / 180); //This converts from degrees into 0-1 automagically.

        // Send calculated power to wheels (There aren't any calculations done, this is pretty much extra at the moment.)
        DriveMotorLeft.setPower(DriveMotorLeftPower);
        DriveMotorRight.setPower(DriveMotorRightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Running, Run Time: " + runtime.toString());
        telemetry.addData("Motors", "Left: (%.2f), Right: (%.2f)", DriveMotorLeftPower, DriveMotorRightPower);
//        telemetry.addData("Servo Positions (Degrees)","%.2f", servoGlyphLeftPosition, servoGlyphRightPosition); (This is temporarily broken, but we'll fix it)
        telemetry.addData( "Motor Speed","%.2f", motorSpeedMultiplier);
        telemetry.addData( "Console Out", consoleOut);



        sleep(50); //This saves battery by only running opMode 20 times a second. This is one Minecraft tick.
    }


    @Override
    public void stop() {
        //Code to run ONCE after the driver hits STOP.

        telemetry.addData("Status:", "Stopped");

    }

}
