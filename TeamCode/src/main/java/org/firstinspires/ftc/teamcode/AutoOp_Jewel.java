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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "Jewlely Thing", group = "Sensor")
//@Disabled
public class AutoOp_Jewel extends LinearOpMode {

    ColorSensor JewelWhackerColorSensor;

    ElapsedTime eTime = new ElapsedTime();

    ElapsedTime eTime2 = new ElapsedTime();

    DcMotor DriveLeftFront;
    DcMotor DriveLeftRear;

    DcMotor DriveRightFront;
    DcMotor DriveRightRear;


    private Servo JewelWhackerServo; //Jewel Whacker Servo
    private Servo GlyphServoLeft; //Left half of glyph grabber

    TouchSensor TeamBlueSwitch;

    private double servoJewelWhackerServoPosition = 0;
    private double servoGlyphLeftPosition = 180;

    @Override
    public void runOpMode() {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        JewelWhackerColorSensor = hardwareMap.get(ColorSensor.class, "JewelWhackerColorSensor");

        TeamBlueSwitch = hardwareMap.get(TouchSensor.class,"TeamBlueSwitch");
        boolean TeamBlueSwitchRead = TeamBlueSwitch.isPressed();


        JewelWhackerServo = hardwareMap.get(Servo.class,"JewelWhackerServo");

        DriveLeftFront = hardwareMap.get(DcMotor.class,"DriveLeftFront");
        DriveLeftFront.setDirection(DcMotor.Direction.REVERSE);
        DriveLeftRear = hardwareMap.get(DcMotor.class,"DriveLeftRear");
        DriveLeftRear.setDirection(DcMotor.Direction.REVERSE);
        DriveRightFront = hardwareMap.get(DcMotor.class,"DriveRightFront");
        DriveRightRear = hardwareMap.get(DcMotor.class,"DriveRightRear");

        // Set the LED in the beginning
        JewelWhackerColorSensor.enableLed(true);

        servoJewelWhackerServoPosition = 10;
        JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180); //This converts from degrees into 0-1 automagically.


        // wait for the start button to be pressed.
        waitForStart();

        servoJewelWhackerServoPosition = 95;
        JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180); //This converts from degrees into 0-1 automagically.

        while (opModeIsActive()) {

            TeamBlueSwitchRead = TeamBlueSwitch.isPressed();
            //If this reads true, red team.
            //If this reads false, blue team.


            eTime2.reset();
            while (eTime2.time() < 1) {}

            servoJewelWhackerServoPosition = 117;
            JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180); //This converts from degrees into 0-1 automagically.

            // send the info back to driver station using telemetry function.
            telemetry.addData("Clear", JewelWhackerColorSensor.alpha());
            telemetry.addData("Red  ", JewelWhackerColorSensor.red());
            telemetry.addData("Green", JewelWhackerColorSensor.green());
            telemetry.addData("Blue ", JewelWhackerColorSensor.blue());
            telemetry.addData("TeamBlueSwitch", TeamBlueSwitchRead);

            //Rotate Towards Read Jewel
            if (((JewelWhackerColorSensor.red() >= 2) && (TeamBlueSwitchRead)) || ((JewelWhackerColorSensor.blue() >= 2) && (!TeamBlueSwitchRead))) {

                telemetry.addData("ConsoleOut", "Rotate Away!");

                DriveLeftFront.setPower(-0.1);
                DriveLeftRear.setPower(-0.1);
                DriveRightFront.setPower(0.1);
                DriveRightRear.setPower(0.1);

                eTime.reset();
                while (eTime.time() < 1) {
                }

                DriveLeftFront.setPower(0);
                DriveLeftRear.setPower(0);
                DriveRightFront.setPower(0);
                DriveRightRear.setPower(0);


            }
            if (((JewelWhackerColorSensor.blue() >= 2) && (TeamBlueSwitchRead)) || ((JewelWhackerColorSensor.red() >= 2) && (!TeamBlueSwitchRead))) {

                telemetry.addData("ConsoleOut", "Attack!!!!");

                DriveLeftFront.setPower(0.1);
                DriveLeftRear.setPower(0.1);
                DriveRightFront.setPower(-0.1);
                DriveRightRear.setPower(-0.1);

                eTime.reset();
                while (eTime.time() > 1) {}

                DriveLeftFront.setPower(0);
                DriveLeftRear.setPower(0);
                DriveRightFront.setPower(0);
                DriveRightRear.setPower(0);
            }

            TeamBlueSwitchRead = TeamBlueSwitch.isPressed();
            telemetry.addData("ConsoleOut", "Not Read/Other Color");
            telemetry.update();

        }
        JewelWhackerColorSensor.enableLed(false);
        telemetry.addData("ConsoleOut", "Job's Done");
        telemetry.update();
    }

}