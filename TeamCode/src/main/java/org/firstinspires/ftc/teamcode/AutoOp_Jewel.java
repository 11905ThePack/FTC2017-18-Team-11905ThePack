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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

    DcMotor DriveLeftRear;
    DcMotor DriveRightRear;
    DcMotor DriveLeftFront;
    DcMotor DriveRightFront;

    private Servo JewelWhackerServo = null; //Jewel Whacker Servo
    private Servo GlyphServoLeft = null; //Left half of glyph grabber

    private double servoJewelWhackerServoPosition = 5;
    private double servoGlyphLeftPosition = 180;
    private final static double servoMinRange  = 1;  //copied from teleop, probably not useful here
    private final static double servoMaxRange  = 180;//copied from teleop, probably not useful here


    @Override
    public void runOpMode() {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our ColorSensor object.
        JewelWhackerColorSensor = hardwareMap.get(ColorSensor.class, "JewelWhackerColorSensor");
        JewelWhackerServo = hardwareMap.get(Servo.class,"JewelWhackerServo");

        DriveLeftRear = hardwareMap.get(DcMotor.class,"DriveLeftRear");
        DriveRightRear = hardwareMap.get(DcMotor.class,"DriveRightRear");
        DriveLeftFront = hardwareMap.get(DcMotor.class,"DriveLeftFront");
        DriveRightFront = hardwareMap.get(DcMotor.class,"DriveRightFront");

        // Set the LED in the beginning
        JewelWhackerColorSensor.enableLed(bLedOn);

        servoJewelWhackerServoPosition = 20;
        JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180); //This converts from degrees into 0-1 automagically.


        // wait for the start button to be pressed.
        waitForStart();

        servoJewelWhackerServoPosition = 95;
        JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180); //This converts from degrees into 0-1 automagically.

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {


            servoGlyphLeftPosition = 70;
            GlyphServoLeft.setPosition(servoGlyphLeftPosition / 180); //This converts from degrees into 0-1 automagically.


            eTime2.reset();
            while (eTime2.time() < 1) {}
            bCurrState = true;

            //servoJewelWhackerServoPosition = 40;
            //JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180); //This converts from degrees into 0-1 automagically.


            servoGlyphLeftPosition = 40;
            GlyphServoLeft.setPosition(servoGlyphLeftPosition / 180); //This converts from degrees into 0-1 automagically.


            eTime2.reset();
            while (eTime2.time() < 1) {}

            // sensor state transition
            if (bCurrState && (bCurrState != bPrevState)) {

                bLedOn = !bLedOn;
                JewelWhackerColorSensor.enableLed(bLedOn);
            }

            // update previous state variable.
            //bPrevState = bCurrState;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(JewelWhackerColorSensor.red() * 8, JewelWhackerColorSensor.green() * 8, JewelWhackerColorSensor.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", JewelWhackerColorSensor.alpha());
            telemetry.addData("Red  ", JewelWhackerColorSensor.red());
            telemetry.addData("Green", JewelWhackerColorSensor.green());
            telemetry.addData("Blue ", JewelWhackerColorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });


            sleep(500);

            if (JewelWhackerColorSensor.red() >= 2) {

                telemetry.addData("Bleh", "Red");


                while (eTime.time()>2) {
                    DriveLeftFront.setPower(0.1);
                    DriveLeftFront.setPower(0.1);
                    DriveLeftFront.setPower(-0.1);
                    DriveLeftFront.setPower(-0.1);

                }
                eTime.reset();

                while (eTime.time()>2) {
                    DriveLeftFront.setPower(0);
                    DriveLeftFront.setPower(0);
                    DriveLeftFront.setPower(0);
                    DriveLeftFront.setPower(0);

                }
                eTime.reset();

            } else if (JewelWhackerColorSensor.blue() >= 2){
                telemetry.addData("Bleh", "blue");

                while (eTime.time()>2) {
                    DriveLeftFront.setPower(-0.1);
                    DriveLeftFront.setPower(-0.1);
                    DriveLeftFront.setPower(0.1);
                    DriveLeftFront.setPower(0.1);

                }
                eTime.reset();

                while (eTime.time()>2) {
                    DriveLeftFront.setPower(0);
                    DriveLeftFront.setPower(0);
                    DriveLeftFront.setPower(0);
                    DriveLeftFront.setPower(0);

                }
                eTime.reset();

            } else {
                telemetry.addData("Bleh", "other color");

            }

            telemetry.update();

        }


        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}