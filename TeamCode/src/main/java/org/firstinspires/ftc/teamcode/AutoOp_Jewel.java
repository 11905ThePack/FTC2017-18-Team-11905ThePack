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
import com.qualcomm.robotcore.hardware.GyroSensor;
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
    ElapsedTime totaleTime = new ElapsedTime();

    DcMotor DriveLeftFront;
    DcMotor DriveLeftRear;
    DcMotor DriveRightFront;
    DcMotor DriveRightRear;

    DcMotor MotorGlyphGrabber;

    private Servo JewelWhackerServo; //Jewel Whacker Servo
    private double servoJewelWhackerServoPosition = 0;

    private Servo GlyphServoLeft; //Left half of glyph grabber
    private double servoGlyphLeftPosition = 180;
    private Servo GlyphServoRight; //Right half of glyph grabber
    private double servoGlyphRightPosition = 0;

    TouchSensor TeamBlueSwitch;
    GyroSensor Gyro;


    @Override
    public void runOpMode() {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        JewelWhackerColorSensor = hardwareMap.get(ColorSensor.class, "JewelWhackerColorSensor");
        JewelWhackerServo = hardwareMap.get(Servo.class,"JewelWhackerServo");

        MotorGlyphGrabber = hardwareMap.get(DcMotor.class, "MotorGlyphGrabber");
        MotorGlyphGrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGlyphGrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GlyphServoLeft = hardwareMap.get(Servo.class, "GlyphServoLeft");
        GlyphServoRight = hardwareMap.get(Servo.class, "GlyphServoRight");


        TeamBlueSwitch = hardwareMap.get(TouchSensor.class,"TeamBlueSwitch");
        boolean TeamBlueSwitchRead = TeamBlueSwitch.isPressed();

        Gyro = hardwareMap.get(GyroSensor.class, "Gyro");

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

        GlyphServoLeft.setPosition(servoGlyphLeftPosition / 180);
        GlyphServoRight.setPosition(servoGlyphRightPosition / 180);

        Gyro.calibrate();
        telemetry.addData("Glyph GrabberPos", MotorGlyphGrabber.getCurrentPosition());
        telemetry.update();

        // wait for the start button to be pressed.
        waitForStart();
        totaleTime.reset();

        servoGlyphLeftPosition = 45;
        GlyphServoLeft.setPosition(servoGlyphLeftPosition / 180);
        servoGlyphRightPosition = 135;
        GlyphServoRight.setPosition(servoGlyphRightPosition / 180);
        sleep(500); //Wait for Servos

        while (MotorGlyphGrabber.getCurrentPosition() < 500){
            MotorGlyphGrabber.setPower(.25);
            telemetry.addData("Glyph GrabberPos", MotorGlyphGrabber.getCurrentPosition());
            telemetry.update();
        }
            MotorGlyphGrabber.setPower(0);

        servoJewelWhackerServoPosition = 95;
        JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180); //This converts from degrees into 0-1 automagically

        eTime2.reset();
        while (eTime2.time() < 1) {}

        servoJewelWhackerServoPosition = 117;
        JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180); //This converts from degrees into 0-1 automagically.

        boolean GlyphNotRead = true;
        while (opModeIsActive()) {
            while (GlyphNotRead) {

                TeamBlueSwitchRead = TeamBlueSwitch.isPressed();

                //Rotate Towards Read Jewel
                if (((JewelWhackerColorSensor.red() >= 2) && (!TeamBlueSwitchRead)) || ((JewelWhackerColorSensor.blue() >= 2) && (TeamBlueSwitchRead))) {

                    telemetry.addData("ConsoleOut", "Attack!");

                    GlyphNotRead = false;

                    while (Gyro.getHeading() != 345){
                        DriveLeftFront.setPower(-0.1);
                        DriveLeftRear.setPower(-0.1);
                        DriveRightFront.setPower(0.1);
                        DriveRightRear.setPower(0.1);}

                    DriveLeftFront.setPower(0);
                    DriveLeftRear.setPower(0);
                    DriveRightFront.setPower(0);
                    DriveRightRear.setPower(0);

                    servoJewelWhackerServoPosition = 10;
                    JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180);

                    eTime.reset();
                    while (eTime.time() < .5) {}

                    while (Gyro.getHeading() != 0){
                        DriveLeftFront.setPower(0.1);
                        DriveLeftRear.setPower(0.1);
                        DriveRightFront.setPower(-0.1);
                        DriveRightRear.setPower(-0.1);}

                    DriveLeftFront.setPower(0);
                    DriveLeftRear.setPower(0);
                    DriveRightFront.setPower(0);
                    DriveRightRear.setPower(0);


                } else if (((JewelWhackerColorSensor.blue() >= 2) && (!TeamBlueSwitchRead)) || ((JewelWhackerColorSensor.red() >= 2) && (TeamBlueSwitchRead))) {

                    telemetry.addData("ConsoleOut", "Retreat!");

                    GlyphNotRead = false;

                    while (Gyro.getHeading() != 15){
                    DriveLeftFront.setPower(0.1);
                    DriveLeftRear.setPower(0.1);
                    DriveRightFront.setPower(-0.1);
                    DriveRightRear.setPower(-0.1);}

                    DriveLeftFront.setPower(0);
                    DriveLeftRear.setPower(0);
                    DriveRightFront.setPower(0);
                    DriveRightRear.setPower(0);

                    servoJewelWhackerServoPosition = 10;
                    JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180);

                    eTime.reset();
                    while (eTime.time() < .5) {}

                    while (Gyro.getHeading() != 0){
                        DriveLeftFront.setPower(-0.1);
                        DriveLeftRear.setPower(-0.1);
                        DriveRightFront.setPower(0.1);
                        DriveRightRear.setPower(0.1);}

                    DriveLeftFront.setPower(0);
                    DriveLeftRear.setPower(0);
                    DriveRightFront.setPower(0);
                    DriveRightRear.setPower(0);
                }

                // send the info back to driver station using telemetry function.
                telemetry.addData("Clear", JewelWhackerColorSensor.alpha());
                telemetry.addData("Red", JewelWhackerColorSensor.red());
                telemetry.addData("Green", JewelWhackerColorSensor.green());
                telemetry.addData("Blue", JewelWhackerColorSensor.blue());
                telemetry.addData("GyroPos", Gyro.getHeading());

                if (TeamBlueSwitchRead) {
                    telemetry.addData("TeamBlueSwitch", "Switch ON");
                } else {
                    telemetry.addData("TeamBlueSwitch", "Switch OFF");
                }

                telemetry.update();

            }
            JewelWhackerColorSensor.enableLed(false);
            telemetry.addData("ConsoleOut", "Job's Done");
            telemetry.update();

            //Next Phase of Autonomous











            while (totaleTime.time() < 30) {
            telemetry.addData("ConsoleOut", "Finished, Wait for end.");
            telemetry.update();
            }
        }
    }
}