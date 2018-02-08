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

@Autonomous(name = "Jewlely Thing", group = "Sensor")

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
    TouchSensor Position1Switch;
    GyroSensor Gyro;


    //encoderDrive constants
    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;  // For finding circumference
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_DEGREE =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) * 1.8/90; //this value is a guess
    //static final double DRIVE_SPEED = 0.3;
    //static final double TURN_SPEED = 0.5;

    private ElapsedTime runtime = new ElapsedTime(); //Elapsed Time
    //runtime is being used in encoderDrive


    @Override
    public void runOpMode() {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        JewelWhackerColorSensor = hardwareMap.get(ColorSensor.class, "JewelWhackerColorSensor");
        JewelWhackerServo = hardwareMap.get(Servo.class, "JewelWhackerServo");

        MotorGlyphGrabber = hardwareMap.get(DcMotor.class, "MotorGlyphGrabber");
        MotorGlyphGrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGlyphGrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GlyphServoLeft = hardwareMap.get(Servo.class, "GlyphServoLeft");
        GlyphServoRight = hardwareMap.get(Servo.class, "GlyphServoRight");


        TeamBlueSwitch = hardwareMap.get(TouchSensor.class, "TeamBlueSwitch");
        boolean TeamBlueSwitchRead = TeamBlueSwitch.isPressed();

        Position1Switch = hardwareMap.get(TouchSensor.class, "Position1Switch");
        boolean Position1SwitchRead = Position1Switch.isPressed();

        Gyro = hardwareMap.get(GyroSensor.class, "Gyro");

        DriveLeftFront = hardwareMap.get(DcMotor.class, "DriveLeftFront");
        DriveLeftFront.setDirection(DcMotor.Direction.REVERSE);
        DriveLeftRear = hardwareMap.get(DcMotor.class, "DriveLeftRear");
        DriveLeftRear.setDirection(DcMotor.Direction.REVERSE);
        DriveRightFront = hardwareMap.get(DcMotor.class, "DriveRightFront");
        DriveRightRear = hardwareMap.get(DcMotor.class, "DriveRightRear");

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

        TeamBlueSwitchRead = TeamBlueSwitch.isPressed();
        Position1SwitchRead = Position1Switch.isPressed();

        servoJewelWhackerServoPosition = 90;
        JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180); //This converts from degrees into 0-1 automagically

        eTime2.reset();
        while (eTime2.time() < 1) {
        }

        servoJewelWhackerServoPosition = 117;   // CHANGE THIS BACK TO 117  (or 30 works ok to disable it)
        JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180); //This converts from degrees into 0-1 automagically.


        boolean SkipJewel = true; //Skips the Process of knocking off the Jewel
        boolean JewelNotRead = true;  // CHANGE THIS BACK TO TRUE

        if (SkipJewel){
            JewelNotRead = false;
            servoJewelWhackerServoPosition = 30;
            JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180); //This converts from degrees into 0-1 automagically.
        }else
            {servoGlyphLeftPosition = 45;
            GlyphServoLeft.setPosition(servoGlyphLeftPosition / 180);
            servoGlyphRightPosition = 135;
            GlyphServoRight.setPosition(servoGlyphRightPosition / 180);
            sleep(500); //Wait for Servos

            MotorGlyphGrabber.setPower(.25);
            while (MotorGlyphGrabber.getCurrentPosition() < 500) {
                telemetry.addData("Glyph GrabberPos", MotorGlyphGrabber.getCurrentPosition());
                telemetry.update();
            }
            MotorGlyphGrabber.setPower(0);

        }

        while (opModeIsActive()) {

            while (JewelNotRead) {

                //Rotate Towards Read Jewel
                if (((JewelWhackerColorSensor.red() >= 2) && (!TeamBlueSwitchRead)) || ((JewelWhackerColorSensor.blue() >= 2) && (TeamBlueSwitchRead))) {

                    telemetry.addData("ConsoleOut", "Attack!");

                    JewelNotRead = false;

                    while (Gyro.getHeading() != 345) {
                        DriveLeftFront.setPower(-0.15);
                        DriveLeftRear.setPower(-0.15);
                        DriveRightFront.setPower(0.15);
                        DriveRightRear.setPower(0.15);
                    }

                    DriveLeftFront.setPower(0);
                    DriveLeftRear.setPower(0);
                    DriveRightFront.setPower(0);
                    DriveRightRear.setPower(0);

                    servoJewelWhackerServoPosition = 10;
                    JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180);

                    eTime.reset();
                    while (eTime.time() < .5) {
                    }

                    while (Gyro.getHeading() != 0) {
                        DriveLeftFront.setPower(0.15);
                        DriveLeftRear.setPower(0.15);
                        DriveRightFront.setPower(-0.15);
                        DriveRightRear.setPower(-0.15);
                    }

                    DriveLeftFront.setPower(0);
                    DriveLeftRear.setPower(0);
                    DriveRightFront.setPower(0);
                    DriveRightRear.setPower(0);


                } else if (((JewelWhackerColorSensor.blue() >= 2) && (!TeamBlueSwitchRead)) || ((JewelWhackerColorSensor.red() >= 2) && (TeamBlueSwitchRead))) {

                    telemetry.addData("ConsoleOut", "Retreat!");

                    JewelNotRead = false;

                    gyroRotate(270,10 );

                    while (Gyro.getHeading() != 15) {
                        DriveLeftFront.setPower(0.15);
                        DriveLeftRear.setPower(0.15);
                        DriveRightFront.setPower(-0.15);
                        DriveRightRear.setPower(-0.15);
                    }

                    DriveLeftFront.setPower(0);
                    DriveLeftRear.setPower(0);
                    DriveRightFront.setPower(0);
                    DriveRightRear.setPower(0);

                    servoJewelWhackerServoPosition = 10;
                    JewelWhackerServo.setPosition(servoJewelWhackerServoPosition / 180);

                    eTime.reset();
                    while (eTime.time() < .5) {
                    }

                    while (Gyro.getHeading() != 0) {
                        DriveLeftFront.setPower(-0.15);
                        DriveLeftRear.setPower(-0.15);
                        DriveRightFront.setPower(0.15);
                        DriveRightRear.setPower(0.15);
                    }

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
            telemetry.addData("ConsoleOut", "Jewel's Done");
            telemetry.update();

            //Next Phase of Autonomous

//            gyroRotate(90,10);
//            sleep(2000);

//            gyroRotate(-90,10);
//            sleep(1000);
            if (TeamBlueSwitchRead)  {   //blue team
               if (Position1SwitchRead)  {  //blue team, in position 1
                    telemetry.addData("ConsoleOut", "blue team in position 1");
                    telemetry.update();
                    sleep(2000);
               } else  {   //blue team, in position 2
                    telemetry.addData("ConsoleOut", "blue team in position 2");
                    telemetry.update();
                    sleep(2000);

               }
            }  else  {  //red team
                if (Position1SwitchRead)  {  //red team, in position 1
                    telemetry.addData("ConsoleOut", "red team in position 1");
                    telemetry.update();
                    sleep(2000);

                } else  {   //red team, in position 2
                    telemetry.addData("ConsoleOut", "red team in position 2");
                    telemetry.update();
                    sleep(2000);

                    encoderDriveStraight(-.20, -24, 5);
                    encoderDriveRotate(.15,90,5);
                    encoderDriveStraight(.20, 30, 5);
                    encoderDriveRotate(.15,90,5);
                    encoderDriveStraight(.20, 4, 5);

                }
            }


            //gyroRotate(-10,10);
            //sleep(1000);   // optional pause after each move
            //gyroRotate(10,10);
            //sleep(1000);   // optional pause after each move

            //encoderDriveStraight(-.20, -36, 10);
            //sleep(1000);   // optional pause after each move
            //gyroRotate(-90,6);
            //sleep(1000);   // optional pause after each move
//            gyroRotate(.15,90,6);
//            sleep(1000);   // optional pause after each move
//            gyroRotate(-.15,-180,12);
//            sleep(1000);   // optional pause after each move

//            encoderDriveRotate(.1, 90, 10);
//            encoderDriveStraight(-.20, -12, 10);
//            encoderDriveStraight(.20, 12, 10);

            /*
            MotorGlyphGrabber.setPower(-.20);
            while (MotorGlyphGrabber.getCurrentPosition() > 0) {
                telemetry.addData("Glyph GrabberPos", MotorGlyphGrabber.getCurrentPosition());
//                telemetry.update();
            }
            MotorGlyphGrabber.setPower(0);
            */


            while (totaleTime.time() < 30) {
                telemetry.addData("ConsoleOut", "Finished, Wait for end.");
//                telemetry.update();


            }
        }
    }


    public void encoderDrive(double speed,
                             double leftBackInches, double rightBackInches, double leftFrontInches, double rightFrontInches,
                             double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        //Ensures OpMode is running
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = DriveLeftRear.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = DriveRightRear.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);
            newLeftFrontTarget = DriveLeftFront.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = DriveRightFront.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);


            DriveLeftRear.setTargetPosition(newLeftBackTarget);
            DriveRightRear.setTargetPosition(newRightBackTarget);
            DriveLeftFront.setTargetPosition(newLeftFrontTarget);
            DriveRightFront.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("encoderDriveOut", "encoderDrive starting");
            telemetry.update();

            // reset the timeout time and start motion.
            runtime.reset();
//            DriveLeftRear.setPower(Math.abs(speed));   // running these as abs values won't allow reverse direction!
//            DriveRightRear.setPower(Math.abs(speed));
//            DriveLeftFront.setPower(Math.abs(speed));
//            DriveRightFront.setPower(Math.abs(speed));
            DriveLeftRear.setPower(speed + .05);
            DriveRightRear.setPower(speed);
            DriveLeftFront.setPower(speed + .05);
            DriveRightFront.setPower(speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (DriveLeftRear.isBusy() || DriveRightRear.isBusy() || DriveLeftFront.isBusy() || DriveRightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        DriveLeftRear.getCurrentPosition(),
                        DriveRightRear.getCurrentPosition(),
                        DriveLeftFront.getCurrentPosition(),
                        DriveRightFront.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            runtime.reset();
            DriveLeftRear.setPower(0);
            DriveRightRear.setPower(0);
            DriveLeftFront.setPower(0);
            DriveRightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(2500);   // optional pause after each move

        }

    }

    public void encoderDriveStraight(double speed,
                                     double inches,
                                     double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        boolean rightAhead = false;
        boolean leftAhead = false;

        //Ensures OpMode is running & not out of time
        if (opModeIsActive() && (totaleTime.time() < 29)) {

            DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
//            newLeftBackTarget = DriveLeftRear.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//            newRightBackTarget = DriveRightRear.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//            newLeftFrontTarget = DriveLeftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//            newRightFrontTarget = DriveRightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newLeftBackTarget = (int) (inches * COUNTS_PER_INCH);
            newRightBackTarget = (int) (inches * COUNTS_PER_INCH);
            newLeftFrontTarget = (int) (inches * COUNTS_PER_INCH);
            newRightFrontTarget = (int) (inches * COUNTS_PER_INCH);

            //this logic originally drove backwards.  all neg values here fix that
            DriveLeftRear.setTargetPosition(-newLeftBackTarget);
            DriveRightRear.setTargetPosition(-newRightBackTarget);
            DriveLeftFront.setTargetPosition(-newLeftFrontTarget);
            DriveRightFront.setTargetPosition(-newRightFrontTarget);
            speed = - speed;

            // Turn On RUN_TO_POSITION
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("encoderDriveOut", "encoderDriveStraight starting");
            telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
            telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                    DriveLeftRear.getCurrentPosition(),
                    DriveRightRear.getCurrentPosition(),
                    DriveLeftFront.getCurrentPosition(),
                    DriveRightFront.getCurrentPosition());
            telemetry.update();
//            sleep(2000);   // optional pause after each move

            // reset the timeout time and start motion.
            runtime.reset();
            DriveLeftRear.setPower(speed);
            DriveRightRear.setPower(speed);
            DriveLeftFront.setPower(speed);
            DriveRightFront.setPower(speed);


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (DriveLeftRear.isBusy() || DriveRightRear.isBusy())  //  || DriveLeftFront.isBusy() || DriveRightFront.isBusy())
                    && (totaleTime.time() < 29)) {

              //attempts to correct drive speed by allowing the side that is "behind" to catch up
                if (speed > 0) {
                    if ((DriveLeftRear.getCurrentPosition() > (DriveRightRear.getCurrentPosition() + 10)) && !leftAhead) {  //left is ahead
                        DriveLeftRear.setPower(speed * 0.0);
                        DriveLeftFront.setPower(speed * 0.0);
                        leftAhead = true;
                    } else if ((DriveLeftRear.getCurrentPosition() <= (DriveRightRear.getCurrentPosition())) && leftAhead) {  //right has caught up
                        DriveLeftRear.setPower(speed);
                        DriveLeftFront.setPower(speed);
                        leftAhead = false;
                    } else if ((DriveLeftRear.getCurrentPosition() < (DriveRightRear.getCurrentPosition() - 10)) && !rightAhead) {  //right is ahead
                        DriveRightRear.setPower(speed * 0.0);
                        DriveRightFront.setPower(speed * 0.0);
                        rightAhead = true;
                    } else if ((DriveLeftRear.getCurrentPosition() >= (DriveRightRear.getCurrentPosition())) && rightAhead) {  //left has caught up
                        DriveRightRear.setPower(speed);
                        DriveRightFront.setPower(speed);
                        rightAhead = false;
                    }
                } else { //running in reverse
                    if ((DriveLeftRear.getCurrentPosition() < (DriveRightRear.getCurrentPosition() - 10)) && !leftAhead) {  //left is ahead
                        DriveLeftFront.setPower(speed * 0.0);
                        DriveLeftRear.setPower(speed * 0.0);
                        leftAhead = true;
                    } else if ((DriveLeftRear.getCurrentPosition() >= (DriveRightRear.getCurrentPosition())) && leftAhead) {  //right has caught up
                        DriveLeftFront.setPower(speed);
                        DriveLeftRear.setPower(speed);
                        leftAhead = false;
                    } else if ((DriveLeftRear.getCurrentPosition() > (DriveRightRear.getCurrentPosition() + 10)) && !rightAhead) {  //right is ahead
                        DriveRightFront.setPower(speed * 0.0);
                        DriveRightRear.setPower(speed * 0.0);
                        rightAhead = true;
                    } else if ((DriveLeftRear.getCurrentPosition() <= (DriveRightRear.getCurrentPosition())) && rightAhead) {  //left has caught up
                        DriveRightFront.setPower(speed);
                        DriveRightRear.setPower(speed);
                        rightAhead = false;
                    }
                }


                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        DriveLeftRear.getCurrentPosition(),
                        DriveRightRear.getCurrentPosition(),
                        DriveLeftFront.getCurrentPosition(),
                        DriveRightFront.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            runtime.reset();
            DriveLeftRear.setPower(0);
            DriveRightRear.setPower(0);
            DriveLeftFront.setPower(0);
            DriveRightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }

    }


    public void encoderDriveRotate(double speed,
                                   double degrees,
                                   double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        //Ensures OpMode is running & not out of time
        if (opModeIsActive() && (totaleTime.time() < 29)) {

            DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
//            newLeftBackTarget = DriveLeftRear.getCurrentPosition() - (int) (degrees * COUNTS_PER_DEGREE);
//            newRightBackTarget = DriveRightRear.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
//            newLeftFrontTarget = DriveLeftFront.getCurrentPosition() - (int) (degrees * COUNTS_PER_DEGREE);
//            newRightFrontTarget = DriveRightFront.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newLeftBackTarget =  - (int) (degrees * COUNTS_PER_DEGREE);
            newRightBackTarget =  + (int) (degrees * COUNTS_PER_DEGREE);
            newLeftFrontTarget = - (int) (degrees * COUNTS_PER_DEGREE);
            newRightFrontTarget = + (int) (degrees * COUNTS_PER_DEGREE);


            DriveLeftRear.setTargetPosition(newLeftBackTarget);
            DriveRightRear.setTargetPosition(newRightBackTarget);
            DriveLeftFront.setTargetPosition(newLeftFrontTarget);
            DriveRightFront.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Display initial values for the driver.
            telemetry.addData("encoderDriveOut", "encoderDriveRotate starting");
            telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
            telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                    DriveLeftRear.getCurrentPosition(),
                    DriveRightRear.getCurrentPosition(),
                    DriveLeftFront.getCurrentPosition(),
                    DriveRightFront.getCurrentPosition());
            telemetry.update();
  //          sleep(2500);   // optional pause after each move

            // reset the timeout time and start motion.
            runtime.reset();
            if (speed > 0.0) {
                DriveLeftRear.setPower(-(speed * 1.00));
                DriveRightRear.setPower(speed);
                DriveLeftFront.setPower(-(speed * 1.00));
                DriveRightFront.setPower(speed);
            } else {   //run in reverse
                DriveLeftRear.setPower(speed);
                DriveRightRear.setPower(-(speed * 1.00));
                DriveLeftFront.setPower(speed);
                DriveRightFront.setPower(-(speed * 1.00));
            }

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    && (DriveLeftRear.isBusy() || DriveRightRear.isBusy())  //  || DriveLeftFront.isBusy() || DriveRightFront.isBusy())
                    && (totaleTime.time() < 29)) {

                // Display progress for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        DriveLeftRear.getCurrentPosition(),
                        DriveRightRear.getCurrentPosition(),
                        DriveLeftFront.getCurrentPosition(),
                        DriveRightFront.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            runtime.reset();
            DriveLeftRear.setPower(0);
            DriveRightRear.setPower(0);
            DriveLeftFront.setPower(0);
            DriveRightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }

    }

    public void gyroRotate(
                             int degrees,      // max degrees should be <270
                             double timeoutS) {

        int degreesStart;
        int degreesEnd;

        double speed = 0.15;

        sleep(5000);

        Gyro.calibrate(); //Gyro Reset

        sleep(5000);

        //Ensures OpMode is running & not out of time
        if (opModeIsActive() && (totaleTime.time() < 29)) {
            runtime.reset();
            degreesStart = Gyro.getHeading();
            //degreesEnd = degreesStart + degrees;

            DriveLeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DriveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (degrees<0) {
                speed = -speed;

                degrees = degrees + 360;
            }

            DriveLeftFront.setPower(speed);
            DriveLeftRear.setPower(speed);
            DriveRightFront.setPower(-speed);
            DriveRightRear.setPower(-speed);

            if (speed <  0) {
                /*if (degreesEnd < 0){
                    while (opModeIsActive()
                            && (runtime.seconds() < timeoutS)
                            && (totaleTime.time() < 29)
                            && (Gyro.getHeading() < 340)) {
                        telemetry.addData("Path1", "Gyro: %7d degreesEnd: %7d ", Gyro.getHeading(), degreesEnd);
                        telemetry.update();
                    }
                    degreesEnd = degreesEnd + 360;
                }*/
                while (opModeIsActive()
                        && (runtime.seconds() < timeoutS)
                        && (totaleTime.time() < 29)
                        && ((Gyro.getHeading() > degrees) || (Gyro.getHeading() == 0))
                        ){
                    telemetry.addData("Path1", "CCW Gyro: %7d degrees: %7d ", Gyro.getHeading(), degrees);
                    telemetry.update();
                }



            } else if (speed >  0) {
                /*if (degreesEnd >= 360) {
                    while (opModeIsActive()
                            && (runtime.seconds() < timeoutS)
                            && (totaleTime.time() < 29)
                            && (Gyro.getHeading() > 20)) {
                        telemetry.addData("Path1", "Gyro: %7d degreesEnd: %7d ", Gyro.getHeading(), degreesEnd);
                        telemetry.update();
                    }
                    degreesEnd = degreesEnd - 360;
                }*/
                while (opModeIsActive()
                        && (runtime.seconds() < timeoutS)
                        && (totaleTime.time() < 29)
                        && (Gyro.getHeading() < degrees)) {
                    telemetry.addData("Path1", "CW Gyro: %7d degrees: %7d ", Gyro.getHeading(), degrees);
                    telemetry.update();
                }
            }

            DriveLeftFront.setPower(0);
            DriveLeftRear.setPower(0);
            DriveRightFront.setPower(0);
            DriveRightRear.setPower(0);


        }

    }

}
