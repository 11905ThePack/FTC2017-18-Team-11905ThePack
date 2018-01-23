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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="AutoOp_Pos3")

public class AutoOp_Pos3 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime(); //Elapsed Time

    private DcMotor DriveLeftFront = null; //Left Front Motor
    private DcMotor DriveRightFront = null; //Right Front Motor
    private DcMotor DriveLeftRear = null; //Left Rear Motor
    private DcMotor DriveRightRear = null; //Right Rear Motor

    private DcMotor MotorRelicExtension = null; //Relic Extension Motor

    private ColorSensor ColourSensor1 = null; //ColourSensor FrontLeft
    private ColorSensor ColourSensor2 = null; //ColourSensor Front Right
    private ColorSensor ColourSensorJewel = null; //ColourSensor Jewel Knocker

    private Servo GlyphServoLeft = null; //Left half of glyph grabber
    private Servo GlyphServoRight = null; //Right half of glyph grabber
    private Servo RelicServoFront = null; //Front half of the Relic Grabber
    private Servo RelicServoBack = null; //Back half of the Relic Grabber
    private Servo RelicServoPitch = null; //Relic Grabber Rotation
    private Servo JewelWhackerServo = null; //Jewel Whacker Servo

    private GyroSensor Gyro = null; // Das ist die Gyro



    static final double COUNTS_PER_MOTOR_REV = 1000;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;  // For finding circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override

    public void runOpMode() {

        GlyphServoLeft = hardwareMap.get(Servo.class, "GlyphServoLeft");
        GlyphServoRight = hardwareMap.get(Servo.class, "GlyphServoRight");

        DriveLeftRear = hardwareMap.get(DcMotor.class, "DriveLeftRear");
        DriveRightRear = hardwareMap.get(DcMotor.class, "DriveRightRear");
        DriveLeftFront = hardwareMap.get(DcMotor.class, "DriveLeftFront");
        DriveRightFront = hardwareMap.get(DcMotor.class, "DriveRightFront");

        DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (vuMark != RelicRecoveryVuMark.LEFT) {
            encoderDrive(DRIVE_SPEED,12,12,1); //S1
            encoderDrive(TURN_SPEED, -3, 3, 1); //S2
            encoderDrive(DRIVE_SPEED,36,361); //S3

        }

        if (vuMark != RelicRecoveryVuMark.CENTER) {

            ncoderDrive(DRIVE_SPEED,36,36,1); //S1
            encoderDrive(TURN_SPEED, -3, 3, 1); //S2
            encoderDrive(DRIVE_SPEED,12,12,1); //S3


        }

        if (vuMark != RelicRecoveryVuMark.RIGHT) {

            encoderDrive(DRIVE_SPEED,36,36,1); //S1
            encoderDrive(TURN_SPEED, -3, 3, 1); //S2
            encoderDrive(DRIVE_SPEED,12,12,1); //S3



        }

    }



    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        //Ensures OpMode is running
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = DriveLeftRear.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = DriveRightRear.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);


            DriveLeftRear.setTargetPosition(newLeftTarget);
            DriveRightRear.setTargetPosition(newRightTarget);
            DriveLeftFront.setTargetPosition(newLeftTarget);
            DriveRightFront.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            DriveLeftRear.setPower(Math.abs(speed));
            DriveRightRear.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (DriveLeftRear.isBusy() && DriveRightRear.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        DriveLeftRear.getCurrentPosition(),
                        DriveRightRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            DriveLeftRear.setPower(0);
            DriveRightRear.setPower(0);
            DriveLeftFront.setPower(0);
            DriveRightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }

    }
}