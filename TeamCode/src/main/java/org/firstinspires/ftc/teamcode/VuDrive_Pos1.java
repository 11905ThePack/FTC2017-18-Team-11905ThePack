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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link}.
 */

@Autonomous(name="Concept: VuDrive Id1 ", group ="Concept")
@Disabled
public class VuDrive_Pos1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime(); //Elapsed Time

    private DcMotor DriveLeftRear = null; //Left Front Motor
    private DcMotor DriveRightRear = null; //Right Front Motor
    private DcMotor DriveLeftFront = null; //Left Rear Motor
    private DcMotor DriveRightFront = null;

    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 20.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;  // For finding circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.5;
   // static final double TRANSLATE_SPEED = 0.55;





    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    @Override public void runOpMode() {


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


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = "AXP04Yz/////AAAAGThpXmXBV06voC5uPNdROCVgdOGJ12EoM411qfUt/jRG2l29n2mFd8m38+Pe+njHWXRsIxU9orFJXA4KO2wOcZBruxk3+hY1C3bUCCwbCN0ngMkkiioI5UuKDxWwNpL/qmMPS2SUg6a6xV5J680p5Yxjh6W5TNLpSyX+vdhc9hlkSrzWJPvrm3TYAt06+ox5OVen6tPW/6uSG9TYbCCsx5HKAu/dVzM1lJWzgF/kxXQaRDEck9k1lZp2Y4kyZadvHuWuDg2+EujNPzA9aST8ajxcARP/EjQM+5+hkvAPqHkRPYiZHf1dFZwy6B3e5+d72F1hsFvUYnrUtqOdXwt1KFENNQ+4ZpL622V2NK7ZA1x8";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            encoderDrive(DRIVE_SPEED,36,36,5);
            sleep(200);
            encoderDrive(TURN_SPEED,3,-3,5);
            encoderDrive(DRIVE_SPEED,12,12,5);

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));


                /*if (vuMark != RelicRecoveryVuMark.LEFT) {

                    encoderDrive(DRIVE_SPEED,36,36,5); //S1
                    encoderDrive(TURN_SPEED,3,-3,5); //S2
                    encoderDrive(DRIVE_SPEED,12,12,5); //S3
                    encoderDrive(TURN_SPEED,3,-3,5);
                    encoderDrive(DRIVE_SPEED,6,6,5);
                    encoderDrive(TURN_SPEED,3,-3,5);
                    encoderDrive(DRIVE_SPEED,1,1,5);


                }

                */


                /*
                if (vuMark != RelicRecoveryVuMark.RIGHT) {

                    encoderDrive(DRIVE_SPEED,36,36,10);
                    encoderDrive(TURN_SPEED,3,-3,10);
                    encoderDrive(DRIVE_SPEED,12,12,10);
                    encoderDrive(TURN_SPEED,3,-3,10);
                    encoderDrive(DRIVE_SPEED,6,6,10);
                    encoderDrive(TURN_SPEED,-3,3,10);
                    encoderDrive(DRIVE_SPEED,1,1,10 );

                }
                */

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");


            }

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        //Ensures OpMode is running
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = DriveLeftRear.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightBackTarget = DriveRightRear.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftFrontTarget = DriveLeftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = DriveRightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);


            DriveLeftRear.setTargetPosition(newLeftBackTarget);
            DriveRightRear.setTargetPosition(newRightBackTarget);
            DriveLeftFront.setTargetPosition(newLeftFrontTarget);
            DriveRightFront.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (DriveLeftRear.isBusy() && DriveRightRear.isBusy() && DriveLeftFront.isBusy() && DriveRightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        DriveLeftRear.getCurrentPosition(),
                        DriveRightRear.getCurrentPosition(),
                        DriveLeftFront.getCurrentPosition(),
                        DriveRightFront.getCurrentPosition());
                telemetry.update();

            // reset the timeout time and start motion.
            runtime.reset();
            DriveLeftRear.setPower(Math.abs(speed));
            DriveRightRear.setPower(Math.abs(speed));
            DriveLeftFront.setPower(Math.abs(speed));
            DriveRightFront.setPower(Math.abs(speed));

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


            sleep(250);   // optional pause after each move

        }

    }
}
