package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoOpEncoderTest")

public class AutoOpEncoderTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor DriveMotorLeft = null;
    private DcMotor DriveMotorRight = null;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        DriveMotorLeft = hardwareMap.get(DcMotor.class, "DriveMotorLeft");
        DriveMotorRight = hardwareMap.get(DcMotor.class, "DriveMotorRight");

        DriveMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        //The numbers goes like this, MotorLeft, MotorRight, Timeout
        encoderDrive(DRIVE_SPEED, 2, 2, 5.0);  // S1: Forward 2 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED, 2, -2, 4.0);  // S2: Turn Right 2 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 4, 4, 4.0);  // S3: Reverse 4 Inches with 4 Sec timeout

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = DriveMotorLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = DriveMotorRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            DriveMotorLeft.setTargetPosition(newLeftTarget);
            DriveMotorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            DriveMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            DriveMotorLeft.setPower(Math.abs(speed));
            DriveMotorRight.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (DriveMotorLeft.isBusy() && DriveMotorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        DriveMotorLeft.getCurrentPosition(),
                        DriveMotorRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            DriveMotorLeft.setPower(0);
            DriveMotorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            DriveMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }

    }
}