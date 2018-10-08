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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;

import java.util.List;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Drive By Encoder", group="DigiOwls")
public class AutoDriveByEncoderWithOpenCVHitNugget extends LinearOpMode {
    private DcMotor leftDrive, leftDriveBack = null;
    private DcMotor rightDrive, rightDriveBack = null;
    private ScanColoredObjects coloredObjects = null;
    /* Declare OpMode members. */
    //HardwareDOBot         robot   = new HardwareDOBot();   // Use a Pushbot's hardware
    FramedDOBot         robot   = new FramedDOBot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1040 ;    // eg: AndyMark NeverRest40
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.6;

    private void configureDCMotors() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        leftDrive  = hardwareMap.get(DcMotor.class, "leftdrive");
        leftDriveBack  = hardwareMap.get(DcMotor.class, "leftdriveBack");
        rightDrive = hardwareMap.get(DcMotor.class, "rightdrive");
        rightDriveBack = hardwareMap.get(DcMotor.class, "rightdriveBack");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
    }

    private void ConfigureCamera() {
        // Set the camera vision
        coloredObjects = new ScanColoredObjects();
        coloredObjects.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
    }

    private void allDrivesSetPower(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        leftDriveBack.setPower(power);
        rightDriveBack.setPower(power);
    }
    private void allDrivesSetMode(DcMotor.RunMode mode){

        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
        rightDriveBack.setMode(mode);
        rightDriveBack.setMode(mode);

    }
    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         */
        configureDCMotors();
        ConfigureCamera();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        allDrivesSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        allDrivesSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Come back for few steps
        encoderDrive(DRIVE_SPEED, 5, 5, 5, 5);  // S1: Forward 47 Inches with 5 Sec timeout

        // start the vision system
        coloredObjects.enable();

        while(opModeIsActive()){
            ScanOrangeJewelAndHit();

            coloredObjects.setRequiredObject("zoneBorderBlue");
        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void ScanOrangeJewelAndHit() {
        List<MatOfPoint> contours = coloredObjects.getContours();
        if(contours.isEmpty())
        {
            //If desired frame is not received then keep rotating
            encoderDrive(TURN_SPEED,   -1, 1, -1, 1);  // S2: Turn Right 12 Inches with 4 Sec timeout
        }
        else
        {
            //If desired jewel is found then move linear and hit it
            encoderDrive(DRIVE_SPEED, 15, 15, 15, 15);  // S1: Forward 47 Inches with 5 Sec timeout
        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double leftBackInches, double rightBackInches) {
        int newLeftTarget, newLeftBackTarget;
        int newRightTarget, newRightBackTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget   = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget  = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        newLeftBackTarget  = leftDriveBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
        newRightBackTarget = rightDriveBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        leftDriveBack.setTargetPosition(newLeftBackTarget);
        rightDriveBack.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        allDrivesSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        allDrivesSetPower(Math.abs(speed));

        // Stop all motion;
        allDrivesSetPower(0);

        // Turn off RUN_TO_POSITION
        allDrivesSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
