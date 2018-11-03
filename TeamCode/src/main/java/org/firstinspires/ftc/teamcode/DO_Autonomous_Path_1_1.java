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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Collections;
import java.util.List;

@Autonomous (name = "DO: Autonomous Path 1", group = "DigiOwls")
//@Disabled
public class DO_Autonomous_Path_1_1 extends LinearOpMode {
    /* Declare OpMode members. */
    FramedDOBot             robot                   = new FramedDOBot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime                 = new ElapsedTime();
    private static final double     COUNTS_PER_MOTOR_REV    = 1120;//1040 ;    // eg: AndyMark NeverRest40
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     DRIVE_SPEED             = 0.6;
    private static final double     TURN_SPEED              = 0.5;

    private static final double     COUNTS_PER_MOTOR_REV_Latch    = 2240;
    private static final double     DRIVE_GEAR_REDUCTION_Latch    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES_Latch   = 1.45 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH_Latch         = (COUNTS_PER_MOTOR_REV_Latch * DRIVE_GEAR_REDUCTION_Latch) /
            (WHEEL_DIAMETER_INCHES_Latch * 3.1415);
    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.AllDrivesSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.AllDrivesSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // reset the timeout time and start motion.
        runtime.reset();
        RunInToPath1();
    }

    private void UnLatchRobot(double speed, double inches, double timeOut, String operation){
        // Ensure that the opmode is still active
//        int newUnlatchTarget = 0;
//        if (opModeIsActive()) {
//            // Determine new target position, and pass to motor controller
//            newUnlatchTarget = robot.latchMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH_Latch);
//            robot.latchMotor.setTargetPosition(newUnlatchTarget);
//
//            // Turn On RUN_TO_POSITION
//            robot.latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.latchMotor.setPower(Math.abs(speed));
//            runtime.reset();
//            while (opModeIsActive() &&
//                   (robot.latchMotor.isBusy() ) &&
//                   (runtime.seconds() < timeOut)){
//
//            }
//            // Stop all motion;
//            robot.latchMotor.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.latchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            telemetry.addData(operation, "takes %.1f s", runtime.seconds());
//            telemetry.update();
//        }
    }

    private void RunInToPath1() {
        telemetry.setAutoClear(false);

        //Unlatch robot
        UnLatchRobot(DRIVE_SPEED, 5, 5, "Unlatching the robot");

        //run lateral
        encoderDrive(DRIVE_SPEED,2,-2,-2,2, 5.0, "Move away (right) from notch", true);

        //run forward
        encoderDrive(DRIVE_SPEED,-10,-10,-10,-10, 5.0, "Move forward", false);

        //run lateral to left for 10inch to hit all the jewels
//        //Gyanesh MoveLeftScanAndHitJewel();
//
        //Turn to right
        encoderDrive(DRIVE_SPEED,5, -5, 5, -5, 5, "Lateral right", false);

        //run towards wall
        encoderDrive(DRIVE_SPEED,-20, -20, -20, -20, 5, "Lateral right", false);

//        //Turn right parallel to wall
        encoderDrive(DRIVE_SPEED,5, -5, 5, -5, 5, "Left for clearance", false);

        //run towards team's zone
        encoderDrive(DRIVE_SPEED,-20, -20, -20,-20, 5, "Reaching zone", false);
//
//        //Finally, go and park in crater
        encoderDrive(DRIVE_SPEED,30, 30, 30,30, 5, "Parking to carater", false);
    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    private void encoderDrive(double speed, double leftInches, double rightInches,
                             double leftBackInches, double rightBackInches, double timeOut, String operation, Boolean goLateral) {
        int newLeftTarget, newLeftBackTarget;
        int newRightTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget       = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget      = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget   = robot.leftDriveBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget  = robot.rightDriveBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            telemetry.addData(operation, "LTgt %d, RTgt %d, LBTgt %d, RBTgt %d", newLeftTarget, newRightTarget, newLeftBackTarget, newRightBackTarget);
            telemetry.update();

            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftDriveBack.setTargetPosition(newLeftBackTarget);
            robot.rightDriveBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.AllDrivesSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.AllDrivesSetPower(Math.abs(speed), goLateral);

            runtime.reset();
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ( opModeIsActive() &&
                   (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() /*&&
                    robot.leftDriveBack.isBusy() && robot.rightDriveBack.isBusy()*/) &&
                    (runtime.seconds() < timeOut)){
                //sleep(100);
            }
            telemetry.addData(operation, "takes %.1f s", runtime.seconds());
            telemetry.update();
        }
        if(!opModeIsActive()){
            // Stop all motion;
            robot.AllDrivesSetPower(0, true);
            // Turn off RUN_TO_POSITION
            robot.AllDrivesSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
