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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Gyanesh- Wheels Movement Trigger", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear_Turning_Wheels_WithTriggers extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ConfigureDCMotors();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        RobotMovements robotMovements = new RobotMovements();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run the robot in the telemode (FWD, BWD and turn)
            robotMovements.MoveAndTurnRobot();

            // Show the elapsed game time and wheel power.
            robotMovements.UpdateStatusOverDriverPhone(robotMovements.getFwdPower(), robotMovements.getBwdPower());
        }
    }

    private void ConfigureDCMotors() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        leftDrive  = hardwareMap.get(DcMotor.class, "leftdrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightdrive");
        leftDriveBack  = hardwareMap.get(DcMotor.class, "leftdriveback");
        rightDriveBack = hardwareMap.get(DcMotor.class, "rightdriveback");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public class RobotMovements {
        private double fwdPower;
        private double bwdPower;

        public double getFwdPower() {
            return fwdPower;
        }

        public double getBwdPower() {
            return bwdPower;
        }

        private void UpdateStatusOverDriverPhone(double leftPower, double rightPower) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }

        public void MoveAndTurnRobot() {

            double turn  =  gamepad1.right_stick_x;

            // Send calculated power to wheels
            if(gamepad1.left_trigger == 0)
            {
                leftDrive.setPower(gamepad1.right_trigger);
                rightDrive.setPower(gamepad1.right_trigger);
                leftDriveBack.setPower(gamepad1.right_trigger);
                rightDriveBack.setPower(gamepad1.right_trigger);
            }
            else if(gamepad1.right_trigger == 0)
            {
                leftDrive.setPower(-gamepad1.left_trigger);
                rightDrive.setPower(-gamepad1.left_trigger);
                leftDriveBack.setPower(-gamepad1.left_trigger);
                rightDriveBack.setPower(-gamepad1.left_trigger);
            }
            else
            {
                leftDrive.setPower(Range.clip(turn, -1.0, 1.0));
                leftDriveBack.setPower(Range.clip(turn, -1.0, 1.0));
                rightDrive.setPower(Range.clip(-turn, -1.0, 1.0));
                rightDriveBack.setPower(Range.clip(-turn, -1.0, 1.0));
            }
        }
    }
}
