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
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Digital Owls TeleOp Mode", group="DigiOwls")
//@Disabled
public class DO_TeleOpMode_GamePad_1 extends LinearOpMode {
    private FramedDOBot robot = new FramedDOBot();   // Use a Pushbot's hardware
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Run the robot in the telemode (FWD, BWD and turn)
            if(gamepad1.left_stick_y != 0) {
                MoveRobot();
            }
            else if( (gamepad1.left_trigger != 0) || (gamepad1.right_trigger != 0)) {
                MovePully(); //latching and unlatching
            }
            else if((gamepad1.right_stick_x != 0) || (gamepad1.right_stick_y != 0)){
                TurnRobot();
            }
            else {
                robot.AllDrivesSetPower(0, true);
                //robot.latchMotor.setPower(0);
            }
        }
    }

    private void MovePully(){
        //Latch motor is for reverse direction
        double latchDist = Range.clip(gamepad1.left_trigger + gamepad1.right_trigger, -1.0, 1.0);
        robot.leftDrive.setPower(-latchDist);
    }

    private void MoveRobot() {
        // Send calculated power to wheels
        double dist  =  Range.clip(gamepad1.left_stick_y, -1.0, 1.0) ;
        robot.leftDrive.setPower(-dist);
        robot.rightDrive.setPower(-dist);
        robot.leftDriveBack.setPower(-dist);
        robot.rightDriveBack.setPower(-dist);
    }

    private void TurnRobot() {
        double turn  =  Range.clip(gamepad1.right_stick_y, -1.0, 1.0) ;
        double lateral = Range.clip(gamepad1.right_stick_x, -1.0, 1.0) ;
        double dist = turn + lateral;
        if(gamepad1.right_stick_x != 0){
            robot.leftDrive.setPower(-dist);
            robot.leftDriveBack.setPower(dist);
            robot.rightDrive.setPower(dist);
            robot.rightDriveBack.setPower(-dist);
        }
        else {
            robot.leftDrive.setPower(dist);
            robot.leftDriveBack.setPower(dist);
            robot.rightDrive.setPower(-dist);
            robot.rightDriveBack.setPower(-dist);
        }
    }
}
