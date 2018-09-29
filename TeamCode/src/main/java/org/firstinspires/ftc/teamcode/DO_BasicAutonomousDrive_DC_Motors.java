package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



@Autonomous(name="DO-Basic-Autonomous-Drive", group="DigiOwls")
// @Disabled
public class DO_BasicAutonomousDrive_DC_Motors extends LinearOpMode {
        private DcMotor leftMotor = null;
        private DcMotor rightMotor = null;
    public void move(double lpower, double rpower, int msec)
            throws InterruptedException {
        if (opModeIsActive()) {
            leftMotor.setPower(lpower);
            rightMotor.setPower(rpower);
            sleep(msec);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }

    private void configureDCMotors() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        leftMotor  = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void runOpMode() throws InterruptedException {

        configureDCMotors();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        move(0.5, 0.5, 2000);    // move forward for two seconds
        move(-0.5, -0.5, 1000);  // move backwards for one second
        move(0.5, 0, 1500);      // turn to right for 1.5 seconds
    }
    }
