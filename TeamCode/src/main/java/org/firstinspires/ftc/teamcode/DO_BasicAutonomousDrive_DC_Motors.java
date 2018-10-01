package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="DO-FramedBot-Autonomous", group="DigiOwls")
// @Disabled
public class DO_BasicAutonomousDrive_DC_Motors extends LinearOpMode {
        private DcMotor leftDrive, leftDriveBack = null;
        private DcMotor rightDrive, rightDriveBack = null;
    public void move(double lpower, double rpower, double lpowerBack, double rpowerBack, int msec)
            throws InterruptedException {
        if (opModeIsActive()) {
            leftDrive.setPower(lpower);
            leftDriveBack.setPower(lpowerBack);
            rightDrive.setPower(rpower);
            rightDriveBack.setPower(rpowerBack);

            sleep(msec);
            leftDrive.setPower(0);
            leftDriveBack.setPower(0);
        }
    }

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

    public void runOpMode() throws InterruptedException {

        configureDCMotors();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//        move(0.5, 0.5, 0.5, 0.5, 1500);    // move forward for two seconds
//        move(-0.5, -0.5, -0.5, -0.5, 1000);    // move backward for 1 sec
        move(0.5, -0.5, -0.5, 0.5, 3000);    // move 3 sec
        move(-0.5, 0.5, 0.5, -0.5, 3000);    // move backward for 1 sec
//        move(0.5, 0, 1500);      // turn to right for 1.5 seconds
    }
    }
