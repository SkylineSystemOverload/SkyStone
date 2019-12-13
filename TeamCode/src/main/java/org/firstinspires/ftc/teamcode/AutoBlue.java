package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="AutoBlue", group="Test")

public class AutoBlue extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    static final double MOTOR_TICK_COUNT = 1120;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        //raise arm
        robot.motor5.setPower(.5);
        sleep(1500);
        robot.motor5.setPower(0);
        sleep(300);

        //extend finger
        robot.motor6.setPower(.5);
        sleep(500);
        robot.motor6.setPower(0);
        sleep(700);

        //strafe left
        StrafeLeft(.7);
        sleep(800);

        StopDriving();
        sleep(300);

        //drive forwards
        DriveForward(.5);

        while (robot.motor1.isBusy() || robot.motor4.isBusy()) {
            telemetry.addData("Path", "Driving");
            telemetry.update();
        }
        sleep(1400);

        //stop
        StopDriving();
        sleep(1000);

        //lower arm
        robot.motor5.setPower(-.5);
        sleep(1500);
        //stop
        robot.motor5.setPower(0);
        sleep(300);

        //drive backwards
        DriveBackward(.5);
        sleep(1300);

        //stop
        StopDriving();
        sleep(500);

        //PLATFORM IS MOVED

        //raise arm
        robot.motor5.setPower(.5);
        sleep(700);
        //stop
        robot.motor5.setPower(0);

        //retract finger
        robot.motor6.setPower(-.5);
        sleep(500);
        robot.motor6.setPower(0);

        //drive backwards
        DriveBackward(.5);
        sleep(300);

        //stop
        StopDriving();
        sleep(500);

        //lower arm
        robot.motor5.setPower(-.5);
        sleep(700);
        //stop
        robot.motor5.setPower(0);

        //strafe right
        StrafeRight(.7);
        sleep(2300);

        //stop
        StopDriving();
        sleep(500);

        //drive forwards
        DriveForward(.5);
        sleep(2000);

        //stop
        StopDriving();
        sleep(500);

        //strafe left
        StrafeLeft(.7);
        sleep(2300);

        //stop
        StopDriving();
        sleep(500);

        //drive backwards
        DriveBackward(.5);
        sleep(1000);

        //stop
        StopDriving();
        sleep(500);

        //PLATFORM IS MOVED TO LOCATION

        //drive forwards
        DriveForward(.5);
        sleep(500);

        //stop
        StopDriving();
        sleep(500);

        //strafe right
        StrafeRight(.7);
        sleep(2000);

        //stop
        StopDriving();
        sleep(500);

        //drive backwards
        DriveBackward(.5);
        sleep(2000);

        //stop
        StopDriving();
        sleep(500);

        //strafe right
        StrafeRight(.7);
        sleep(1500);

        //stop
        StopDriving();
        sleep(500);

    }
    private void DriveForward(double power) {
        robot.motor1.setPower(power);
        robot.motor2.setPower(power);
        robot.motor3.setPower(power);
        robot.motor4.setPower(power);
    }
    private void DriveBackward(double power) {
        robot.motor1.setPower(-power);
        robot.motor2.setPower(-power);
        robot.motor3.setPower(-power);
        robot.motor4.setPower(-power);
    }
    private void TurnLeft(double power) {
        robot.motor1.setPower(-power);
        robot.motor2.setPower(power);
        robot.motor3.setPower(-power);
        robot.motor4.setPower(power);
    }
    private void TurnRight(double power) {
        TurnLeft(-power);
    }

    private void StopDriving() {
        DriveForward(0);
    }

    private void StrafeLeft(double power) {
        robot.motor1.setPower(power);
        robot.motor2.setPower(-power);
        robot.motor3.setPower(-power);
        robot.motor4.setPower(power);
    }
    private void StrafeRight(double power){
        robot.motor1.setPower(-power);
        robot.motor2.setPower(power);
        robot.motor3.setPower(power);
        robot.motor4.setPower(-power);
    }
}
