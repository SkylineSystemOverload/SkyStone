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


@Autonomous(name="AutoRed", group="Test")

public class AutoRed extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        //extend finger
        robot.motor6.setPower(-.5);
        sleep(500);
        robot.motor6.setPower(0);

        //raise arm
        robot.motor5.setPower(.5);
        sleep(700);
        robot.motor5.setPower(0);

        //strafe right
        robot.motor1.setPower(.5);
        robot.motor2.setPower(-.5);
        robot.motor3.setPower(-.5);
        robot.motor4.setPower(.5);
        sleep(800);

        //drive forwards
        DriveForward(.5);

        while (robot.motor1.isBusy() || robot.motor4.isBusy()) {
            telemetry.addData("motor1 Power", robot.motor1.getPower());
            telemetry.addData("motor2 Power", robot.motor2.getPower());
            telemetry.addData("motor3 Power", robot.motor3.getPower());
            telemetry.addData("motor4 Power", robot.motor4.getPower());
            telemetry.update();
        }
        sleep(1400);

        //stop
        StopDriving();
        sleep(1000);

        //lower arm
        robot.motor5.setPower(-.5);
        sleep(700);
        //stop
        robot.motor5.setPower(0);

        //drive backwards
        DriveBackward(-0.5);
        sleep(1700);

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
        robot.motor6.setPower(.5);
        sleep(500);
        robot.motor6.setPower(0);

        //drive backwards
        DriveBackward(-0.5);
        sleep(700);

        //stop
        StopDriving();
        sleep(500);

        //lower arm
        robot.motor5.setPower(-.5);
        sleep(700);
        //stop
        robot.motor5.setPower(0);

        //strafe left
        robot.motor1.setPower(-.5);
        robot.motor2.setPower(.5);
        robot.motor3.setPower(.5);
        robot.motor4.setPower(-.5);
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

        //strafe right
        robot.motor1.setPower(.5);
        robot.motor2.setPower(-.5);
        robot.motor3.setPower(-.5);
        robot.motor4.setPower(.5);
        sleep(2300);

        //stop
        StopDriving();
        sleep(500);

        //drive backwards
        DriveBackward(-0.5);
        sleep(1500);

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

        //strafe left
        robot.motor1.setPower(-.7);
        robot.motor2.setPower(.7);
        robot.motor3.setPower(.7);
        robot.motor4.setPower(-.7);
        sleep(2000);

        //stop
        StopDriving();
        sleep(500);

        //drive backwards
        DriveBackward(-0.5);
        sleep(2000);

        //stop
        StopDriving();
        sleep(500);

        //strafe right
        robot.motor1.setPower(-.5);
        robot.motor2.setPower(.5);
        robot.motor3.setPower(.5);
        robot.motor4.setPower(-.5);
        sleep(1500);

        //stop
        StopDriving();
        sleep(500);
    }

    public void DriveForward(double power) {
        robot.motor1.setPower(power);
        robot.motor2.setPower(power);
        robot.motor3.setPower(power);
        robot.motor4.setPower(power);
    }
    public void DriveBackward(double power) {
        robot.motor1.setPower(-power);
        robot.motor2.setPower(-power);
        robot.motor3.setPower(-power);
        robot.motor4.setPower(-power);
    }
    public void TurnLeft(double power) {
        robot.motor1.setPower(-power);
        robot.motor2.setPower(power);
        robot.motor3.setPower(-power);
        robot.motor4.setPower(power);
    }
    public void TurnRight(double power) {
        TurnLeft(-power);
    }
    public void StopDriving() {
        DriveForward(0);
    }

}
