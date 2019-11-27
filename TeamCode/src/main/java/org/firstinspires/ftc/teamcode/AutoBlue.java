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

        //extend finger
        robot.motor6.setPower(-.5);
        sleep(500);
        robot.motor6.setPower(0);

        //raise arm
        robot.motor5.setPower(.5);
        sleep(700);
        robot.motor5.setPower(0);

        //strafe left
        robot.motor1.setPower(-.5);
        robot.motor2.setPower(.5);
        robot.motor3.setPower(.5);
        robot.motor4.setPower(-.5);
        sleep(800);

        //drive forwards
        robot.motor1.setPower(.5);
        robot.motor2.setPower(.5);
        robot.motor3.setPower(.5);
        robot.motor4.setPower(.5);

        while (robot.motor1.isBusy() || robot.motor4.isBusy()) {
            telemetry.addData("Path", "Driving");
            telemetry.update();
        }
        sleep(1400);

        //stop
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        sleep(1000);

        //lower arm
        robot.motor5.setPower(-.5);
        sleep(700);
        //stop
        robot.motor5.setPower(0);

        //drive backwards
        robot.motor1.setPower(-.5);
        robot.motor2.setPower(-.5);
        robot.motor3.setPower(-.5);
        robot.motor4.setPower(-.5);
        sleep(1300);

        //stop
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
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
        robot.motor1.setPower(-.5);
        robot.motor2.setPower(-.5);
        robot.motor3.setPower(-.5);
        robot.motor4.setPower(-.5);
        sleep(700);

        //stop
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        sleep(500);

        //lower arm
        robot.motor5.setPower(-.5);
        sleep(700);
        //stop
        robot.motor5.setPower(0);

        //strafe right
        robot.motor1.setPower(.5);
        robot.motor2.setPower(-.5);
        robot.motor3.setPower(-.5);
        robot.motor4.setPower(.5);
        sleep(2300);

        //stop
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        sleep(500);

        //drive forwards
        robot.motor1.setPower(.5);
        robot.motor2.setPower(.5);
        robot.motor3.setPower(.5);
        robot.motor4.setPower(.5);
        sleep(2000);

        //stop
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        sleep(500);

        //strafe left
        robot.motor1.setPower(-.5);
        robot.motor2.setPower(.5);
        robot.motor3.setPower(.5);
        robot.motor4.setPower(-.5);
        sleep(2300);

        //stop
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        sleep(500);

        //drive backwards
        robot.motor1.setPower(-.5);
        robot.motor2.setPower(-.5);
        robot.motor3.setPower(-.5);
        robot.motor4.setPower(-.5);
        sleep(1500);

        //stop
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        sleep(500);

        //PLATFORM IS MOVED TO LOCATION

        //drive forwards
        robot.motor1.setPower(.5);
        robot.motor2.setPower(.5);
        robot.motor3.setPower(.5);
        robot.motor4.setPower(.5);
        sleep(500);

        //stop
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        sleep(500);

        //strafe right
        robot.motor1.setPower(.7);
        robot.motor2.setPower(-.7);
        robot.motor3.setPower(-.7);
        robot.motor4.setPower(.7);
        sleep(2000);

        //stop
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        sleep(500);

        //drive backwards
        robot.motor1.setPower(-.5);
        robot.motor2.setPower(-.5);
        robot.motor3.setPower(-.5);
        robot.motor4.setPower(-.5);
        sleep(2000);

        //stop
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        sleep(500);

        //strafe right
        robot.motor1.setPower(.5);
        robot.motor2.setPower(-.5);
        robot.motor3.setPower(-.5);
        robot.motor4.setPower(.5);
        sleep(1500);

        //stop
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        sleep(500);

    }
}
