package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="Auto2", group="Test")
public class Auto2 extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        DriveForwardDistance(50, 1110);


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

    //DRIVE FORWARDS USING ENCODERS
    public void DriveForwardDistance(double power, int distance)
    {
        //Reset encoders
        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set target position
        robot.motor1.setTargetPosition(distance);
        robot.motor2.setTargetPosition(distance);
        robot.motor3.setTargetPosition(distance);
        robot.motor4.setTargetPosition(distance);

        //set RUN_TO_POSITION mode
        robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set drive power
        DriveForward(power);

        while(robot.motor1.isBusy() || robot.motor2.isBusy())
        {
            //Wait until target position is reached
            telemetry.addData("Encoder", robot.motor1.getCurrentPosition());
            telemetry.addData("Encoder", robot.motor2.getCurrentPosition());
            telemetry.addData("Encoder", robot.motor3.getCurrentPosition());
            telemetry.addData("Encoder", robot.motor4.getCurrentPosition());
            telemetry.update();
        }

        //Stop and change modes back to normal
        StopDriving();
        robot.motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //DRIVE BACKWARDS USING ENCODERS
    public void DriveBackwardDistance(double power, int distance)
    {
        //Reset encoders
        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set target position
        robot.motor1.setTargetPosition(-distance);
        robot.motor2.setTargetPosition(-distance);
        robot.motor3.setTargetPosition(-distance);
        robot.motor4.setTargetPosition(-distance);

        //set RUN_TO_POSITION mode
        robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set drive power
        DriveBackward(power);

        while(robot.motor1.isBusy() && robot.motor2.isBusy())
        {
            //Wait until target position is reached
        }

        //Stop and change modes back to normal
        StopDriving();
        robot.motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //TURN LEFT USING ENCODERS
    public void TurnLeftDistance(double power, int distance) {
        //Reset encoders
        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set target position
        robot.motor1.setTargetPosition(-distance);
        robot.motor2.setTargetPosition(distance);
        robot.motor3.setTargetPosition(-distance);
        robot.motor4.setTargetPosition(distance);

        //set RUN_TO_POSITION mode
        robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set drive power
        TurnLeft(power);

        while (robot.motor1.isBusy() && robot.motor2.isBusy()) {
            //Wait until target position is reached
        }

        //Stop and change modes back to normal
        StopDriving();
        robot.motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //TURN RIGHT USING ENCODERS
    public void TurnRightDistance(double power, int distance) {
        //Reset encoders
        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set target position
        robot.motor1.setTargetPosition(distance);
        robot.motor2.setTargetPosition(-distance);
        robot.motor3.setTargetPosition(distance);
        robot.motor4.setTargetPosition(-distance);

        //set RUN_TO_POSITION mode
        robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set drive power
        TurnRight(power);

        while (robot.motor1.isBusy() && robot.motor2.isBusy()) {
            //Wait until target position is reached
        }

        //Stop and change modes back to normal
        StopDriving();
        robot.motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
