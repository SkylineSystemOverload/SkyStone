// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="IMUTest", group="Test")
public class IMUTest extends LinearOpMode
{
    RobotHardware robot = new RobotHardware();
    //private ElapsedTime runtime = new ElapsedTime();

    //TouchSensor             touch;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .50, correction;
    boolean                 aButton, bButton, touched;
    boolean hasRun = false;

    /*public void smartSleep (double secondsToSleep) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < secondsToSleep)) {
            telemetry.addData("Path", "Leg: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }*/

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);


        // get a reference to touch sensor.
        //touch = hardwareMap.touchSensor.get("touch_sensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;


        robot.imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        long setTime = System.currentTimeMillis();

        while (opModeIsActive())
        {
            // Use gyro to drive in a straight line.
            correction = checkDirection();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

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

            if(System.currentTimeMillis() - setTime > 3200) {
                StrafeLeft();
            }
            else if(System.currentTimeMillis() - setTime > 4000) {
                StopDriving();
            }
            else if(System.currentTimeMillis() - setTime > 4300) {
                DriveForward();
            }
            else if(System.currentTimeMillis() - setTime > 5700) {
                StopDriving();
            }
            else if(System.currentTimeMillis() - setTime > 6000) {
                //lower arm
                robot.motor5.setPower(-.5);
            }
            else if(System.currentTimeMillis() - setTime > 7500) {
                robot.motor5.setPower(0);
            }
            else if(System.currentTimeMillis() - setTime > 7800) {
                DriveBackward();
            }
            else if(System.currentTimeMillis() - setTime > 8100) {
                StopDriving();
        //PLATFORM IS MOVED
                //raise arm
                robot.motor5.setPower(.5);
            }
            else if(System.currentTimeMillis() > 8800) {
                robot.motor5.setPower(0);
                //retract finger
                robot.motor6.setPower(-.5);
            }
            else if(System.currentTimeMillis() > 9300) {
                robot.motor6.setPower(0);
                DriveBackward();
            }
            else if(System.currentTimeMillis() - setTime > 9600) {
                StopDriving();
                //lower arm
                robot.motor5.setPower(-.5);
            }
            else if(System.currentTimeMillis() - setTime > 10300) {
                robot.motor5.setPower(0);
                StrafeRight();
            }
            else if(System.currentTimeMillis() - setTime > 12600) {
                StopDriving();
            }
            else if(System.currentTimeMillis() - setTime > 13100) {
                DriveForward();
            }
            else if(System.currentTimeMillis() - setTime > 15100) {
                StopDriving();
            }
            else if(System.currentTimeMillis() - setTime > 15600) {
                StrafeLeft();
            }
            else if(System.currentTimeMillis() - setTime > 17900) {
                StopDriving();
            }
            else if(System.currentTimeMillis() - setTime > 18400) {
                DriveBackward();
            }
            else if(System.currentTimeMillis() - setTime > 19400) {
                StopDriving();
            }
        //PLATFORM IS MOVED TO NEW LOCATION
            else if(System.currentTimeMillis() - setTime > 19900) {
                DriveForward();
            }
            else if(System.currentTimeMillis() - setTime > 20400) {
                StopDriving();
            }
            else if(System.currentTimeMillis() - setTime > 20900) {
                StrafeRight();
            }
            else if(System.currentTimeMillis() - setTime > 22900) {
                StopDriving();
            }
            else if(System.currentTimeMillis() - setTime > 23400) {
                DriveBackward();
            }
            else if(System.currentTimeMillis() - setTime > 25400) {
                StopDriving();
            }
            else if(System.currentTimeMillis() - setTime > 25900) {
                StrafeRight();
            }
            else if(System.currentTimeMillis() - setTime >  27400) {
                StopDriving();
            }
            else {
                StopDriving();
            }
            //smartSleep(2);

                // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            aButton = gamepad1.a;
            bButton = gamepad1.b;
            //touched = touch.isPressed();

            if (/*touched || */aButton || bButton)
            {
                // backup.
                robot.motor1.setPower(power);
                robot.motor2.setPower(power);

                sleep(500);

                // stop.
                robot.motor1.setPower(0);
                robot.motor2.setPower(0);

                // turn 90 degrees right.
                if (/*touched || */aButton) rotate(-90, power);

                // turn 90 degrees left.
                if (bButton) rotate(90, power);
            }
        }

        // turn the motors off.
        StopDriving();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void DriveForward() {
        robot.motor1.setPower(-power + correction);
        robot.motor3.setPower(-power + correction);//test
        robot.motor2.setPower(-power - correction);
        robot.motor4.setPower(-power - correction);
    }
    public void DriveBackward() {
        robot.motor1.setPower(power + correction);
        robot.motor2.setPower(power + correction);
        robot.motor3.setPower(power - correction);
        robot.motor4.setPower(power - correction);
    }
    public void StrafeLeft() {
        robot.motor1.setPower(-power + correction);
        robot.motor2.setPower(power + correction);
        robot.motor3.setPower(power - correction);
        robot.motor4.setPower(-power - correction);
    }
    public void StrafeRight() {
        robot.motor1.setPower(power + correction);
        robot.motor2.setPower(-power + correction);
        robot.motor3.setPower(-power - correction);
        robot.motor4.setPower(power - correction);
    }
    public void StopDriving() {
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
    }


    private void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .05;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;//test
            rightPower = +power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = +power;//test
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        robot.motor1.setPower(leftPower);
        robot.motor3.setPower(leftPower);
        robot.motor2.setPower(rightPower);
        robot.motor4.setPower(rightPower);


        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);

        // wait for rotation to stop.
        sleep(2000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}