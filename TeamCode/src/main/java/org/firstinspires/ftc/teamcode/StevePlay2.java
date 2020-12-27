/*
Copyright 2019 FIRST Tech Challenge Team 17235

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Math;

//import com.qualcomm.robotcore.hardware.Gyroscope;
//import com.qualcomm.robotcore.hardware.OrientationSensor;  //This may be what's needed for getAngularOrientation
//import com.qualcomm.hardware.bosch.BNO55IMU;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class StevePlay2 extends LinearOpMode {
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    private Servo back_foundation_puller;
    private DcMotor back_left_wheel;
    private DcMotor back_right_wheel;
    private DcMotor clamp_motor;
    private DistanceSensor color_sensor;
    private DistanceSensor distance_sensor;
    private ColorSensor front_color_sensor;
    private DcMotor front_left_wheel;
    private DcMotor front_right_wheel;
    //private Gyroscope imu_1;
    //private IntegratingGyroscope imu_1;
    private BNO055IMU imu;
    //private OrientationSensor imu;
    private DcMotor lift_motor;
    private DigitalChannel switch_;

    private class Chassis {
        double frontLeft;
        double backLeft;
        double frontRight;
        double backRight;
        
        private void SetMotors (double drive, double strafe, double rotate) {
            this.frontLeft = -drive + strafe + rotate;
            this.backLeft = -drive - strafe + rotate;
            this.frontRight = drive + strafe + rotate;
            this.backRight = drive - strafe + rotate;
        }
        
        private void Drive () {
            front_left_wheel.setPower(this.frontLeft);
            back_left_wheel.setPower(this.backLeft);
            front_right_wheel.setPower(this.frontRight);
            back_right_wheel.setPower(this.backRight);
            
        }
    }
    
    private class UberTool {
        private void Lift (double liftPower) {
            lift_motor.setPower(liftPower);
        }
        
        private void Clamp (double clampPower) {
            clamp_motor.setPower(clampPower);
        }
    }

    enum OperState {
        NORMALDRIVE,
        ROTATETOPOSITION
    }

    @Override
    public void runOpMode() {
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        back_foundation_puller = hardwareMap.get(Servo.class, "back foundation puller");
        back_left_wheel = hardwareMap.get(DcMotor.class, "back left wheel");
        back_right_wheel = hardwareMap.get(DcMotor.class, "back right wheel");
        clamp_motor = hardwareMap.get(DcMotor.class, "clamp motor");
        color_sensor = hardwareMap.get(DistanceSensor.class, "color sensor");
        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance sensor");
        front_color_sensor = hardwareMap.get(ColorSensor.class, "front color sensor");
        front_left_wheel = hardwareMap.get(DcMotor.class, "front left wheel");
        front_right_wheel = hardwareMap.get(DcMotor.class, "front right wheel");
        //imu_1 = hardwareMap.get(Gyroscope.class, "imu 1");
        //imu_1 = hardwareMap.get(IntegratingGyroscope.class, "imu 1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        //imu = hardwareMap.get(OrientationSensor.class, "imu");
        lift_motor = hardwareMap.get(DcMotor.class, "lift motor");
        switch_ = hardwareMap.get(DigitalChannel.class, "switch ");

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu.initialize(parameters);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double liftPower = 0;
        double clampPower = 0;
        double frontLeftWheelPower = 0;
        double frontRightWheelPower = 0;
        double backLeftWheelPower = 0;
        double backRightWheelPower = 0;
        double drive = 0;  //Power for forward and back motion
        double strafe = 0; //Power for left and right motion
        double rotate = 0; //Power for rotating the robot
        Chassis superRobot = new Chassis();
        UberTool bigTool = new UberTool();
        double rotationAngle = 0;
        OperState driveOpState = OperState.NORMALDRIVE;
        
        while (opModeIsActive()) {
            telemetry.addData("Status", "Let's get GOING!!!");
            //liftPower = -this.gamepad1.left_stick_y;
            //lift_motor.setPower(liftPower);
            //telemetry.addData("Target Power", liftPower);
            //telemetry.addData("Motor Power", lift_motor.getPower());
            //Keypad Control Definition

            //Orientation Angle = imu_1.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
            //double Angle = getHeading();
            //zAngle = string.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(Angle.firstAngle));
            double zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
            double yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).secondAngle;
            double xAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).thirdAngle;
            switch (driveOpState) {
                case NORMALDRIVE :
                    drive = -this.gamepad1.left_stick_y;
                    strafe = -this.gamepad1.left_stick_x;
                    rotate = -this.gamepad1.right_stick_x;
                    liftPower = -this.gamepad1.right_stick_y;
                    clampPower = 0.25*(-this.gamepad1.right_trigger + this.gamepad1.left_trigger);
                    superRobot.SetMotors (drive, strafe, rotate);
                    superRobot.Drive();
                    bigTool.Lift(liftPower);
                    bigTool.Clamp(clampPower);
                    if (this.gamepad1.left_bumper) {
                        driveOpState = OperState.ROTATETOPOSITION;
                        rotationAngle = -45;
                    } else if (this.gamepad1.right_bumper) {
                        driveOpState = OperState.ROTATETOPOSITION;
                        rotationAngle = 60;
                    } else {
                        driveOpState = OperState.NORMALDRIVE;
                    }
                    break;
                case ROTATETOPOSITION :
                    if (Math.abs(zAngle - rotationAngle) < 2) {
                        superRobot.SetMotors(0,0,0);
                        superRobot.Drive();
                        driveOpState = OperState.NORMALDRIVE;
                    } else {
                        rotate = Math.max(0.15, Math.abs(rotationAngle - zAngle) / 180);
                        rotate = Math.signum(rotationAngle - zAngle) * rotate;
                        superRobot.SetMotors(0, 0, rotate);
                        superRobot.Drive();
                        driveOpState = OperState.ROTATETOPOSITION;
                    }
                    break;
                default :
                    break;

            }
            
            telemetry.addData("MyRotation", zAngle);
            //telemetry.addData("Gyro Angle Y", yAngle);
            //telemetry.addData("Gyro Angle X", xAngle);
            telemetry.addData("Current State: ", driveOpState);
            telemetry.update();
            

        }
    }
}
