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

// test

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
//blinkin import


// This is not an OpMode.  It is a class that holds all the boring stuff

public class haydenbot {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // create arrays for motors
    public DcMotor[] LeftMotors = new DcMotor[2];
    public DcMotor[] RightMotors = new DcMotor[2];
    public DcMotor[] AllMotors = new DcMotor[4];


    // Motors
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;
    public DcMotor DS = null;

    // just gonna define some variables for encoders real quick dont mind me
    static final double mmPerInch               = 25.4f;    // this is jus math tho
    static final double countsPerRevolution     = 383.6f;   // Gobilda Yellowjacket 435
    static final double wheelDiameterMM         = 96;      // For figuring circumference
    static final double WheelDiameterIn         = wheelDiameterMM / mmPerInch;
    static final double wheelCircumferenceIn    = WheelDiameterIn * Math.PI;
    static final double countsPerInch         = (countsPerRevolution / wheelCircumferenceIn);

    double autoSpeedMult = .5;

    // you will need a reference to your OpMode
    private LinearOpMode OpModeReference;
    public haydenbot(LinearOpMode opMode) {
        OpModeReference = opMode;
    }

    public void initialize() {

        /*// imu stuff from last year cause i dont want to rebuild this from the ground up
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

        imu = OpModeReference.hardwareMap.get(BNO055IMU.class, "IMU");*/

        // wheels
        FL = OpModeReference.hardwareMap.get(DcMotor.class, "FL");
        FR = OpModeReference.hardwareMap.get(DcMotor.class, "FR");
        BL = OpModeReference.hardwareMap.get(DcMotor.class, "BL");
        BR = OpModeReference.hardwareMap.get(DcMotor.class, "BR");
        DS = OpModeReference.hardwareMap.get(DcMotor.class, "DS");

        // motor arrays
        // left
        LeftMotors[0] = FL;
        LeftMotors[1] = BL;
        // right
        RightMotors[0] = FR;
        RightMotors[1] = BR;
        // all
        AllMotors[0] = FL;
        AllMotors[1] = FR;
        AllMotors[2] = BL;
        AllMotors[3] = BR;

        for (DcMotor l : LeftMotors)
            l.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor r : RightMotors)
            r.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotor m : AllMotors){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        DS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DS.setDirection(DcMotor.Direction.FORWARD);
    }


    //Autonomous Methods:

    public void stopDriving (){
        for (DcMotor m : AllMotors)
            m.setPower(0);
    }


    public void drive(double inches, double speed) {

        // Ensure that the opmode is still active
        if (OpModeReference.opModeIsActive()) {

            // calculate the number of ticks you want to travel (cast to integer)
            //int targetTicks = (int) (2 * inches * countsPerInch);
            int targetTicks = (int) (inches * countsPerInch);

            // reset ticks to 0 on all motors
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set target position on all motors
            // mode must be changed to RUN_TO_POSITION

            for(DcMotor m : AllMotors) {
                m.setTargetPosition(targetTicks);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // turn all motors on!
            for (DcMotor m : AllMotors)
                m.setPower(speed/2);

            // just keep looping while both motors are busy
            // stop if driver station stop button pushed
            while (OpModeReference.opModeIsActive() && ((FL.isBusy() && FR.isBusy()) && (BL.isBusy() && BR.isBusy()))) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.addData("FR current", FR.getCurrentPosition());
                OpModeReference.telemetry.addData("FL current", FL.getCurrentPosition());
                OpModeReference.telemetry.addData("BL current", BL.getCurrentPosition());
                OpModeReference.telemetry.addData("BR current", BR.getCurrentPosition());

                OpModeReference.telemetry.update();
            }

            // once all motors get to where they need to be, turn them off
            stopDriving();

            // set motors back to RUN_USING_ENCODERS
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void bbStrafe(double inches, double speed) {

        double driveSpeed = speed * autoSpeedMult; // defined at top of class

        // Ensure that the opmode is still active
        if (OpModeReference.opModeIsActive()) {

            // calculate the number of ticks you want to travel (cast to integer)
            //int targetTicks = (int) (2 * inches * countsPerInch);

            //swag
            int mult = 1;
            if (inches < 0)
                mult = -1;
            int targetTicks = (int) (Math.abs(inches) * 1.4f * countsPerInch);
            int rampThreshInches = (int) (3);
            int rampThreshStart = (int) (rampThreshInches * countsPerInch);
            int rampThreshEnd = (int) (targetTicks - (rampThreshInches * countsPerInch));

            int rampThreshStart1 = (int) (rampThreshStart * 0.05);
            int rampThreshStart2 = (int) (rampThreshStart * 0.125);
            int rampThreshStart3 = (int) (rampThreshStart * 0.25);
            int rampThreshEnd1 = (int) ((targetTicks - rampThreshEnd) + (rampThreshEnd * 0.75));
            int rampThreshEnd2 = (int) ((targetTicks - rampThreshEnd) + (rampThreshEnd * 0.875));
            int rampThreshEnd3 = (int) ((targetTicks - rampThreshEnd) + (rampThreshEnd * 0.95));
            //int rampThreshEnd4 = (int) (targetTicks);


            if (Math.abs(targetTicks) <= (rampThreshStart + rampThreshEnd)) {
                rampThreshStart = targetTicks/2;
                rampThreshEnd = targetTicks/2;
            }


            // reset ticks to 0 on all motors
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set target position on all motors
            // mode must be changed to RUN_TO_POSITION

            FL.setTargetPosition(targetTicks*mult);
            FR.setTargetPosition(-targetTicks*mult);
            BL.setTargetPosition(-targetTicks*mult);
            BR.setTargetPosition(targetTicks*mult);

            for(DcMotor m : AllMotors) {
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // turn all motors on!
//            for (DcMotor m : AllMotors)
//                m.setPower(driveSpeed * 0.1);

            setDriveMotorPower(driveSpeed, 0.1);

            // just keep looping while the motors are busy
            // stop if driver station stop button pushed
            while (OpModeReference.opModeIsActive() && ((FL.isBusy() && FR.isBusy()) && (BL.isBusy() && BR.isBusy()))) {
                int pos = (int) Math.abs(-FR.getCurrentPosition()+FL.getCurrentPosition()+BR.getCurrentPosition()-BL.getCurrentPosition())/4;

                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.addData("pos", pos);
                OpModeReference.telemetry.addData("FR current", -FR.getCurrentPosition());
                OpModeReference.telemetry.addData("FL current", FL.getCurrentPosition());
                OpModeReference.telemetry.addData("BL current", -BL.getCurrentPosition());
                OpModeReference.telemetry.addData("BR current", BR.getCurrentPosition());


                if (pos <= rampThreshStart1){
                    setDriveMotorPower(driveSpeed, .1);
                    OpModeReference.telemetry.addData("threshold", rampThreshStart1);
                    OpModeReference.telemetry.addData("mult", .1);
                }
                else if (pos <= rampThreshStart2){
                    setDriveMotorPower(driveSpeed, .25);
                    OpModeReference.telemetry.addData("threshold", rampThreshStart2);
                    OpModeReference.telemetry.addData("mult", .25);
                }
                else if (pos <= rampThreshStart3){
                    setDriveMotorPower(driveSpeed, .5);
                    OpModeReference.telemetry.addData("threshold", rampThreshStart3);
                    OpModeReference.telemetry.addData("mult", .5);
                }
                else if (pos <= rampThreshStart){
                    setDriveMotorPower(driveSpeed, .75);
                    OpModeReference.telemetry.addData("threshold", rampThreshStart);
                    OpModeReference.telemetry.addData("mult", .75);
                }

                else if (pos >= rampThreshEnd3){
                    setDriveMotorPower(driveSpeed, .1);
                    OpModeReference.telemetry.addData("threshold", rampThreshEnd3);
                    OpModeReference.telemetry.addData("mult", .1);
                }
                else if (pos >= rampThreshEnd2){
                    setDriveMotorPower(driveSpeed, .25);
                    OpModeReference.telemetry.addData("threshold", rampThreshEnd2);
                    OpModeReference.telemetry.addData("mult", .25);
                }
                else if (pos >= rampThreshEnd1){
                    setDriveMotorPower(driveSpeed, .5);
                    OpModeReference.telemetry.addData("threshold", rampThreshEnd1);
                    OpModeReference.telemetry.addData("mult", .5);
                }
                else if (pos >= rampThreshEnd){
                    setDriveMotorPower(driveSpeed, .75);
                    OpModeReference.telemetry.addData("threshold", rampThreshEnd);
                    OpModeReference.telemetry.addData("mult", .75);
                }
                else {
                    setDriveMotorPower(driveSpeed, 1);
                    OpModeReference.telemetry.addData("threshold", "none :)");
                    OpModeReference.telemetry.addData("mult", 1);
                }
                OpModeReference.telemetry.update();
            }

            // once all motors get to where they need to be, turn them off
            stopDriving();

            // set motors back to RUN_USING_ENCODERS
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void bbDrive(double inches, double speed) {

        double driveSpeed = speed * autoSpeedMult; // defined at top of class

        // Ensure that the opmode is still active
        if (OpModeReference.opModeIsActive()) {

            // calculate the number of ticks you want to travel (cast to integer)
            //int targetTicks = (int) (2 * inches * countsPerInch);

            //swag
            int mult = 1;
            if (inches < 0)
                mult = -1;
            int targetTicks = (int) (Math.abs(inches) * countsPerInch);
            int rampThreshInches = (int) (3);
            int rampThreshStart = (int) (rampThreshInches * countsPerInch);
            int rampThreshEnd = (int) (targetTicks - (rampThreshInches * countsPerInch));

            int rampThreshStart1 = (int) (rampThreshStart * 0.1);
            int rampThreshStart2 = (int) (rampThreshStart * 0.25);
            int rampThreshStart3 = (int) (rampThreshStart * 0.5);
            int rampThreshEnd1 = (int) ((targetTicks - rampThreshEnd) + (rampThreshEnd * 0.5));
            int rampThreshEnd2 = (int) ((targetTicks - rampThreshEnd) + (rampThreshEnd * 0.75));
            int rampThreshEnd3 = (int) ((targetTicks - rampThreshEnd) + (rampThreshEnd * 0.9));
            //int rampThreshEnd4 = (int) (targetTicks);


            if (Math.abs(targetTicks) <= (rampThreshStart + rampThreshEnd)) {
                rampThreshStart = targetTicks/2;
                rampThreshEnd = targetTicks/2;
            }


            // reset ticks to 0 on all motors
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set target position on all motors
            // mode must be changed to RUN_TO_POSITION

            for(DcMotor m : AllMotors) {
                m.setTargetPosition(targetTicks*mult);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // turn all motors on!
//            for (DcMotor m : AllMotors)
//                m.setPower(driveSpeed * 0.1);

            setDriveMotorPower(driveSpeed, 0.1);

            // just keep looping while the motors are busy
            // stop if driver station stop button pushed
            while (OpModeReference.opModeIsActive() && ((FL.isBusy() && FR.isBusy()) && (BL.isBusy() && BR.isBusy()))) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.addData("FR current", FR.getCurrentPosition());
                OpModeReference.telemetry.addData("FL current", FL.getCurrentPosition());
                OpModeReference.telemetry.addData("BL current", BL.getCurrentPosition());
                OpModeReference.telemetry.addData("BR current", BR.getCurrentPosition());

                int pos = (int) Math.abs(FR.getCurrentPosition()+FL.getCurrentPosition()+BR.getCurrentPosition()+BL.getCurrentPosition())/4;

                if (pos <= rampThreshStart1){
                    setDriveMotorPower(driveSpeed, .1);
                    OpModeReference.telemetry.addData("threshold", rampThreshStart1);
                    OpModeReference.telemetry.addData("mult", .1);
                }
                else if (pos <= rampThreshStart2){
                    setDriveMotorPower(driveSpeed, .25);
                    OpModeReference.telemetry.addData("threshold", rampThreshStart2);
                    OpModeReference.telemetry.addData("mult", .25);
                }
                else if (pos <= rampThreshStart3){
                    setDriveMotorPower(driveSpeed, .5);
                    OpModeReference.telemetry.addData("threshold", rampThreshStart3);
                    OpModeReference.telemetry.addData("mult", .5);
                }
                else if (pos <= rampThreshStart){
                    setDriveMotorPower(driveSpeed, .75);
                    OpModeReference.telemetry.addData("threshold", rampThreshStart);
                    OpModeReference.telemetry.addData("mult", .75);
                }

                else if (pos >= rampThreshEnd3){
                    setDriveMotorPower(driveSpeed, .1);
                    OpModeReference.telemetry.addData("threshold", rampThreshEnd3);
                    OpModeReference.telemetry.addData("mult", .1);
                }
                else if (pos >= rampThreshEnd2){
                    setDriveMotorPower(driveSpeed, .25);
                    OpModeReference.telemetry.addData("threshold", rampThreshEnd2);
                    OpModeReference.telemetry.addData("mult", .25);
                }
                else if (pos >= rampThreshEnd1){
                    setDriveMotorPower(driveSpeed, .5);
                    OpModeReference.telemetry.addData("threshold", rampThreshEnd1);
                    OpModeReference.telemetry.addData("mult", .5);
                }
                else if (pos >= rampThreshEnd){
                    setDriveMotorPower(driveSpeed, .75);
                    OpModeReference.telemetry.addData("threshold", rampThreshEnd);
                    OpModeReference.telemetry.addData("mult", .75);
                }
                else {
                    setDriveMotorPower(driveSpeed, 1);
                    OpModeReference.telemetry.addData("threshold", "none :)");
                    OpModeReference.telemetry.addData("mult", 1);
                }
                OpModeReference.telemetry.update();
            }

            // once all motors get to where they need to be, turn them off
            stopDriving();

            // set motors back to RUN_USING_ENCODERS
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void setDriveMotorPower(double methodSpeed, double stepMult) {
        for (DcMotor m : AllMotors)
            m.setPower(methodSpeed * stepMult);
    }

    private void strafe (double inches, double speed){
        if (OpModeReference.opModeIsActive()) {
            //int targetTicks = (int) (countsPerInch * inches * 2);
            int targetTicks = (int) (countsPerInch * inches);
            for (DcMotor m : AllMotors) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            FL.setTargetPosition(targetTicks);
            FR.setTargetPosition(-targetTicks);
            BL.setTargetPosition(-targetTicks);
            BR.setTargetPosition(targetTicks);

            for (DcMotor m : AllMotors)
                m.setPower(speed * Math.sqrt(autoSpeedMult));

            while (OpModeReference.opModeIsActive() && (FL.isBusy() && FR.isBusy() && BL.isBusy()
                    && BR.isBusy())) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.update();
            }
            stopDriving();
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

//  im not sure why multiplying it by 1.05 works, but it seems to fix it
    public void strafeL (double inches, double speed) {
        strafe(-inches * 1.05, speed);
    }
    public void strafeR (double inches, double speed) {
        strafe(inches * 1.05, speed);
    }


    //Tele-op Methods:


    public void mecanum () {

                double speed = -OpModeReference.gamepad1.left_stick_y / Math.sqrt(2);
                double strafe = -OpModeReference.gamepad1.left_stick_x;
                double rotate = -OpModeReference.gamepad1.right_stick_x;
                double movingSpeed;

                if (OpModeReference.gamepad1.left_bumper) {
                    movingSpeed = 0.5;
                }
                else {
                    movingSpeed = 1;
                }

                double leftFrontDir = Range.clip((speed - strafe - rotate), -1, 1) * movingSpeed;
                double rightFrontDir = Range.clip((speed + strafe + rotate), -1, 1) * movingSpeed;
                double leftBackDir = Range.clip((speed + strafe - rotate), -1, 1) * movingSpeed;
                double rightBackDir = Range.clip((speed - strafe + rotate), -1, 1) * movingSpeed;

                FL.setPower(leftFrontDir);
                FR.setPower(rightFrontDir);
                BL.setPower(leftBackDir);
                BR.setPower(rightBackDir);

//        OpModeReference.telemetry.addData("Central Velocity", speed*movingSpeed);
//        OpModeReference.telemetry.addData("Lateral Velocity", strafe*movingSpeed);
//        OpModeReference.telemetry.addData("Rotation", rotate*movingSpeed);
    }

    public void mecanumInv () {

        double speed = OpModeReference.gamepad1.left_stick_y / Math.sqrt(2);
        double strafe = OpModeReference.gamepad1.left_stick_x;
        double rotate = -OpModeReference.gamepad1.right_stick_x;
        double movingSpeed;

        if (OpModeReference.gamepad1.left_bumper) {
            movingSpeed = 0.5;
        }
        else {
            movingSpeed = 1;
        }

        double leftFrontDir = Range.clip((speed - strafe - rotate), -1, 1) * movingSpeed;
        double rightFrontDir = Range.clip((speed + strafe + rotate), -1, 1) * movingSpeed;
        double leftBackDir = Range.clip((speed + strafe - rotate), -1, 1) * movingSpeed;
        double rightBackDir = Range.clip((speed - strafe + rotate), -1, 1) * movingSpeed;

        FL.setPower(leftFrontDir);
        FR.setPower(rightFrontDir);
        BL.setPower(leftBackDir);
        BR.setPower(rightBackDir);

//        OpModeReference.telemetry.addData("Central Velocity", speed*movingSpeed);
//        OpModeReference.telemetry.addData("Lateral Velocity", strafe*movingSpeed);
//        OpModeReference.telemetry.addData("Rotation", rotate*movingSpeed);
    }

    // this is a method to initialize the camera, vuforia, and tensor flow
    // it needs to be called before WaitForStart()
    // keep separate from other initialization code - only use in autonomous where want to detect rings
    // the result of this method will be one of our predefined strings from above
    // in your opmode, you can check for RINGS_NONE, RINGS_ONE, or RINGS_FOUR
    // and then decide what to do based on that

}
