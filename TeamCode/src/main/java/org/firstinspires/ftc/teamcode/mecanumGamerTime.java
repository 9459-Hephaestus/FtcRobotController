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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum gamertime 🤣👌😎", group="Linear Opmode")

public class mecanumGamerTime extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private DcMotor DS = null;
    private DcMotor AL = null;
    private DcMotor AS = null;
    //private Servo Kraken = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        DS = hardwareMap.get(DcMotor.class, "DS");
        AL = hardwareMap.get(DcMotor.class, "AL");
        AS = hardwareMap.get(DcMotor.class, "AS");
        //Kraken = hardwareMap.get(Servo.class, "Kraken");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        DS.setDirection(DcMotor.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            //double leftPower;
            //double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            /*double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;*/

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            /*leftPower  = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;

            // Send calculated power to wheels
            if (gamepad1.left_bumper) {
                FR.setPower(rightPower);
                FL.setPower(leftPower);
            } else if (gamepad1.right_bumper){
                FR.setPower(rightPower*0.25);
                FL.setPower(leftPower*0.25);
            } else {
                FR.setPower(rightPower * 0.7);
                FL.setPower(leftPower * 0.7);
            }*/


            double speed = -gamepad1.left_stick_y / Math.sqrt(2);
            double strafe = -gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;
            double moveSpeed;

            double duckSpeed = gamepad2.left_stick_y;

            if (gamepad1.left_bumper) {
                moveSpeed = 0.5;
            }
            else if (gamepad2.right_bumper) {
                moveSpeed = 0.25;
            }
            else {
                moveSpeed = 1;
            }

            double FL_Dir = Range.clip((speed - strafe - rotate), -1, 1) * moveSpeed;
            double FR_Dir = Range.clip((speed + strafe + rotate), -1, 1) * moveSpeed;
            double BL_Dir = Range.clip((speed + strafe - rotate), -1, 1) * moveSpeed;
            double BR_Dir = Range.clip((speed - strafe + rotate), -1, 1) * moveSpeed;


            FL.setPower(FL_Dir);
            FR.setPower(FR_Dir);
            BL.setPower(BL_Dir);
            BR.setPower(BR_Dir);


            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            FL.setPower(v1);
            FR.setPower(v2);
            BL.setPower(v3);
            BR.setPower(v4);
//            if(gamepad2.left_trigger > .3){
//               Kraken.setPosition(0);
//            } else if(gamepad2.right_trigger >.3){
//                Kraken.setPosition(1);
//            }

            if (gamepad2.left_bumper){
                DS.setPower(duckSpeed);
            } else {
                DS.setPower(duckSpeed * 0.5);
            }


            if (gamepad2.dpad_right){
                AL.setPower(0);
            }else if (gamepad2.dpad_left) {
                AL.setPower(-.35);
            }else {
                AL.setPower(.35);
            }

            if(gamepad2.left_trigger > .3){
                AL.setPower(.6);


            } else if(gamepad2.right_trigger >.3){
                AL.setPower(-.6);
            } else {
                AL.setPower(-.0
                );
            }

            if (gamepad2.right_stick_y > .3){
                if(gamepad2.right_bumper) {
                    AS.setPower(-.3);}
                else {
                    AS.setPower(-.3);
                }
            } else if (gamepad2.right_stick_y < -.3){
                if(gamepad2.right_bumper) {
                    AS.setPower(.4);}
                else {
                    AS.setPower(.4);
                }
            }
            else {
                AS.setPower(0);
            }}}}
