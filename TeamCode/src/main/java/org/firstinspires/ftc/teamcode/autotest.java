package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name="autotest", group="Linear Opmode")
public class autotest extends LinearOpMode{
    @Override
    public void runOpMode() {
        haydenbot hayden = new haydenbot(this);
        hayden.initialize();
        waitForStart();

//        double speed1 = 0.75;
//        double speed2 = 0.5;
//        double speed1 = 0.375;
//        double speed2 = 0.25;
        double speed1 = 0.6;
        double speed2 = 0.4;

        //get away from wall
        hayden.strafeL(.5, speed1);
        //drive to spinner
        hayden.drive(6, speed1);
        //spin, hold, stop
        hayden.DS.setPower(1);
        sleep(3000);
        hayden.DS.setPower(0);
        //move to storage unit
        hayden.strafeL(12, speed1);

    }
}
