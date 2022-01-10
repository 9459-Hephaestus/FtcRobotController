package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="DSWarehouseBlueAuto", group="Linear Opmode")
public class DSWarehouseBlueAuto extends LinearOpMode{
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
        double speed1ds = 0.3;
//        hayden.Kraken.setPosition(1);

        //drive to spinner
        hayden.drive(-6, speed1);
        hayden.strafeL(11, speed1);
        hayden.drive(1, speed1ds);
        //spin, hold, stop
        hayden.DS.setPower(0.13);
        sleep(19000);
        hayden.DS.setPower(0);
        //move to warehouse
        hayden.strafeR(11, speed1);
        hayden.turn(-88, 0.3);
        hayden.strafeL(5, 1);
        hayden.drive(90, 1);
        //hayden.drive(.3, speed1ds);

    }
}

