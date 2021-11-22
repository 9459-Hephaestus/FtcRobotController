package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="DsDepotRedAuto", group="Linear Opmode")
public class DSDepotRedAuto extends LinearOpMode{
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
        hayden.Kraken.setPosition(1);

        //drive to spinner
        hayden.strafeR(8, speed1);
        hayden.drive(22, speed1);
        hayden.strafeL(1.5, 1);
        //spin, hold, stop
        hayden.DS.setPower(-0.5);
        sleep(3000);
        hayden.DS.setPower(0);
        //move to storage unit
        hayden.strafeR(21, speed1ds);
        //hayden.drive(-.3, speed1ds);

    }
}
