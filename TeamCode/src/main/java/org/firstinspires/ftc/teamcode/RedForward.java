package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Forward (test)", group="Linear Opmode")
public class RedForward extends LinearOpMode{
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

        //drive to white box
        hayden.drive(40, speed1);
    }
}
