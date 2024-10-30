package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class ServoTest extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Servo testServo = hardwareMap.servo.get("testServo");
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            testServo.setPosition(1);
        }
    }
}
