package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class armTest extends OpMode {
    DcMotor armLeft;
    Servo armServo;
    public void init() {
        armLeft = hardwareMap.get(DcMotor.class,"armLeft");
        armServo = hardwareMap.get(Servo.class,"armServo");
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        armLeft.setPower(0);
        if (gamepad1.a){
            armLeft.setPower(1);
        }
        if (gamepad1.b){
            armLeft.setPower(-1);
        }
        if (gamepad1.x){
            armServo.setPosition(0);
        }
        if (gamepad1.y){
            armServo.setPosition(1);
        }

    }
}
