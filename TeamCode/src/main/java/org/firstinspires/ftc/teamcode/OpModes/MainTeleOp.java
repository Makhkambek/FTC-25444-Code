//package org.firstinspires.ftc.teamcode.OpModes;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.SubSystems.Robot;
//
//@Disabled
//@TeleOp(name = "MainTeleOp", group = "TeleOp")
//public class MainTeleOp extends OpMode {
//
//    private Robot robot;
//
//    @Override
//    public void init() {
//        robot = new Robot(hardwareMap, telemetry);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//    }
//
//    @Override
//    public void init_loop() {
//        if (robot.vision.hasValidSequenceTag()) {
//            telemetry.addData(" Tag Found", "ID: %d", robot.vision.getBestTagId());
//            telemetry.addData("Range", "%.2f m", robot.vision.getBestTagRange());
//        } else {
//            telemetry.addData(" Searching...", "No valid tag");
//        }
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        robot.start();
//        telemetry.clearAll();
//    }
//
//    @Override
//    public void loop() {
//        robot.update(gamepad1, gamepad2, telemetry);
////        displayTelemetry();
//    }
//
//    @Override
//    public void stop() {
//        robot.stop();
//    }
//}
