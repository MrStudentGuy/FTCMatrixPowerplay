//package org.firstinspires.ftc.teamcode.Tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.ftc9974.thorcore.seasonal.powerplay.PowerPlaySeeker;
//import org.ftc9974.thorcore.util.TimingUtilities;
//import org.ftc9974.thorcore.vision.Seeker;
//
//import java.io.IOException;
//
//
//@TeleOp
//public class ThorCoreTest extends LinearOpMode {
//    private PowerPlaySeeker ppSeeker;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.setMsTransmissionInterval(20);
//        telemetry.setItemSeparator("\n\t");
//
//        telemetry.addLine("Initializing...");
//        telemetry.update();
//
//        try {
//            // initialize the seeker
//            ppSeeker = new PowerPlaySeeker("seeker", hardwareMap);
//
//            // begin streaming camera frames to the Driver Station
//            ppSeeker.startStreaming();
//        } catch (IOException e) {
//            e.printStackTrace();
//            RobotLog.ee("Seeker", e, "IOException initializing seeker");
//            throw new RuntimeException("IOException initializing seeker", e);
//        }
//
//
//
//
//        while(opModeInInit()){
//            displaySignature("Blue Cone & Line", ppSeeker.blueConeAndLine);
//            displaySignature("Red Cone & Line", ppSeeker.redConeAndLine);
//            displaySignature("Pole", ppSeeker.pole);
//
//            if (gamepad1.dpad_down) {
//                ppSeeker.hideSignatures();
//            } else if (gamepad1.dpad_left) {
//                ppSeeker.showSignature(ppSeeker.blueConeAndLine);
//            } else if (gamepad1.dpad_up) {
//                ppSeeker.showSignature(ppSeeker.redConeAndLine);
//            } else if (gamepad1.dpad_right) {
//                ppSeeker.showSignature(ppSeeker.pole);
//            }
//        }
//
//        ppSeeker.hideSignatures();
//        ppSeeker.stopStreaming();
//
//
//
//        while(opModeIsActive()){
//            displaySignature("Blue Cone & Line", ppSeeker.blueConeAndLine);
//            displaySignature("Red Cone & Line", ppSeeker.redConeAndLine);
//            displaySignature("Pole", ppSeeker.pole);
//        }
//        TimingUtilities.runOnSeparateThread(ppSeeker::shutdown);
//    }
//
//    private void displaySignature(String name, Seeker.Signature signature) {
//        telemetry.addLine(signature.hasLock() ? name + " [LOCKED]" : name)
//                .addData("", "")
//                .addData("X", signature.getX())
//                .addData("Y", signature.getY())
//                .addData("Lock Strength", signature.getLockStrength());
//    }
//}
