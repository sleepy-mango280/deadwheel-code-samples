@TeleOp(name = "Encoder Test")
public class EncoderTest extends LinearOpMode {

    // declare the motor/encoder we're plugging the dead wheel into
    DcMotor parallelEncoder;

    @Override
    public void runOpMode() {

        // tell the robot which port it's plugged into
        parallelEncoder = hardwareMap.get(DcMotor.class, "parallel");

        // reset the encoder to 0 at the start
        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // read the raw tick count
            int ticks = parallelEncoder.getCurrentPosition();

            telemetry.addData("Parallel ticks", ticks);
            telemetry.update();
        }
    }
}
