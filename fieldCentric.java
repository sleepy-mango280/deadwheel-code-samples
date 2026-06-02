@TeleOp(name = "Field Centric Drive")
public class FieldCentricDrive extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    IMU imu;

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();

        while (opModeIsActive()) {

            // step 1 -- read joystick inputs
            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double rotate  =  gamepad1.right_stick_x;

            // step 2 -- get current heading
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // step 3 -- rotate joystick inputs by heading
            double fieldForward = (forward * Math.cos(heading)) - (strafe * Math.sin(heading));
            double fieldStrafe  = (forward * Math.sin(heading)) + (strafe * Math.cos(heading));

            // step 4 -- mecanum mixing with field adjusted inputs
            frontLeft.setPower(fieldForward  + fieldStrafe + rotate);
            frontRight.setPower(fieldForward - fieldStrafe - rotate);
            backLeft.setPower(fieldForward   - fieldStrafe + rotate);
            backRight.setPower(fieldForward  + fieldStrafe - rotate);

            telemetry.addData("Heading (deg)", Math.toDegrees(heading));
            telemetry.addData("Field Forward",  fieldForward);
            telemetry.addData("Field Strafe",   fieldStrafe);
            telemetry.update();
        }
    }
}
