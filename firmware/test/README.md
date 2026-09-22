# Host tests

Run on the host, without a board, the two parts of the firmware that are pure
math and independent of hardware:

- `test_odom`: differential-drive kinematics and gyro fusion
- `test_payload`: telemetry JSON serialization

`run_tests.sh` extracts the relevant code from `main/odometry.c` and
`main/telemetry.c` and compiles it against host stubs instead of copying it, so
the tests follow any change to the formulas or the payload.

    ./run_tests.sh
