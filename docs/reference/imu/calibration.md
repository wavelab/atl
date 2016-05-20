## Accelerometer Calibration

Calibration does not change sensor outputs, but it does tell you what the
sensor output is for a known stable reference force in both directions on each
axis. Knowing that, you can calculate the corrected output from a sensor
reading.

### Gravity as a Calibration Reference

Acceleration can be measured in uints of gravitational forcea it is
a relatively stable force and makes a convenient and reliable calibration
reference, most magnetometer calibration methods use gravity as a reference.




## Magnetometer Calibration

For each of the X, Y and Z magnetometer outputs the reading it gives will be
the value of the earth's magnet field in that direction plus the magnetometer
hard iron offset. The hard iron offset will be the same whichever way you turn
it, but if you turn it through 180 degrees it will measure an equal and
opposite value for the Earth's field. So all you have to do to get the offset
is add the two readings and divide by 2.

Assuming the magnetometer defines X and Y as horizontal (in the plane of the
magnetometer board) and Z as vertical (perpendicular to it), a single rotation
will have reversed both X and Y, so we can get both the X and Y offsets. If we
now turn it upside down we will reverse Z and can determine the Z offset. But
we will have also reversed either X or Y once more (depending which way we
flipped it) and can get a second estimate of that offset. Rotate it once more
through 180 degrees and we will have 2 readings in each sense for X, Y and Z.
Add all 4 X readings and divide by 4 and we'll have a slightly better estimate
of the X offset, and the same for Y and Z.

