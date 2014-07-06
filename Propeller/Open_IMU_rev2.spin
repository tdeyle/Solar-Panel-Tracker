'' Open_IMU
'' based on Madgwick Algorithm for MARG

'' I wonder if passing the address locations of the raw values to the IMU object, will alleviate the need to pass them for each read call
'' 
'' -> Is there any way to save the multiple instances of creating the quat, matrices, etc.?
'' -> Counter module to count up to 5000us, for the AHRSupdate. Then one to count to 50ms for the printTimer.
''    -> CTRA := %00001 << 26
''    -> FRQA := 80_000_000 per second, we need 0.000_001 or 80 ticks
''    -> PHSA will accumulate every 80 / 1us -> overflow in ~74 minutes

''    -> CTRB := %00001 << 26
''    -> FRQB := 100_000, accumulates every 1ms -> overflows in ~49 days.
'' -> Need to show whether variable is float or integer
'' -> Sub constants as needed
'' -> Systematically display variables, in the order that they are needed for the algo.
'' -> Test the fast inverse formula
'' -> Test the ATAN2 formula
'' -> Make all variables into arrays that need to be, i.e., axes
'' -> Introduce constants for axes, #0, X, Y, Z
'' -> Use fm.FPow for squares of numbers
'' -> Have radians for angles until very end

CON

  _clkmode      = xtal1 + pll16x
  _clkfreq      = 80_000_000

  SDA = 6
  SCL = 7

  #0, X, Y, Z
  '#0, PITCH, ROLL, YAW

  PI_FLOAT = PI
  PIBY2_FLOAT = PI / 2.0
  
  COMPASS_XMIN = -342.0
  COMPASS_XMAX =  287.0
  COMPASS_YMIN = -406.0
  COMPASS_YMAX =  353.0
  COMPASS_ZMIN = -321.0
  COMPASS_ZMAX =  336.0

  INVERSE_XRANGE = 2.0 / (COMPASS_XMAX - COMPASS_XMIN)
  INVERSE_YRANGE = 2.0 / (COMPASS_YMAX - COMPASS_YMIN) 
  INVERSE_ZRANGE = 2.0 / (COMPASS_ZMAX - COMPASS_ZMIN)

  GYRO_SCALE = 0.07 'Double Check this wiht the scale given in the IMU assembly file.
  ACC_SCALE = 0.00390625 '+/- 8g => 16g/4096
  betaDef = 0.08

  TO_RAD = pi / 180.0
  TO_DEG = 180.0 / pi

VAR

  long rawAccl[3]
  long rawGyro[3]
  long rawMag[3]
    
  long gyroSum[3]
  long offset[3]

  long floatMag[3]
  long smoothAcc[3]

  long q0, q1, q2, q3

  'long euler[3]
  long pitch, roll, yaw
   
  long fRawAccl[3]
  long fRawMag[3]
  
  long magnitude

  long i   
  long timer, printTimer
  long G_Dt
  long loopCount               
  long beta
  
  byte debug
  
OBJ

  IMU         : "MinIMUv2-pasm2"               '
  PST         : "Terminally Formatted"
  intmath     : "SL32_INTEngine_2"
    
  fm          : "FME"
  fs          : "FloatString"  
    
PUB Setup | radPitch, radRoll, xMag, yMag, sinRoll, sinPitch, cosRoll, cosPitch

  IMU.start(SCL,SDA)
  'fm.start

  PST.start(250000, @rawAccl)
  PST.Clear
  PST.SetupDisplay

  debug := True

  'Initialize smoothed Accel variables to zero.
  smoothAcc[X] := smoothAcc[Y] := smoothAcc[Z] := 0.0
{
  repeat
    IMU.readMag(@rawMag)
    IMU.readAccl(@rawAccl)
    IMU.readGyro(@rawGyro)      

    fRawAccl[X] := fm.FFloat(rawAccl[X])
    fRawAccl[Y] := fm.FFloat(rawAccl[Y])
    fRawAccl[Z] := fm.FFloat(rawAccl[Z])
    
    fRawMag[X] := fm.FFloat(rawMag[X])
    fRawMag[Y] := fm.FFloat(rawMag[Y])
    fRawMag[Z] := fm.FFloat(rawMag[Z])
    
    smoothAcc[X] := Smoothing(fRawAccl[X], smoothAcc[X])
    smoothAcc[Y] := Smoothing(fRawAccl[Y], smoothAcc[Y])
    smoothAcc[Z] := Smoothing(fRawAccl[Z], smoothAcc[Z])
     
    pitch := fm.FMul((fastatan2(fRawAccl[X], fm.FSqr(fm.FAdd(fm.FMul(fRawAccl[Y], fRawAccl[Y]), fm.FMul(fRawAccl[Z], fRawAccl[Z]))))), To_DEG)
    roll := fm.FMul((fastatan2(fm.FMul(-1.0, fRawAccl[Y]), fm.FSqr(fm.FAdd(fm.FMul(fRawAccl[X], fRawAccl[X]), fm.FMul(fRawAccl[Z], fRawAccl[Z]))))), TO_DEG)

    'Tilt Compensate the Compass
    floatMag[X] :=  fm.FSub(fm.FMul(fm.FSub(fRawMag[X], COMPASS_XMIN), INVERSE_XRANGE), 1.0)
    floatMag[Y] :=  fm.FSub(fm.FMul(fm.FSub(fRawMag[Y], COMPASS_YMIN), INVERSE_YRANGE), 1.0)
    floatMag[Z] :=  fm.FSub(fm.FMul(fm.FSub(fRawMag[Z], COMPASS_ZMIN), INVERSE_ZRANGE), 1.0)
    
    radPitch := fm.FMul(pitch, TO_RAD)
    radRoll := fm.FMul(roll, TO_RAD)

    sinRoll := fm.Sin(radRoll)
    sinPitch := fm.Sin(radPitch)
    cosRoll := fm.cos(radRoll)
    cosPitch := fm.cos(radPitch)

    xMag := fm.FAdd(fm.FMul(floatMag[X], cosPitch), fm.FMul(floatMag[Z], sinPitch))
    yMag := fm.FMul(-1.0, fm.FSub(fm.FAdd(fm.FMul(fm.FMul(floatMag[X], sinRoll), sinPitch), fm.FMul(floatMag[Y], cosRoll)), (fm.FMul(fm.FMul(floatMag[Z], sinRoll), cosPitch)))) 
     
    yaw := fm.FMul(fastAtan2(yMag, xMag), TO_DEG)
     
    
    if fm.FCmp(yaw, 0.0) == -1
      yaw := fm.FAdd(yaw, 360.0)

    PST.printDataLabel(0, 0, string("X: "), rawAccl[X])
    PST.printDataLabel(0, 1, string("Y: "), rawAccl[Y])
    PST.printDataLabel(0, 2, string("Z: "), rawAccl[Z])

    PST.printDataLabelFP(0, 4, string("sinRoll: "), sinRoll)
    PST.printDataLabelFP(0, 5, string("cosRoll: "), cosRoll)
    PST.printDataLabelFP(0, 6, string("sinPitch: "), sinPitch)
    PST.printDataLabelFP(0, 7, string("cosPitch: "), cosPitch)

    PST.printDataLabelFP(0, 8, string("radPitch: "), radPitch)
    PST.printDataLabelFP(0, 9, string("radRoll: "), radRoll)
    
            
    PST.printDataLabel(0, 16, string("magX: "), rawMag[X])
    PST.printDataLabel(0, 17, string("magY: "), rawMag[Y])
    PST.printDataLabel(0, 18, string("magZ: "), rawMag[Z])

    PST.printDataLabelFP(0, 20, string("fmagX: "), frawMag[X])
    PST.printDataLabelFP(0, 21, string("fmagY: "), frawMag[Y])
    PST.printDataLabelFP(0, 22, string("fmagZ: "), frawMag[Z])

    PST.printDataLabelFP(0, 24, string("floatmagX: "), floatMag[X])
    PST.printDataLabelFP(0, 25, string("floatmagY: "), floatMag[Y])
    PST.printDataLabelFP(0, 26, string("floatmagZ: "), floatMag[Z])

    PST.printDataLabelFP(0, 28, string("xMag: "), xMag)
    PST.printDataLabelFP(0, 29, string("yMag: "), yMag)

    'PST.printDataLabelFP(0, 4, string("Float X: "), fm.FFloat(rawAccl[X]))
    'PST.printDataLabelFP(0, 5, string("Float Y: "), fm.FFloat(rawAccl[Y]))
    'PST.printDataLabelFP(0, 6, string("Float Z: "), fm.FFloat(rawAccl[Z]))

    'PST.printDataLabelFP(0, 8, string("Smooth X: "), smoothAcc[X])
    'PST.printDataLabelFP(0, 9, string("Smooth Y: "), smoothAcc[Y])
    'PST.printDataLabelFP(0, 10, string("Smooth Z: "), smoothAcc[Z])

    PST.printDataLabelFP(0, 12, string("pitch: "), pitch)
    PST.printDataLabelFP(0, 13, string("roll: "), roll)
    PST.printDataLabelFP(0, 14, string("yaw: "), yaw)

    'PST.printEuler2
 }
                                             
  'IMU_init
  
  ctra := constant(%00001 << 26)
  frqa := 80
  phsa := 0

  ctrb := constant(%00001 << 26)
  frqb := 100_000
  phsb := 0

  Main

PUB Main | idx

pst.setupMain

repeat
  if phsa => 400_000 '5_000us, 5ms
    G_Dt := fm.FDiv(fm.FFloat(phsa), 1_000_000.0)
    phsa := 0

    'Initialize smoothed Accel variables to zero.
    'smoothAccX := smoothAccY := smoothAccZ := 0

    IMU.readMag(@rawMag)
    IMU.readAccl(@rawAccl)
    IMU.readGyro(@rawGyro)
    
    fRawAccl[0] := fm.FFloat(rawAccl[0])
    fRawAccl[1] := fm.FFloat(rawAccl[1])
    fRawAccl[2] := fm.FFloat(rawAccl[2])
     
    fRawMag[0] := fm.FFloat(rawMag[0])
    fRawMag[1] := fm.FFloat(rawMag[1])
    fRawMag[2] := fm.FFloat(rawMag[2])
     
    floatMag[X] := fm.FSub(fm.FMul(fm.FSub(fRawMag[0], COMPASS_XMIN), INVERSE_XRANGE), 1.0)
    floatMag[Y] := fm.FSub(fm.FMul(fm.FSub(fRawMag[1], COMPASS_YMIN), INVERSE_YRANGE), 1.0)
    floatMag[Z] := fm.FSub(fm.FMul(fm.FSub(fRawMag[2], COMPASS_ZMIN), INVERSE_ZRANGE), 1.0)

    smoothAcc[X] := smoothing(fRawAccl[0], smoothAcc[X])
    smoothAcc[Y] := smoothing(fRawAccl[1], smoothAcc[Y])
    smoothAcc[Z] := smoothing(fRawAccl[2], smoothAcc[Z])    
                                                       
    AHRSupdate
                                  
  if phsb > 4_000_000 '50ms
    phsb := 0
    GetEuler
    PST.updateDisplay

PRI IMU_init | idx, radPitch, radRoll, xMag, yMag, zMag, cosPitch, sinPitch, cosRoll, sinRoll, cosYaw, sinYaw, r11, r12, r13, r21, r22, r23, r31, r32, r33

  beta := betaDef

  'Calculate the initial quaternion
  'Take an average of the gyro readings to remove the bias

  repeat idx from 1 to 500
    IMU.readGyro(@rawGyro)
    IMU.readAccl(@rawAccl)
    IMU.readMag(@rawMag)

    smoothAcc[X] := Smoothing(rawAccl[0], smoothAcc[X])
    smoothAcc[Y] := Smoothing(rawAccl[1], smoothAcc[Y])
    smoothAcc[Z] := Smoothing(rawAccl[2], smoothAcc[Z])

    gyroSum[X] += rawGyro[0]
    gyroSum[Y] += rawGyro[1]
    gyroSum[Z] += rawGyro[2]

    waitcnt(clkfreq/1_000 * 3 + cnt)

    PST.UpdateCalibratingIteration(idx)

  offset[X] := fm.FDiv(fm.FFloat(gyroSum[X]), 500.0)
  offset[Y] := fm.FDiv(fm.FFloat(gyroSum[Y]), 500.0)
  offset[Z] := fm.FDiv(fm.FFloat(gyroSum[Z]), 500.0)

  PST.updateGyroOffset

  'Update the measurements
  IMU.readAccl(@rawAccl)
  IMU.readMag(@rawMag)

  fRawAccl[0] := fm.FFloat(rawAccl[0])
  fRawAccl[1] := fm.FFloat(rawAccl[1])
  fRawAccl[2] := fm.FFloat(rawAccl[2])

  fRawMag[0] := fm.FFloat(rawMag[0])
  fRawMag[1] := fm.FFloat(rawMag[1])
  fRawMag[2] := fm.FFloat(rawMag[2])
   
  'Find the Initial Pitch and Roll - Keep it steady!                                                                         
  pitch := fm.FMul((fastatan2(fRawAccl[X], fm.FSqr(fm.FAdd(fm.FMul(fRawAccl[Y], fRawAccl[Y]), fm.FMul(fRawAccl[Z], fRawAccl[Z]))))), To_DEG)
  roll := fm.FMul((fastatan2(fm.FMul(-1.0, fRawAccl[Y]), fm.FSqr(fm.FAdd(fm.FMul(fRawAccl[X], fRawAccl[X]), fm.FMul(fRawAccl[Z], fRawAccl[Z]))))), TO_DEG)

  if rawAccl[2] > 0
    if rawAccl[0] > 0
      pitch := fm.FSub(180.0, pitch)

    else
      pitch := fm.FSub(-180.0, pitch)

    if rawAccl[1] > 0
      roll := fm.FSub(-180.0, roll)

    else
      roll := fm.FSub(180.0, roll)
  
  'Tilt Compensate the Compass
  floatMag[X] := fm.FSub(fm.FMul(fm.FSub(fRawMag[0], COMPASS_XMIN), INVERSE_XRANGE), 1.0)
  floatMag[Y] := fm.FSub(fm.FMul(fm.FSub(fRawMag[1], COMPASS_YMIN), INVERSE_YRANGE), 1.0)
  floatMag[Z] := fm.FSub(fm.FMul(fm.FSub(fRawMag[2], COMPASS_ZMIN), INVERSE_ZRANGE), 1.0)

  radPitch := fm.FMul(pitch, TO_RAD)
  radRoll := fm.FMul(roll, TO_RAD)
   
  sinRoll := fm.Sin(radRoll)
  sinPitch := fm.Sin(radPitch)
  cosRoll := fm.cos(radRoll)
  cosPitch := fm.cos(radPitch)
   
  xMag := fm.FAdd(fm.FMul(floatMag[X], cosPitch), fm.FMul(floatMag[Z], sinPitch))
  yMag := fm.FMul(-1.0, fm.FSub(fm.FAdd(fm.FMul(fm.FMul(floatMag[X], sinRoll), sinPitch), fm.FMul(floatMag[Y], cosRoll)), (fm.FMul(fm.FMul(floatMag[Z], sinRoll), cosPitch)))) 
   
  yaw := fm.FMul(fastAtan2(yMag, xMag), TO_DEG)
    
  if fm.FCmp(0.0, yaw) == 1 
    yaw := fm.FAdd(yaw, 360.0)

  'Calculate the Rotation Matrix
  'cosPitch := fm.Cos(fm.FMul(pitch, TO_RAD))        
  'sinPitch := fm.Sin(fm.FMul(pitch, TO_RAD))

  'cosRoll := fm.Cos(fm.FMul(roll, TO_RAD))
  'sinRoll := fm.Sin(fm.FMul(roll, TO_RAD))

  cosYaw :=  fm.Cos(fm.FMul(yaw, TO_RAD))
  sinYaw :=  fm.Sin(fm.FMul(yaw, TO_RAD))

  'Calculate the Transpose of the Rotation Matrix
  r11 := fm.FMul(cosPitch, cosYaw)
  r21 := fm.FMul(cosPitch, sinYaw)
  r31 := fm.FMul(-1.0, sinPitch)

  r12 := fm.FAdd(fm.FMul(-1.0, fm.FMul(cosRoll, sinYaw)), fm.FMul(fm.FMul(sinRoll, sinPitch), cosYaw))
  r22 := fm.FAdd(fm.FMul(cosRoll, cosYaw), fm.FMul(fm.FMul(sinRoll, sinPitch), sinYaw))
  r32 := fm.FMul(sinRoll, cosPitch)

  r13 := fm.FAdd(fm.FMul(sinRoll, sinYaw), fm.FMul(fm.FMul(cosRoll, sinPitch), cosYaw))
  r23 := fm.FAdd(fm.FMul(-1.0, fm.FMul(sinRoll, cosYaw)), fm.FMul(fm.fMul(cosRoll, sinPitch), sinYaw))
  r33 := fm.FMul(cosRoll, cosPitch)

  'Convert to Initial Quaternion
  q0 := fm.FMul(0.5, fm.FSqr(fm.FAdd(fm.FAdd(fm.FAdd(1.0, r11), r22), r33)))
  q1 := fm.FDiv(fm.FSub(r32, r23), fm.FMul(4.0, q0))
  q2 := fm.FDiv(fm.FSub(r13, r31), fm.FMul(4.0, q0))
  q3 := fm.FDiv(fm.FSub(r21, r12), fm.FMul(4.0, q0))

  PST.updateInit

PRI AHRSupdate | table, _a, _b, _c, _d, _e, _f, _ax, _ay, _az, _mx, _my, _mz, gx, gy, gz, ax, ay, az, mx, my, mz, recipNorm, s0, s1, s2, s3, qDot1, qDot2, qDot3, qDot4, hx, hy, _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3

  gx := 0.01 'fm.FMul(fm.FMul(fm.FSub(fm.FFloat(rawGyro[0]), offSet[X]), GYRO_SCALE), TO_RAD)
  gy := -0.01 'fm.FMul(fm.FMul(fm.FSub(fm.FFloat(rawGyro[1]), offSet[Y]), GYRO_SCALE), TO_RAD)
  gz := 0.00 'fm.FMul(fm.FMul(fm.FSub(fm.FFloat(rawGyro[2]), offSet[Z]), GYRO_SCALE), TO_RAD)

  ax := 28.0 'fm.FMul(-1.0, fRawAccl[X]) 'fm.FFloat(rawAccl[0]))
  ay := -248.0 'fm.FMul(-1.0, fRawAccl[Y]) 'fm.FFloat(rawAccl[1]))
  az := -36.00 'fm.FMul(-1.0, fRawAccl[Z]) 'fm.FFloat(rawAccl[2]))

  mx := 0.22 'floatMag[X]
  my := -0.89 'floatMag[Y]
  mz := 0.16 'floatMag[Z]

  q0 := 0.76
  q1 := 0.65
  q2 := -0.40
  q3 := 0.05
  
  'Rate of change of quaternion from gyroscope
  qDot1 := fm.FMul(0.5, fm.FSub(fm.FSub(fm.FMul(fm.FNeg(q1), gx), fm.FMul(q2, gy)), fm.FMul(q3, gz)))
  qDot2 := fm.FMul(0.5, fm.FSub(fm.FAdd(fm.FMul(q0, gx), fm.FMul(q2, gz)), fm.FMul(q3, gy)))
  qDot3 := fm.FMul(0.5, fm.FAdd(fm.FSub(fm.FMul(q0, gy), fm.FMul(q1, gz)), fm.FMul(q3, gx)))
  qDot4 := fm.FMul(0.5, fm.FSub(fm.FAdd(fm.FMul(q0, gz), fm.FMul(q1, gy)), fm.FMul(q2, gx)))

  magnitude := fm.FSqr(fm.FAdd(fm.FAdd(fm.FMul(ax, ax), fm.FMul(ay, ay)), fm.FMul(az, az)))

  if ((magnitude > 384.0) AND (magnitude < 128.0))
    ax := 0.0
    ay := 0.0
    az := 0.0

  'Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (NOT(ax == 0.0 AND ay == 0.0 AND az == 0.0)) == TRUE
  {if ax == 0.0
    if ay == 0.0
      if az == 0.0
        table := string("False")
      else
        table := string("True")
    else
      table := string("True")
  else
    table := string("True")
    
    pst.printLabel(50,50,table)   

  if table == string("True") }
    'Normalise accelerometer measurement
    recipNorm := invSqrt(fm.FAdd(fm.FAdd(fm.FMul(ax, ax), fm.FMul(ay, ay)), fm.FMul(az, az)))
    ax := fm.FMul(recipNorm, ax)
    ay := fm.FMul(recipNorm, ay)
    az := fm.FMul(recipNorm, az)

    'Normalise magnetometer measurement
    recipNorm := invSqrt(fm.FAdd(fm.FAdd(fm.FMul(mx, mx), fm.FMul(my, my)), fm.FMul(mz, mz)))
    mx := fm.FMul(recipNorm, mx)
    my := fm.FMul(recipNorm, my)
    mz := fm.FMul(recipNorm, mz)

    'Auxiliary variables to avoid repeated arithmetic
    _2q0mx := fm.FMul(fm.FMul(2.0, q0), mx)
    _2q0my := fm.FMul(fm.FMul(2.0, q0), my)
    _2q0mz := fm.FMul(fm.FMul(2.0, q0), mz)
    _2q1mx := fm.FMul(fm.FMul(2.0, q1), mx)
    _2q0 := fm.FMul(2.0, q0)
    _2q1 := fm.FMul(2.0, q1)
    _2q2 := fm.FMul(2.0, q2)
    _2q3 := fm.FMul(2.0, q3)
    _2q0q2 := fm.FMul(fm.FMul(2.0, q0), q2)
    _2q2q3 := fm.FMul(fm.FMul(2.0, q2), q3)
    q0q0 := fm.FMul(q0, q0)
    q0q1 := fm.FMul(q0, q1)
    q0q2 := fm.FMul(q0, q2)
    q0q3 := fm.FMul(q0, q3)
    q1q1 := fm.FMul(q1, q1)
    q1q2 := fm.FMul(q1, q2)
    q1q3 := fm.FMul(q1, q3)
    q2q2 := fm.FMul(q2, q2)
    q2q3 := fm.FMul(q2, q3)
    q3q3 := fm.FMul(q3, q3)


    'Reference direction of Earth's magnetic field
    hx := fm.FSub(fm.FSub(fm.FAdd(fm.FAdd(fm.FAdd(fm.FAdd(fm.FSub(fm.FMul(mx, q0q0), fm.FMul(_2q0my, q3)), fm.FMul(_2q0mz, q2)), fm.FMul(mx, q1q1)), fm.FMul(fm.FMul(_2q1, my), q2)), fm.FMul(fm.FMul(_2q1, mz), q3)), fm.FMul(mx, q2q2)), fm.FMul(mx, q3q3))
    hy := fm.FSub(fm.FAdd(fm.FAdd(fm.FSub(fm.FAdd(fm.FSub(fm.FAdd(fm.FMul(_2q0mx, q3), fm.FMul(my, q0q0)), fm.FMul(_2q0mz, q1)), fm.FMul(_2q1mx, q2)), fm.FMul(my, q1q1)), fm.FMul(my, q2q2)), fm.FMul(fm.FMul(_2q2, mz), q3)), fm.FMul(my, q3q3))
    _2bx := fm.FSqr(fm.FAdd(fm.FMul(hx, hx), fm.FMul(hy, hy)))
    _2bz := fm.FAdd(fm.FSub(fm.FAdd(fm.FSub(fm.FAdd(fm.FAdd(fm.FAdd(fm.FMul(fm.FNeg(_2q0mx), q2), fm.FMul(_2q0my, q1)), fm.FMul(mz, q0q0)), fm.FMul(_2q1mx, q3)), fm.FMul(mz, q1q1)), fm.FMul(fm.FMul(_2q2, my), q3)), fm.FMul(mz, q2q2)), fm.FMul(mz, q3q3))
    _4bx := fm.FMul(2.0, _2bx)
    _4bz := fm.FMul(2.0, _2bz)

    'More auxillary variables
    _ax := fm.FSub(fm.FSub(fm.FMul(2.0, q1q3), _2q0q2), ax)
    _ay := fm.FSub(fm.FAdd(fm.FMul(2.0, q0q1), _2q2q3), ay)
    _az := fm.FSub(fm.FSub(fm.FSub(1.0, fm.FMul(2.0, q1q1)), fm.FMul(2.0, q2q2)), az)
     
    _mx := fm.FSub(fm.FAdd(fm.FMul(_2bx, fm.FSub(fm.FSub(0.5, q2q2), q3q3)), fm.FMul(_2bz, (fm.FSub(q1q3, q0q2)))), mx)
    _my := fm.FSub(fm.FAdd(fm.FMul(_2bx, fm.FSub(q1q2, q0q3)), fm.FMul(_2bz, fm.FAdd(q0q1, q2q3))), my)
    _mz := fm.FSub(fm.FAdd(fm.FMul(_2bx, fm.FAdd(q0q2, q1q3)), fm.FMul(_2bz, fm.FSub(fm.FSub(0.5, q1q1), q2q2))), mz)
     
    'Constant to the s0 equation
    's0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    _a := fm.FMul(fm.FNeg(_2q2), _ax)
    _b := fm.FMul(_2q1, _ay)
    _c := fm.FMul(fm.FMul(_2bz, q2), _mx)
    _d := fm.FMul(fm.FAdd(fm.FMul(fm.FNeg(_2bz), q3), fm.FMul(_2bz, q1)), _my)
    _e := fm.FMul(fm.FMul(_2bx, q2), _mz)
     
    s0 := fm.FAdd(fm.FAdd(fm.FSub(fm.FAdd(_a, _b), _c), _d), _e)

     
    'Constant to the s1 equation
    's1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    _a := fm.FMul(_2q3, _ax)
    _b := fm.FMul(_2q0, _ay)
    _c := fm.FMul(fm.FMul(4.0, q1), _az)
    _d := fm.FMul(fm.FMul(_2bz, q3), _mx)
    _e := fm.FMul(fm.FAdd(fm.FMul(_2bx, q2), fm.FMul(_2bz, q0)), _my)
    _f := fm.FMul(fm.FSub(fm.fMul(_2bx, q3), fm.FMul(_4bz, q1)), _mz)
     
    s1 := fm.FAdd(fm.FAdd(fm.FAdd(fm.FSub(fm.FAdd(_a, _b), _c), _d), _e), _f)
         
    'Constant to the s2 equation.
    's2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    _a := fm.FMul(fm.FNeg(_2q0), _ax)
    _b := fm.FMul(_2q3, _ay)
    _c := fm.FMul(fm.FMul(fm.FNeg(4.0), q2), _az)
    _d := fm.FMul(fm.FSub(fm.FMul(fm.FNeg(_4bx), q2), fm.FMul(_2bz, q0)), _mx)
    _e := fm.FMul(fm.FAdd(fm.FMul(_2bx, q1), fm.FMul(_2bz, q3)), _my)
    _f := fm.FMul(fm.FSub(fm.FMul(_2bx, q0), fm.FMul(_4bz, q2)), _mz)
     
    s2 := fm.FAdd(fm.FAdd(fm.FAdd(fm.FSub(fm.FAdd(_a, _b), _c), _d), _e), _f)
    
    'Constant to the s3 equation.
    's3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    _a := fm.FMul(_2q1, _ax)
    _b := fm.FMul(_2q2, _ay)
    _c := fm.FMul(fm.FAdd(fm.FMul(fm.FNeg(_4bx), q3), fm.FMul(_2bz, q1)), _mx)
    _d := fm.FMul(fm.FAdd(fm.FMul(fm.FNeg(_2bx), q0), fm.FMul(_2bz, q2)), _my)
    _e := fm.FMul(fm.FMul(_2bx, q1), _mz)
     
    s3 := fm.FAdd(fm.FAdd(fm.FAdd(fm.FAdd(_a, _b), _c), _d), _e)

    recipNorm := invSqrt(fm.FAdd(fm.FAdd(fm.FAdd(fm.FMul(s0, s0), fm.FMul(s1, s1)), fm.FMul(s2, s2)), fm.FMul(s3, s3)))
    s0 := fm.FMul(recipNorm, s0)
    s1 := fm.FMul(recipNorm, s1)
    s2 := fm.FMul(recipNorm, s2)
    s3 := fm.FMul(recipNorm, s3)

    'Apply feedback step
    qDot1 := fm.FSub(qDot1, fm.FMul(beta, s0))
    qDot2 := fm.FSub(qDot2, fm.FMul(beta, s1))
    qDot3 := fm.FSub(qDot3, fm.FMul(beta, s2))
    qDot4 := fm.FSub(qDot4, fm.FMul(beta, s3))

  'Integrate rate of change of quaternion to yield quaternion
  q0 := fm.FAdd(fm.FMul(qDot1, G_Dt), q0)
  q1 := fm.FAdd(fm.FMul(qDot2, G_Dt), q1)
  q2 := fm.FAdd(fm.FMul(qDot3, G_Dt), q2)
  q3 := fm.FAdd(fm.FMul(qDot4, G_Dt), q3)

  'Normalise quaternion
  recipNorm := invSqrt(fm.FAdd(fm.FAdd(fm.FAdd(fm.FMul(q0, q0), fm.FMul(q1, q1)), fm.FMul(q2, q2)), fm.FMul(q3, q3)))
  q0 := fm.FMul(q0, recipNorm)
  q1 := fm.FMul(q1, recipNorm)
  q2 := fm.FMul(q2, recipNorm)
  q3 := fm.FMul(q3, recipNorm)

  PST.printDataLabelFP(50,50,string("q0: "), q0)
  PST.printDataLabelFP(50,51,string("q1: "), q1)
  PST.printDataLabelFP(50,52,string("q2: "), q2)
  PST.printDataLabelFP(50,53,string("q3: "), q3)

  PST.printDataLabelFP(50,54,string("ax: "), ax)
  PST.printDataLabelFP(50,55,string("ay: "), ay)
  PST.printDataLabelFP(50,56,string("az: "), az)
  PST.printDataLabelFP(50,57,string("G_Dt: "), G_Dt)


PRI GetEuler
'Converts quaternion radings into Euler Angles, in degrees.
roll := ToDeg(fastAtan2(fm.FMul(2.0, fm.FAdd(fm.FMul(q0, q1), fm.FMul(q2, q3))), fm.FSub(1.0, fm.FMul(2.0, fm.FAdd(fm.FMul(q1, q1), fm.FMul(q2, q2))))))
pitch := ToDeg(fm.asin(fm.FMul(2.0, fm.FSub(fm.FMul(q0, q2), fm.FMul(q3, q1)))))
yaw := ToDeg(fastAtan2(fm.FMul(2.0, fm.FAdd(fm.FMul(q0, q3), fm.FMul(q1, q2))), fm.FSub(1.0, fm.fMul(2.0, (fm.FAdd(fm.FMul(q2, q2), fm.FMul(q3, q3)))))))

if yaw < 0.0
  yaw := fm.FAdd(yaw, 360.0)

PRI Smoothing(raw, smooth)
'Performs a basic smoothing function on the two inputs

  return fm.FAdd(fm.FMul(raw, 0.15), fm.FMul(smooth, 0.85))

PRI BrownLinearExpo(raw, smooth) | single_smoothed, double_smoothed, est_a, est_b, estimate, a
'Performs a double Brown Linear Exponential Smoothing factor on the inputs given.

  a := 0.1
  
  single_smoothed := fm.FAdd(fm.FMul(a, raw), fm.FMul(fm.FSub(1.0, a), smooth))
  double_smoothed := fm.FAdd(fm.FMul(a, single_smoothed), fm.FMul(fm.FSub(1.0, a), double_smoothed))

  est_a := fm.FSub(fm.FMul(2.0, single_smoothed), double_smoothed)
  est_b := fm.FMul(fm.FDiv(a, fm.FSub(1.0, a)), fm.FSub(single_smoothed, double_smoothed))
  estimate := fm.FAdd(est_a, est_b)

  return estimate

PRI toDeg(angle_in_rads)

  return fm.FMul(angle_in_rads, TO_DEG) 'fm.FDiv(180.0, PI))

PRI toRad(angle_in_degs)

  return fm.FMul(angle_in_degs, TO_RAD) 'fm.FDiv(PI, 180.0))

PRI fastAtan2(_y, _x) | atan, _z
'Performs a fastAtan2 function on the variables given.
' -> Variables must be floats

  if fm.Fcmp(_x, 0.0) == 0 'x == 0.0
    if fm.Fcmp(_y, 0.0) == 1 '_y > 0.0
      return PIBY2_FLOAT
    if fm.Fcmp(_y, 0.0) == 0 '_y == 0.0
      return 0.0
    return fm.FNeg(PIBY2_FLOAT)

  _z := fm.FDiv(_y, _x)

  if fm.Fcmp(fm.FAbs(_z), 1.0) == -1 '(fm.FAbs(_z)) < 1.0
    atan := fm.FDiv(_z, fm.FAdd(1.0, fm.FMul(0.28,fm.FMul( _z, _z))))
    if fm.Fcmp(_x, 0.0) == -1 '_x < 0.0
      if fm.Fcmp(_y, 0.0) == -1 '_y < 0.0
        return fm.FSub(atan, PI_FLOAT)
      return fm.FAdd(atan, PI_FLOAT)

  else
    atan := fm.FSub(PIBY2_FLOAT, fm.FDiv(_z, fm.FAdd(fm.FMul(_z, _z), 0.28)))
    if fm.Fcmp(_y, 0.0) == -1 '_y < 0.0
      return fm.FSub(atan, PI_FLOAT)

  return atan

PRI fastInvSqrt(number) | _j, _x, _y, _f
'Per_forms a _fast inverse square root
' -> number must be a _float
' Tr_y it, i_f it works, good. I_f not, use the one below.

  _x := fm.FMul(number, 0.5)
  _y := number
  _j := fm.FTrunc(_y)
  _j := $5_f375a86 - (_j >> 1)
  _y := fm.FFloat(_j)
  _y := fm.FMul(_y, fm.FSub(_f, fm.FMul(fm.FMul(_x, _y), _y)))

  return _y

PRI invSqrt(number) | _t

  _t := fm.FDiv(fm.FNeg(1.0), 2.0)

  return fm.Pow(number, _t)
