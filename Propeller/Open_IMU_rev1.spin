'' Open_IMU
'' based on Madgwick Algorithm for MARG

CON

  _clkmode      = xtal1 + pll16x
  _clkfreq      = 80_000_000

  SDA = 6
  SCL = 7

  COMPASS_XMIN = -345.0
  COMPASS_XMAX =  216.0
  COMPASS_YMIN = -347.0
  COMPASS_YMAX =  210.0
  COMPASS_ZMIN = -305.0
  COMPASS_ZMAX =  249.0

  INVERSE_XRANGE = 2.0 / (COMPASS_XMAX - COMPASS_XMIN)
  INVERSE_YRANGE = 2.0 / (COMPASS_YMAX - COMPASS_YMIN) 
  INVERSE_ZRANGE = 2.0 / (COMPASS_ZMAX - COMPASS_ZMIN)

  SAMPLE_FREQ = 512.0
  TWO_KP_DEF = 2.0 * 0.05
  TWO_KI_DEF = 2.0 * 0.00

  GYRO_SCALE = 0.07
                         
  PIBY2_FLOAT = 1.5707963
  PI_FLOAT = 3.14159265

  MILLI = _clkfreq / 1_000

  TO_RAD = pi / 180.0
  TO_DEG = 180.0 / pi

VAR

  long rawAcclX, rawAcclY, rawAcclZ
  long rawMagX, rawMagY, rawMagZ
  long rawGyroX, rawGyroY, rawGyroZ
  
  long q0, q1, q2, q3

  long r11, r12, r13
  long r21, r22, r23
  long r31, r32, r33
  
  long beta
  long magnitude
  long pitch, roll, yaw

  long sinPitch, cosPitch
  long sinRoll, cosRoll
  long sinYaw, cosYaw

  long gyroSumX, gyroSumY, gyroSumZ
  long offsetX, offsetY, offsetZ

  long floatMagX, floatMagY, floatMagZ
  long smoothAccX, smoothAccY, smoothAccZ
  
  long xMag, yMag, zMag

  long i, recipNorm

  long twoKp, twoKi
  long integralFBx, integralFBy, integralFBz

  long q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3
  long hx, hy, bx, bz
  long halfvx, halfvy, halfvz, halfwx, halfwy, halfwz
  long halfex, halfey, halfez
  long qa, qb, qc

  long debug
  
OBJ

  IMU         : "MinIMUv2-pasm"                '
  PST         : "Parallax Serial Terminal"
  intmath     : "SL32_INTEngine_2"
    
  fm          : "FME"
  fs          : "FloatString"
    
PUB Init

  IMU.start(SCL,SDA)
  'fm.start
  PST.start(250000)
  PST.Clear

  debug := True

  Main

PUB SetupDisplay
  
  PST.home
  PST.str(string("OpenIMU - Propeller Implementation"))
  PST.newline

  printHeading(35, 1, string("----Main----"))
  printHeading(35, 16, string("Pitch: "))
  printHeading(35, 17, string("Roll: "))
  printHeading(35, 18, string("Yaw: "))
  
  printHeading(0, 0, string("----IMUInit----"))
  printHeading(0, 1, string("->Calibrating Iteration: "))
  printHeading(0, 2, string("rawMagX: "))
  printHeading(0, 3, string("rawMagY: "))
  printHeading(0, 4, string("rawMagZ: "))
  printHeading(0, 5, string("rawAcclX: "))
  printHeading(0, 6, string("rawAcclY: "))
  printHeading(0, 7, string("rawAcclZ: "))
  printHeading(0, 8, string("rawGyroX: "))
  printHeading(0, 9, string("rawGyroY: "))
  printHeading(0, 10, string("rawGyroZ: "))
  printHeading(0, 11, string("smoothAccX: "))
  printHeading(0, 12, string("smoothAccY: "))
  printHeading(0, 13, string("smoothAccZ: "))

  printHeading(0, 16, string("->Gyro Summing Iteration:"))
  printHeading(0, 17, string("smoothAccX: "))
  printHeading(0, 18, string("smoothAccY: "))
  printHeading(0, 19, string("smoothAccZ: "))
  printHeading(0, 20, string("gyroSumX: "))
  printHeading(0, 21, string("gyroSumY: "))
  printHeading(0, 22, string("gyroSumZ: "))

  printHeading(0, 25, string("offSetX: "))
  printHeading(0, 26, string("offSetY: "))
  printHeading(0, 27, string("offSetZ: "))

  printHeading(0, 36, string("floatMagX: "))
  printHeading(0, 37, string("floatMagY: "))
  printHeading(0, 38, string("floatMagZ: "))
  printHeading(0, 39, string("xMag: "))
  printHeading(0, 40, string("yMag: "))
  printHeading(0, 41, string("yaw: "))
   
PUB Main

  SetupDisplay
  
  IMUinit
  
  repeat
    rawMagX := fm.ffloat(IMU.getMx)
    rawMagY := fm.ffloat(IMU.getMy)
    rawMagZ := fm.ffloat(IMU.getMz)
   
    rawAcclX := fm.ffloat(IMU.getAx)
    rawAcclY := fm.ffloat(IMU.getAy)
    rawAcclZ := fm.ffloat(IMU.getAz)
   
    rawGyroX := fm.ffloat(IMU.getRx)
    rawGyroY := fm.ffloat(IMU.getRy)
    rawGyroZ := fm.ffloat(IMU.getRz)

    floatMagX := fm.fsub(fm.fmul(fm.fsub(rawMagX, COMPASS_XMIN), INVERSE_XRANGE), 1.0)
    floatMagY := fm.fsub(fm.fmul(fm.fsub(rawMagY, COMPASS_YMIN), INVERSE_YRANGE), 1.0)
    floatMagZ := fm.fsub(fm.fmul(fm.fsub(rawMagZ, COMPASS_ZMIN), INVERSE_ZRANGE), 1.0)
    
    smoothAccX := smoothing(rawAcclX, smoothAccX)
    smoothAccY := smoothing(rawAcclY, smoothAccY)
    smoothAccZ := smoothing(rawAcclZ, smoothAccZ)

    if debug == True
      printData(25, 1, i)
      printDataFP(9, 2, rawMagX)
      printDataFP(9, 3, rawMagY)
      printDataFP(9, 4, rawMagZ)
      printDataFP(10, 5, rawAcclX)
      printDataFP(10, 6, rawAcclY)
      printDataFP(10, 7, rawAcclZ)
      printDataFP(10, 8, rawGyroX)
      printDataFP(10, 9, rawGyroY)
      printDataFP(10, 10, rawGyroZ)
      printDataFP(12, 11, smoothAccX)
      printDataFP(12, 12, smoothAccY)
      printDataFP(12, 13, smoothAccZ)
      
    AHRSupdate

    GetEuler

    UpdateDisplay

PRI UpdateDisplay | pitch2, roll2, yaw2

  pitch2 := ToDeg(fm.atan2(rawAcclX, fm.fsqr(fm.fadd(fm.fmul(rawAcclY, rawAcclY), fm.fmul(rawAcclZ, rawAcclZ)))))
  roll2 := ToDeg(fm.atan2(fm.fmul(fm.FNeg(1.0), rawAcclY), fm.fsqr(fm.fadd(fm.fmul(rawAcclX, rawAcclX), fm.fmul(rawAcclZ, rawAcclZ)))))
  yaw2 := ToDeg(fm.atan2(fm.fmul(2.0, fm.fadd(fm.fmul(q0, q3), fm.fmul(q1, q2))), fm.fsub(1.0, fm.fmul(2.0, fm.fadd(fm.fmul(q2, q2), fm.fmul(q3, q3))))))

  if debug == True
    printDataFP(42, 16, pitch)
    printDataFP(41, 17, roll)
    printDataFP(41, 18, yaw)
    printDataFP(0, 48, q0)
    printDataFP(15, 48, q1)
    printDataFP(30, 48, q2)
    printDataFP(45, 48, q3)
    printDataFP(7, 30, pitch2)
    printDataFP(3, 31, roll2)
    printDataFP(3, 32, yaw2)
    
    
PRI IMUinit

  repeat i from 0 to 100
    rawMagX := fm.ffloat(IMU.getMx)
    rawMagY := fm.ffloat(IMU.getMy)
    rawMagZ := fm.ffloat(IMU.getMz)
    
    rawAcclX := fm.ffloat(IMU.getAx)
    rawAcclY := fm.ffloat(IMU.getAy)
    rawAcclZ := fm.ffloat(IMU.getAz)
    
    rawGyroX := ToRad(fm.ffloat(IMU.getRx))
    rawGyroY := ToRad(fm.ffloat(IMU.getRy))
    rawGyroZ := ToRad(fm.ffloat(IMU.getRz))
    
    smoothAccX := smoothing(rawAcclX, smoothAccX)
    smoothAccY := smoothing(rawAcclY, smoothAccY)
    smoothAccZ := smoothing(rawAcclZ, smoothAccZ)

    if debug == True     
      printData(25, 1, i)
      printDataFP(9, 2, rawMagX)
      printDataFP(9, 3, rawMagY)
      printDataFP(9, 4, rawMagZ)
      printDataFP(10, 5, rawAcclX)
      printDataFP(10, 6, rawAcclY)
      printDataFP(10, 7, rawAcclZ)
      printDataFP(10, 8, rawGyroX)
      printDataFP(10, 9, rawGyroY)
      printDataFP(10, 10, rawGyroZ)
      printDataFP(12, 11, smoothAccX)
      printDataFP(12, 12, smoothAccY)
      printDataFP(12, 13, smoothAccZ)

    waitcnt(MILLI * 3 + cnt)
  
  if debug == True
    printHeading(25, 1, string("Done"))
    
  gyroSumX := gyroSumY := gyroSumZ := 0

  repeat i from 0 to 100
    smoothAccX := smoothing(rawAcclX, smoothAccX)
    smoothAccY := smoothing(rawAcclY, smoothAccY)
    smoothAccZ := smoothing(rawAcclZ, smoothAccZ)

    gyroSumX += IMU.getRx
    gyroSumY += IMU.getRy
    gyroSumZ += IMU.getRz

    if debug == True                 
      printData(24, 16, i)
      printDataFP(12, 17, smoothAccX)
      printDataFP(12, 18, smoothAccY)
      printDataFP(12, 19, smoothAccZ)
      printDataFP(10, 20, gyroSumX)
      printDataFP(10, 21, gyroSumY)
      printDataFP(10, 22, gyroSumZ)
      
    waitcnt(MILLI * 3 + cnt)

  if debug == True
    printHeading(24, 16, string("Done"))
    printHeading(0, 24, string("Finding gyro offset: "))
    
  offSetX := fm.fdiv(fm.ffloat(gyroSumX), 500.0)
  offSetY := fm.fdiv(fm.ffloat(gyroSumY), 500.0)
  offSetZ := fm.fdiv(fm.ffloat(gyroSumZ), 500.0)

  if debug == True
    printDataFP(9, 25, offSetX)
    printDataFP(9, 26, offSetY)
    printDataFP(9, 27, offSetZ)
    printHeading(21, 24, string("Done"))
    printHeading(0, 29, string("Finding pitch and roll: "))
  
  pitch := ToDeg(atan2(rawAcclX, fm.fsqr(fm.fadd(fm.fmul(rawAcclY, rawAcclY), fm.fmul(rawAcclZ, rawAcclZ)))))
  roll := ToDeg(atan2(fm.fmul(fm.FNeg(1.0), rawAcclY), fm.fsqr(fm.fadd(fm.fmul(rawAcclX, rawAcclX), fm.fmul(rawAcclZ, rawAcclZ)))))

  if debug == True
    printHeading(0, 30, string("pitch: "))
    printHeading(0, 31, string("roll"))
    printDataFP(7, 30, pitch)
    printDataFP(6, 31, roll)
    printHeading(29, 29, string("correcting pitch and roll"))
  
  if rawAcclZ > 0
    if rawAcclX > 0
      pitch := fm.fsub(180.0, pitch)
    else
      pitch := fm.fsub(-180.0, pitch)

    if rawAcclY >0
      roll := fm.fsub(-180.0, roll)
    else
      roll := fm.fsub(180.0, roll)

  if debug == True
    printDataFP(7, 30, pitch)
    printDataFP(6, 31, roll)
    printHeading(29, 29, string("Done"))
    printHeading(0, 34, string("Finding Yaw"))
    printHeading(0, 35, string("finding floatMag"))
    
  floatMagX := fm.fsub(fm.fmul(fm.fsub(rawMagX, COMPASS_XMIN), INVERSE_XRANGE), 1.0)
  floatMagY := fm.fsub(fm.fmul(fm.fsub(rawMagY, COMPASS_YMIN), INVERSE_YRANGE), 1.0)
  floatMagZ := fm.fsub(fm.fmul(fm.fsub(rawMagZ, COMPASS_ZMIN), INVERSE_ZRANGE), 1.0)
                                  
  if debug == True
    printDataFP(11, 35, floatMagX)
    printDataFP(11, 36, floatMagY)
    printDataFP(11, 37, floatMagZ)
    printHeading(13, 34, string("tilt compensating compass"))
  
  xMag := fm.fadd(fm.fmul(floatMagX, fm.cos(ToRad(pitch))), fm.fmul(floatMagZ, fm.sin(ToRad(pitch))))
  yMag := fm.fmul(fm.FNeg(1.0), fm.fsub(fm.fadd(fm.fmul(fm.fmul(floatMagX, fm.sin(ToRad(roll))), fm.sin(ToRad(pitch))), fm.fmul(floatMagY, fm.cos(ToRad(roll)))), fm.fmul(fm.fmul(floatMagZ, fm.sin(ToRad(roll))), fm.cos(ToRad(pitch)))))

  if debug == True
    printDataFP(6, 38, xMag)
    printDataFP(6, 39, yMag)
    
    printHeading(13, 34, string("finding yaw"))
  
  yaw := ToDeg(atan2(yMag, xMag))
  if yaw < 0.0
    yaw := fm.fadd(yaw, 360.0)

  if debug == True
    printDataFP(5, 40, yaw)
    printHeading(13, 34, string("Done"))
    printHeading(0, 42, string("Finding rotation matrix :"))
    
  cosPitch := fm.cos(ToRad(pitch))
  sinPitch := fm.sin(ToRad(pitch))                   
  cosRoll := fm.cos(ToRad(roll))
  sinRoll := fm.sin(ToRad(roll))                     
  cosYaw := fm.cos(ToRad(yaw))
  sinYaw := fm.sin(ToRad(yaw))

  if debug == True
    printDataLabelFP(30, 35, string("cosPitch: "), cosPitch)
    printDataLabelFP(30, 36, string("sinPitch: "), sinPitch)
    printDataLabelFP(30, 37, string("cosRoll: "), cosRoll)
    printDataLabelFP(30, 38, string("sinRoll: "), sinRoll)
    printDataLabelFP(30, 39, string("cosYaw: "), cosYaw)
    printDataLabelFP(30, 40, string("sinYaw: "), sinYaw)

  r11 := fm.fmul(cosPitch, cosYaw)
  r21 := fm.fmul(cosPitch, sinYaw)
  r31 := fm.fmul(fm.FNeg(1.0), sinPitch)
  r12 := fm.fadd(fm.fmul(fm.FNeg(1.0), fm.fmul(cosRoll, sinYaw)), fm.fmul(fm.fmul(sinRoll, sinPitch), cosYaw))
  r22 := fm.fadd(fm.fmul(cosRoll, cosYaw), fm.fmul(fm.fmul(sinRoll, sinPitch), sinYaw))
  r32 := fm.fmul(sinRoll, cosPitch)
  r13 := fm.fadd(fm.fmul(sinRoll, sinYaw), fm.fmul(fm.fmul(cosRoll, sinPitch), cosYaw))
  r23 := fm.fadd(fm.fmul(fm.FNeg(1.0), fm.fmul(sinRoll, cosYaw)), fm.fmul(fm.fmul(cosRoll, sinPitch), sinYaw))
  r33 := fm.fmul(cosRoll, cosPitch)

  if debug == True
    printDataFP(0, 43, r11)
    printDataFP(15, 43, r12)
    printDataFP(30, 43, r13)
    printDataFP(0, 44, r21)
    printDataFP(15, 44, r22)
    printDataFP(30, 44, r23)
    printDataFP(0, 45, r31)
    printDataFP(15, 45, r32)
    printDataFP(30, 45, r33)
  
  q0 := fm.fmul(0.5, fm.fsqr(fm.fadd(fm.fadd(1.0, r11), fm.fadd(r22, r33))))
  q1 := fm.fdiv(fm.fsub(r32, r23), fm.fmul(4.0, q0))
  q2 := fm.fdiv(fm.fsub(r13, r31), fm.fmul(4.0, q0)) 
  q3 := fm.fdiv(fm.fsub(r21, r12), fm.fmul(4.0, q0))

  if debug == True                                        
    printHeading(0, 47, string("Quaternions"))
    printDataFP(0, 48, q0)
    printDataFP(15, 48, q1)
    printDataFP(30, 48, q2)
    printDataFP(45, 48, q3)                 

  repeat until PST.charIn == "D"

PRI printDataLabelFP(row, column, label, data)

  PST.position(row, column)
  PST.str(label)
  PST.str(fs.floattostring(data))
  PST.clearEnd

PRI printHeading(row, column, data)

  PST.position(row, column)
  PST.str(data)
  PST.clearEnd

PRI printHeadingNL(data)

  PST.str(data)
  PST.newLine
  
PRI printLabel(row, column, data)

  PST.position(row, column)
  PST.str(data)

PRI printData(row, column, data)

  PST.position(row, column)
  PST.dec(data)
   
PRI printDataFP(row, column, data)

  PST.position(row, column)
  PST.str(fs.floattostring(data))
  PST.clearEnd  
  
PRI AHRSupdate | ax, ay, az, gx, gy, gz, mx, my, mz

ifnot (ax == 0.0) and (ay == 0.0) and (az == 0.0)
  ' Normalise acclerometer measurements
  ax := fm.fmul(fm.FNeg(1.0), rawAcclX)
  ay := fm.fmul(fm.FNeg(1.0), rawAcclY)
  az := fm.fmul(fm.FNeg(1.0), rawAcclZ)

  printDataFP(50, 3, ax)
  printDataFP(50, 4, ay)
  printDataFP(50, 5, az)

  gx := ToRad(fm.fmul(fm.fsub(rawGyroX, offSetX), GYRO_SCALE))
  gy := ToRad(fm.fmul(fm.fsub(rawGyroY, offSetY), GYRO_SCALE))
  gz := ToRad(fm.fmul(fm.fsub(rawGyroZ, offSetZ), GYRO_SCALE))

  printDataFP(50, 7, gx)
  printDataFP(50, 8, gy)
  printDataFP(50, 9, gz)

  mx := floatMagX
  my := floatMagY
  mz := floatMagZ

  printDataFP(50, 11, mx)
  printDataFP(50, 12, my)
  printDataFP(50, 13, mz)

  recipNorm := invSqrt(fm.fadd(fm.fadd(fm.fmul(ax, ax), fm.fmul(ay, ay)), fm.fmul(az, az)))
  ax := fm.fmul(recipNorm, ax)
  ay := fm.fmul(recipNorm, ay)
  az := fm.fmul(recipNorm, az)

  printDataFP(50, 3, ax)
  printDataFP(50, 4, ay)
  printDataFP(50, 5, az)
 
  ' Normalise magentometer measurements
  recipNorm := invSqrt(fm.fadd(fm.fadd(fm.fmul(mx, mx), fm.fmul(my, my)), fm.fmul(mz, mz)))
  mx := fm.fmul(recipNorm, mx) 
  my := fm.fmul(recipNorm, my)
  mz := fm.fmul(recipNorm, mz)

  printDataFP(50, 11, mx)
  printDataFP(50, 12, my)
  printDataFP(50, 13, mz)
   
  q0q0 := fm.fmul(q0, q0) 
  q0q1 := fm.fmul(q0, q1) 
  q0q2 := fm.fmul(q0, q2) 
  q0q3 := fm.fmul(q0, q3) 
  q1q1 := fm.fmul(q1, q1) 
  q1q2 := fm.fmul(q1, q2) 
  q1q3 := fm.fmul(q1, q3) 
  q2q2 := fm.fmul(q2, q2) 
  q3q3 := fm.fmul(q3, q3)                 

  printDataFP(50, 23, q0q0)
  printDataFP(50, 24, q0q1)
  printDataFP(50, 25, q0q2)
  printDataFP(50, 26, q0q3)
  printDataFP(50, 27, q1q1)
  printDataFP(50, 28, q1q2)
  printDataFP(50, 29, q1q3)
  printDataFP(50, 30, q2q2)
  printDataFP(50, 31, q3q3) 
  
  ' Reference direction of Earth's magnetic field
 
  hx := fm.fmul(2.0, fm.fadd(fm.fadd(fm.fmul(mx, fm.fsub(fm.fsub(0.5, q2q2), q3q3)), fm.fmul(my, fm.fsub(q1q2, q0q3))), fm.fmul(mz, fm.fadd(q1q3, q0q2))))
  hy := fm.fmul(2.0, fm.fadd(fm.fadd(fm.fmul(mx, fm.fadd(q1q2, q0q3)), fm.fmul(my, fm.fsub(fm.fsub(0.5, q1q1), q3q3))), fm.fmul(mz, fm.fsub(q2q3, q0q1)))) 
  bx := fm.fsqr(fm.fadd(fm.fmul(hx, hx), fm.fmul(hy, hy)))
  bz := fm.fmul(2.0, fm.fadd(fm.fadd(fm.fmul(mx, fm.fsub(q1q3, q0q2)), fm.fmul(my, fm.fadd(q2q3, q0q1))), fm.fmul(mz, fm.fsub(fm.fsub(0.5, q1q1), q2q2))))
   
  ' Estimated direction of gravity and magnetic field
  halfvx := fm.fsub(q1q3, q0q2)
  halfvy := fm.fadd(q0q1, q2q3)
  halfvz := fm.fadd(fm.fsub(q0q0, 0.5), q3q3)
  halfwx := fm.fadd(fm.fmul(bx, fm.fsub(fm.fsub(0.5, q2q2), q3q3)), fm.fmul(bz, fm.fsub(q1q3, q0q2)))
  halfwy := fm.fadd(fm.fmul(bx, fm.fsub(q1q2, q0q3)), fm.fmul(bz, fm.fadd(q0q1, q2q3)))
  halfwz := fm.fadd(fm.fmul(bx, fm.fadd(q0q2, q1q3)), fm.fmul(bz, fm.fsub(fm.fsub(0.5, q1q1), q2q2)))
   
  ' Error is sum of cross product between estimated direction and measured direction of field vectors
  halfex := fm.fadd(fm.fsub(fm.fmul(ay, halfvz), fm.fmul(az, halfvy)), fm.fsub(fm.fmul(my, halfwz), fm.fmul(mz, halfwy)))
  halfey := fm.fadd(fm.fsub(fm.fmul(az, halfvx), fm.fmul(ax, halfvz)), fm.fsub(fm.fmul(mz, halfwx), fm.fmul(mx, halfwz))) 
  halfez := fm.fadd(fm.fsub(fm.fmul(ax, halfvy), fm.fmul(ay, halfvx)), fm.fsub(fm.fmul(mx, halfwy), fm.fmul(my, halfwx)))
   
  ' Compute and apply integral feedback if enabled
  if twoKi > 0.0
    integralFBx := fm.fadd(integralFBx, fm.fmul(twoKi, fm.fmul(halfex, fm.fdiv(1.0, SAMPLE_FREQ))))
    integralFBy := fm.fadd(integralFBx, fm.fmul(twoKi, fm.fmul(halfey, fm.fdiv(1.0, SAMPLE_FREQ))))
    integralFBz := fm.fadd(integralFBx, fm.fmul(twoKi, fm.fmul(halfez, fm.fdiv(1.0, SAMPLE_FREQ))))
   
  else
    integralFBx := 0.0  
    integralFBy := 0.0
    integralFBz := 0.0
   
  gx := fm.fadd(gx, fm.fmul(twoKp, halfex))
  gy := fm.fadd(gy, fm.fmul(twoKp, halfey))
  gz := fm.fadd(gz, fm.fmul(twoKp, halfez))
  
  gx := fm.fmul(gx, fm.fmul(0.5, fm.fdiv(1.0, SAMPLE_FREQ)))
  gy := fm.fmul(gx, fm.fmul(0.5, fm.fdiv(1.0, SAMPLE_FREQ)))
  gz := fm.fmul(gx, fm.fmul(0.5, fm.fdiv(1.0, SAMPLE_FREQ)))
  qa := q0
  qb := q1
  qc := q2
  q0 := fm.fadd(q0, fm.fsub(fm.fsub(fm.fmul(fm.fneg(qb), gz), fm.fmul(qc, gy)), fm.fmul(q3, gz)))
  q1 := fm.fadd(q1, fm.fsub(fm.fadd(fm.fmul(qa, gx), fm.fmul(qc, gz)), fm.fmul(q3, gy)))
  q2 := fm.fadd(q2, fm.fadd(fm.fsub(fm.fmul(qa, gy), fm.fmul(qb, gz)), fm.fmul(q3, gx)))
  q3 := fm.fadd(q3, fm.fsub(fm.fadd(fm.fmul(qa, gz), fm.fmul(qb, gy)), fm.fmul(qc, gx)))
  
  ' Normalise quaternion
  recipNorm := invSqrt(fm.fadd(fm.fadd(fm.fadd(fm.fmul(q0, q0), fm.fmul(q1, q1)), fm.fmul(q2, q2)), fm.fmul(q3, q3)))
  q0 := fm.fmul(q0, recipNorm)
  q1 := fm.fmul(q1, recipNorm)
  q2 := fm.fmul(q2, recipNorm)
  q3 := fm.fmul(q3, recipNorm)
  
PRI invSqrt(x) 

  return(fm.pow(x, fm.FNeg(0.5)))

PRI GetEuler

  roll := ToDeg(fm.atan2(fm.fmul(2.0, fm.fadd(fm.fmul(q0, q1), fm.fmul(q2, q3))), fm.fsub(1.0, fm.fmul(2.0, fm.fadd(fm.fmul(q1, q1), fm.fmul(q2, q2))))))
  pitch := ToDeg(fm.asin(fm.fmul(2.0, fm.fsub(fm.fmul(q0, q2), fm.fmul(q3, q1)))))
  yaw := ToDeg(fm.atan2(fm.fmul(2.0, fm.fadd(fm.fmul(q0, q3), fm.fmul(q1, q2))), fm.fsub(1.0, fm.fmul(2.0, fm.fadd(fm.fmul(q2, q2), fm.fmul(q3, q3))))))

  if yaw < 0.0
    yaw := fm.fadd(yaw, 360.0)
    
PRI Smoothing(rawAccl, smoothedAccl)

  return(fm.fadd(fm.fmul(rawAccl, 0.15), fm.fmul(smoothedAccl, 0.85)))

PRI atan2(y, x) | z, atan

  if x == 0.0
    if y > 0.0
      return(PIBY2_FLOAT)
    if y == 0.0
      return(0.0)
    else
      return(fm.fneg(PIBY2_FLOAT))

  z := fm.fdiv(y, x)
  if fm.fabs(z) < 1.0
    atan := fm.fdiv(z, fm.fadd(1.0,fm.fmul(fm.fmul(0.28, z), z)))

    if x < 0.0
      if y < 0.0
        return(fm.fsub(atan, PI_FLOAT))
      else
        return(fm.fadd(atan, PI_FLOAT))

  else
    atan := fm.fsub(PIBY2_FLOAT, fm.fdiv(z , fm.fadd(fm.fmul(z, z), 0.28)))
    if y < 0.0
      return(fm.fsub(atan, PI_FLOAT))

  return(atan)

PRI toDeg(angle_in_rads)

  return fm.FMul(angle_in_rads, TO_DEG) 'fm.FDiv(180.0, PI))

PRI toRad(angle_in_degs)

  return fm.FMul(angle_in_degs, TO_RAD) 'fm.FDiv(PI, 180.0))

                                                 