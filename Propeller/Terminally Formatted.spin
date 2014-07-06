{{{
-> Need to have an update display for iterations and values.
-> Add constants for other values.
}}

CON

  _clkmode      = xtal1 + pll16x
  _clkfreq      = 80_000_000


  rawAcclX = 0
  rawAcclY = 1
  rawAcclZ = 2

  rawGyroX = 3
  rawGyroY = 4
  rawGyroZ = 5
  
  rawMagX = 6
  rawMagY = 7
  rawMagZ = 8
    
  gyroSumX = 9
  gyroSumY = 10
  gyroSumZ = 11

  offsetX = 12
  offSetY = 13
  offSetZ = 14

  floatMagX = 15
  floatMagY = 16
  floatMagZ = 17

  smoothAccX = 18
  smoothAccY = 19
  smoothAccZ = 20
  
  q0 = 21
  q1 = 22
  q2 = 23
  q3 = 24

  pitch = 25
  roll  = 26
  yaw   = 27

  fRawAcclX = 28
  fRawAcclY = 29
  fRawAcclZ = 30
  
  fRawMagX  = 31
  fRawMagY  = 32
  fRawMagZ  = 33


VAR

  long master_add

OBJ

  PST : "Parallax Serial Terminal"
  fs  : "FloatString"

PUB Start(baudrate, var_address)

  waitcnt(clkfreq + cnt)
  master_add := var_address
  
  PST.Start(250000)  

PUB UpdateCalibratingIteration(index)
                         
  printData(25, 1, index)
  printData(12, 2, long[master_add][rawMagX])
  printData(12, 3, long[master_add][rawMagY])
  printData(12, 4, long[master_add][rawMagZ])

  printData(12, 5, long[master_add][rawAcclX])
  printData(12, 6, long[master_add][rawAcclY])
  printData(12, 7, long[master_add][rawAcclZ])

  printData(12, 8, long[master_add][rawGyroX])
  printData(12, 9, long[master_add][rawGyroY])
  printData(12, 10, long[master_add][rawGyroZ])

  printDataFP(12, 11, long[master_add][smoothAccX])
  printDataFP(12, 12, long[master_add][smoothAccY])
  printDataFP(12, 13, long[master_add][smoothAccZ])

  printDataFP(12, 15, long[master_add][gyroSumX])
  printDataFP(12, 16, long[master_add][gyroSumY])
  printDataFP(12, 17, long[master_add][gyroSumZ])


PUB updateGyroOffset

  printHeading(0, 19, string("Result:"))

  printHeading(0, 20, string("offSetX: "))
  printHeading(0, 21, string("offSetY: "))
  printHeading(0, 22, string("offSetZ: "))

  printDataFP(12, 20, long[master_add][offSetX])
  printDataFP(12, 21, long[master_add][offSetY])
  printDataFP(12, 22, long[master_add][offSetZ])

PUB SetupDisplay
  
  PST.home
  PST.str(string("OpenIMU - Propeller Implementation"))
  PST.newline
  
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

  printHeading(0, 15, string("gyroSumX: "))
  printHeading(0, 16, string("gyroSumY: "))
  printHeading(0, 17, string("gyroSumZ: "))
  
PUB updateInit

  printDataLabelFP(0, 24, string("fRawAcclX: "), long[master_add][fRawAcclX])
  printDataLabelFP(0, 25, string("fRawAcclY: "), long[master_add][fRawAcclY])
  printDataLabelFP(0, 26, string("fRawAcclZ: "), long[master_add][fRawAcclZ])

  printDataLabelFP(0, 28, string("fRawMagX: "), long[master_add][fRawMagX])
  printDataLabelFP(0, 29, string("fRawMagY: "), long[master_add][fRawMagY])
  printDataLabelFP(0, 30, string("fRawMagZ: "), long[master_add][fRawMagZ])

  printDataLabelFP(0, 32, string("Pitch: "), long[master_add][pitch])
  printDataLabelFP(0, 33, string("Roll: "), long[master_add][roll])
  printDataLabelFP(0, 34, string("Yaw: "), long[master_add][yaw])
  
  printDataLabelFP(0, 36, string("q0: "), long[master_add][q0])
  printDataLabelFP(0, 37, string("q1: "), long[master_add][q1])
  printDataLabelFP(0, 38, string("q2: "), long[master_add][q2])
  printDataLabelFP(0, 39, string("q3: "), long[master_add][q3])
                           
PUB setupMain

  printHeading(35, 0, string("----Main----"))
  printHeading(35, 15, string("Pitch: "))
  printHeading(35, 16, string("Roll: "))
  printHeading(35, 17, string("Yaw: "))

  printHeading(35, 2, string("rawMagX: "))
  printHeading(35, 3, string("rawMagY: "))
  printHeading(35, 4, string("rawMagZ: "))

  printHeading(35, 5, string("rawAcclX: "))
  printHeading(35, 6, string("rawAcclY: "))
  printHeading(35, 7, string("rawAcclZ: "))

  printHeading(35, 8, string("rawGyroX: "))
  printHeading(35, 9, string("rawGyroY: "))
  printHeading(35, 10, string("rawGyroZ: "))

  printHeading(35, 11, string("smoothAccX: "))
  printHeading(35, 12, string("smoothAccY: "))
  printHeading(35, 13, string("smoothAccZ: "))

  printHeading(35, 24, string("fRawAcclX: "))
  printHeading(35, 25, string("fRawAcclY: "))
  printHeading(35, 26, string("fRawAcclZ: "))

  printHeading(35, 28, string("fRawMagX: "))
  printHeading(35, 29, string("fRawMagY: "))
  printHeading(35, 30, string("fRawMagZ: "))

  printHeading(35, 18, string("q0: "))
  printHeading(35, 19, string("q1: "))
  printHeading(35, 20, string("q2: "))
  printHeading(35, 21, string("q3: "))
                                              
PUB updateDisplay

  printDataFP(47, 15, long[master_add][Pitch])
  printDataFP(47, 16, long[master_add][Roll])
  printDataFP(47, 17, long[master_add][Yaw])

  printData(47, 2, long[master_add][rawMagX])
  printData(47, 3, long[master_add][rawMagY])
  printData(47, 4, long[master_add][rawMagZ])

  printData(47, 5, long[master_add][rawAcclX])
  printData(47, 6, long[master_add][rawAcclY])
  printData(47, 7, long[master_add][rawAcclZ])

  printData(47, 8, long[master_add][rawGyroX])
  printData(47, 9, long[master_add][rawGyroY])
  printData(47, 10, long[master_add][rawGyroZ])

  printDataFP(47, 11, long[master_add][smoothAccX])
  printDataFP(47, 12, long[master_add][smoothAccY])
  printDataFP(47, 13, long[master_add][smoothAccZ])

  printDataFP(47, 24, long[master_add][fRawAcclX])
  printDataFP(47, 25, long[master_add][fRawAcclY])
  printDataFP(47, 26, long[master_add][fRawAcclZ])

  printDataFP(47, 28, long[master_add][fRawMagX])
  printDataFP(47, 29, long[master_add][fRawMagY])
  printDataFP(47, 30, long[master_add][fRawMagZ])

  printDataFP(47, 18, long[master_add][q0])
  printDataFP(47, 19, long[master_add][q1])
  printDataFP(47, 20, long[master_add][q2])
  printDataFP(47, 21, long[master_add][q3])   
   
PUB Clear

  PST.Clear

PUB printEuler2

  pst.str(fs.floattostring(long[master_add][pitch]))
  pst.str(string(" "))
  pst.str(fs.floattostring(long[master_add][roll]))
  pst.str(string(" "))
  pst.str(fs.floattostring(long[master_add][yaw]))
  pst.NewLine

PUB printEuler

  printDataLabelFP(0, 1, string("pitch: "), long[master_add][pitch])
  printDataLabelFP(0, 2, string("roll: "), long[master_add][roll])
  printDataLabelFP(0, 3, string("yaw: "), long[master_add][yaw])

PUB printDataLabelFP(row, column, label, data)

  fs.setPrecision(4)
  PST.position(row, column)
  PST.str(label)
  PST.str(fs.floattostring(data))
  PST.clearEnd

PUB printDataLabel(row, column, label, data)

  PST.position(row, column)
  PST.str(label)
  PST.dec(data)
  PST.clearEnd
  
PUB printHeading(row, column, data)

  PST.position(row, column)
  PST.str(data)
  PST.clearEnd

PUB printHeadingNL(data)

  PST.str(data)
  PST.newLine
  
PUB printLabel(row, column, data)

  PST.position(row, column)
  PST.str(data)

PUB printData(row, column, data)

  PST.position(row, column)
  PST.dec(data)
  PST.clearEnd
   
PUB printDataFP(row, column, data)

  PST.position(row, column)
  PST.str(fs.floattostring(data))
  PST.clearEnd
  