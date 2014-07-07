CON

  _clkmode      = xtal1 + pll16x
  _clkfreq      = 80_000_000

  AZI_PIN = 8
  ELE_PIN = 9
  RX_PIN = 24
  TX_PIN = 23
  
OBJ

  serial : "Parallax Serial Terminal"
  PST : "Parallax Serial Terminal"
  ser: "Servo_Engine"
  
VAR

  byte data[20]
  byte roll[10], pitch[10], yaw[10]

  long full_roll, full_pitch, full_yaw

  long iRoll, iPitch, iYaw

  long IMU_Stack[100]

  long timeCounter
  
PUB Main | ptr, idx, frequencyCounter

  ser.SEREngineStart(AZI_PIN, ELE_PIN, 50)

  ser.leftPulseLength(2_000 - frequencyCounter)
  
  cognew(Do_IMU, @IMU_Stack)

  waitcnt(clkfreq * 10 + cnt)

  timeCounter := ((clkfreq / 1_000) * 20)

  repeat

    repeat frequencyCounter from 0 to 1_000 step 10

      ser.leftPulseLength(1_000 + frequencyCounter)

      ser.rightPulseLength(2_000 - frequencyCounter)

      waitcnt(timeCounter + cnt)

    repeat frequencyCounter from 1_000 to 0 step 10

      ser.leftPulseLength(1_000 + frequencyCounter)

      ser.rightPulseLength(2_000 - frequencyCounter)

      waitcnt(timeCounter + cnt)
  
PUB Do_IMU

  PST.start(115200)
  serial.startrxtx(RX_PIN,TX_PIN,0, 57600) '24 is RX, 23 is TX Arduino => 15 is rx, 14 is tx

  waitcnt(clkfreq+cnt)

  repeat
    if serial.charIn == "!"
      full_pitch := 0
      full_roll := 0
      full_yaw := 0
      
      pitch[0] := serial.charIn
      pitch[1] := serial.charIn
      pitch[2] := serial.charIn
      roll[0] := serial.charIn
      roll[1] := serial.charIn
      roll[2] := serial.charIn
      yaw[0] := serial.charIn
      yaw[1] := serial.charIn
      yaw[2] := serial.charIn

      full_pitch := pitch[1] | (pitch[2] << 8)
      if pitch[0] == 1
        full_pitch := 1 + (-full_pitch ^ %1111_1111_1111_1111)
        if full_pitch == $1_0000
          full_pitch := 0

      full_roll := roll[1] | (roll[2] << 8)
      if roll[0] == 1
        full_roll := 1 + (-full_roll ^ %1111_1111_1111_1111)
        if full_roll == $1_0000
          full_roll := 0

      full_yaw := yaw[1] | (yaw[2] << 8)
      if yaw[0] == 1
        full_yaw := 1 + (-full_yaw ^ %1111_1111_1111_1111)
        if full_yaw == $1_0000
          full_yaw := 0


      PST.dec(full_pitch)
      PST.str(string(", "))
      PST.dec(full_roll)
      PST.str(string(", "))
      PST.dec(full_yaw)
      PST.newLine                  
                                                        