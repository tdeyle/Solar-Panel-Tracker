CON

  _clkmode      = xtal1 + pll16x
  _clkfreq      = 80_000_000

  ROLL_PIN = 2
  PITCH_PIN = 3
  YAW_PIN = 4
  RX_PIN = 24
  TX_PIN = 23
  'CLK_PIN = 2
  'DAT_PIN = 3
  'LATCH = 4
  
OBJ

  serial : "Parallax Serial Terminal"
  PST : "Parallax Serial Terminal"
  
VAR

  byte data[20]
  byte roll[10], pitch[10], yaw[10]

  long full_roll, full_pitch, full_yaw

  long iRoll, iPitch, iYaw
  
PUB Main | ptr, idx

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
                                                        