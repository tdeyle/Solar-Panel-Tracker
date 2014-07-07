''Test Code for IMU

CON

   _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
  

OBJ

pst : "Parallax Serial Terminal"
imu : "MinIMUadj"

PUB Main  | index

  pst.start(115200)
  imu.start(17,16)

  pst.str(string("Testing..."))

  repeat until imu.isready
  
  repeat
    pst.home
    pst.newline
    pst.str(string("X: "))
    pst.dec(imu.getAx)
    pst.NewLine 

    pst.str(string("Y: "))
    pst.dec(imu.getAy)
    pst.NewLine

    pst.str(string("Z: "))
    pst.dec(imu.getAz)
    pst.NewLine
    
    pst.dec(index)
    
    index++
    
   