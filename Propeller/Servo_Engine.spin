{{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PWM2C Servo Engine
//
// Author: Kwabena W. Agyeman
// Updated: 8/22/2010
// Designed For: P8X32A
// Version: 1.1
//
// Copyright (c) 2010 Kwabena W. Agyeman
// See end of file for terms of use.
//
// Update History:
//
// v1.0 - Original release - 4/8/2009.
// v1.1 - Added support for variable pin assignments and improved code - 8/22/2010.
//
// For each included copy of this object only one spin interpreter should access it at a time.
//
// Nyamekye,
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Servo Circuit:
//
//                 5V
//                 |
//                 --- Servo 5V Power
//
// Left Servo Pin  --- Servo Pulse Width Pin
//
//                 --- Servo Ground
//                 |
//                GND
//
//                 5V
//                 |
//                 --- Servo 5V Power
//
// Right Servo Pin --- Servo Pulse Width Pin
//
//                 --- Servo Ground
//                 |
//                GND
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}}

VAR

  long leftServoPulse, rightServoPulse, pinMasks, stack[6]
  byte cogNumber, leftPinMask, rightPinMask, frequencyNumber

PUB leftPulseLength(microseconds) '' 4 Stack Longs

'' ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'' // Sets the standard servo to go to the position assigned to the pulse length in microseconds.
'' //
'' // Microseconds - Number of microseconds to send the pulse for.
'' ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  leftServoPulse := ((microseconds <# 1_000_000) #> 0)

PUB rightPulseLength(microseconds) '' 4 Stack Longs

'' ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'' // Sets the standard servo to go to the position assigned to the pulse length in microseconds.
'' //
'' // Microseconds - Number of microseconds to send the pulse for.
'' ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  rightServoPulse := ((microseconds <# 1_000_000) #> 0)

PUB SEREngineStart(leftServoPin, rightServoPin, cycleFrequency) '' 9 Stack Longs

'' ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'' // Starts up the SER driver running on a cog.
'' //
'' // Returns true on success and false on failure.
'' //
'' // LeftServoPin - Pin for left channel servo pulse width output. Between (0 - 31). -1 to disable.
'' // RightServoPin - Pin for right channel servo pulse width output. Between (0 - 31). -1 to disable.
'' // CycleFrequency - The PWM frequency to drive the servos at. Between 1 Hz and 60 Hz. (Try 50 Hz).
'' ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  SEREngineStop
  if(chipver == 1)

    leftPinMask := ((leftServoPin <# 31) #> 0)
    rightPinMask := ((rightServoPin <# 31) #> 0)
    pinMasks := (((|<leftPinMask) & (leftServoPin <> -1)) | ((|<rightPinMask) & (rightServoPin <> -1)))
    frequencyNumber := ((cycleFrequency <# 60) #> 1)

    cogNumber := cognew(SERDriver, @stack)
    result or= ++cogNumber

PUB SEREngineStop '' 3 Stack Longs

'' ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'' // Shuts down the SER driver running on a cog.
'' ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if(cogNumber)
    cogstop(-1 + cogNumber~)

PRI SERDriver ' 6 Stack Longs

  dira := pinMasks
  ctra := (constant(%0_0100 << 26) + leftPinMask)
  ctrb := (constant(%0_0100 << 26) + rightPinMask)
  frqa := 1
  frqb := 1

  pinMasks := (clkfreq / 1_000_000)

  result := cnt
  repeat
    result += (clkfreq / (~frequencyNumber))
    waitcnt(result)
    phsa := -(leftServoPulse * pinMasks)
    phsb := -(rightServoPulse * pinMasks)

{{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  TERMS OF USE: MIT License
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}}