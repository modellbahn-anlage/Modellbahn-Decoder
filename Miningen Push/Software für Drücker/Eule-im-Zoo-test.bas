' Hardware PWM mit Timer1

$regfile = "attiny44.dat"                                   'Chip festlegen
$crystal = 800000                                          'Taktfrequenz: 8 MHz
$hwstack = 8                                                ' Platz für nur 4 Unterprogramme reserviert, spart SRAM
$swstack = 0                                                ' wird nicht gebraucht
$framesize = 0




Config PortA.6 = Output



Config Timer1 = Pwm , Pwm = 8 , Prescale = 8 , Compare A Pwm = Clear Up, Compare B Pwm = Clear Up

Dim A As Byte


Do


For A = 1 To 150 Step 1
  Compare1a = A
  Waitms 50
Next A




For A = 150 To 0 Step -1
  Compare1a = A
  Waitms 50
Next A

wait 20


Loop

End