' Miningen Push - Eisbär
' ----------------------
'
' Sollte der Speicher des ATtiny13 nicht reichen, so kann der baugleiche ATtiny45 verwendet werden, der hat mehr Speicher
'
'
$regfile = "attiny44.dat"                                   'Chip festlegen
$crystal = 800000                                           'Taktfrequenz: 8 MHz
$hwstack = 8                                                ' Platz für nur 4 Unterprogramme reserviert, spart SRAM
$swstack = 0                                                ' wird nicht gebraucht
$framesize = 0                                              ' dito

DIM t as SINGLE
DIM Tz as SINGLE
DIM x1 as BIT                                               'Variable für 1 Schleife (Abbruch)
x1 = 0

''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Laenge der Dauer in MILLISEKUNDEN (ohne Einheit) eingeben.
' 1 Sekunde = 1000ms
' 1 Minute = 60000ms
'
t = 10000 'Millisekunden (:1.000 = Sekunden)
'
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

'I/Os bestimmen
Relais Alias PortB.0
Config Relais = Output

Switch Alias PINB.1
Config Switch = Input

Status_rot Alias PortA.1
Config Status_rot = Output

Status_gruen Alias PortA.3
Config Status_gruen = Output

'Zeitberechnung
't = t * 1000
Tz = 0

Do
   'Abschaltung des Systems durch Knopfdruck in der ersten Minute

   If x1=0 then
   Status_gruen = 1
   wait 2
   Status_rot = 1
      While Tz < 59999
         If Switch = 1 then
            Status_gruen = 0
            Do
               'Endlosschleife
            Loop
         End if
         waitms 1
         Incr Tz                                                 'increase by one
      Wend
      x1 = 1
    End If


    'Zeitprogramm - Start mit Knopfdruck
    Tz = 0
    Status_gruen = 1
    Status_rot = 0
    If Switch = 1 then
       While Tz < t
         Status_gruen = 0
         Status_rot = 1
         Relais = 1
         waitms 1
         Incr Tz                                                 'increase by one
       Wend

       'RESET
       Tz = 0
       While Tz < 5000
          Relais = 0
          Status_rot = 1
          Status_gruen = 1
          Incr Tz
          waitms 1
       Wend
    End if
Loop








'4/12: Weicher LED-Blinker Ttiny13_Soft.bas

'LED soft flasher
'$regfile = "attiny13.dat"
'$crystal = 1200000
'$hwstack = 8
'$swstack = 4
'$framesize = 4
'Dim I As Byte
'Dim D As Integer

'Config Portb = Output
'Config Timer0 = Pwm , Prescale = 1 , Compare A Pwm = Clear Down

'Do
'   For I = 40 To 215
'     If I < 128 Then
'       D = I
'       D = D * D
'     End If
'     If I > 127 Then
'       D = 255 - I
'       D = D * D
'     End If
'     D = D / 64
'     Pwm0a = D
'     Waitms 60
'   Next I
'   Waitms 800
'Loop
'End