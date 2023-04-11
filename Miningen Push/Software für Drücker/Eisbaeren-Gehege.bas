' Miningen Push - Eisbär
' ----------------------
'
$regfile = "attiny44.dat"                                   'Chip festlegen
$crystal = 800000                                          'Taktfrequenz: 8 MHz
$hwstack = 8                                                ' Platz für nur 4 Unterprogramme reserviert, spart SRAM
$swstack = 0                                                ' wird nicht gebraucht
$framesize = 0                                              ' dito

DIM t as SINGLE
DIM Tz as SINGLE
DIM TLED as Single
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

Config PINb.1 = Input
Portb.1 = 1
Switch Alias PINB.1

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
   wait 5
   Status_rot = 1
      While Tz < 59999
         If Switch = 0 then
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
    If Switch = 0 then
       While Tz < t


          Select Case TLED
             Case 0 To 1000 :
               Status_gruen = 0
               Status_rot = 1
               Incr TLED
             Case 1001 To 2000 :
               Status_gruen = 1
               Status_rot = 0
               Incr TLED
             Case Is >= 2000 :
               TLED = 0
          End Select
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