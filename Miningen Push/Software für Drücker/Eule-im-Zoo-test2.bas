
' Hardware PWM mit Timer1

$regfile = "attiny44.dat"                                   'Chip festlegen
$crystal = 800000                                          'Taktfrequenz: 8 MHz
$hwstack = 8                                                ' Platz für nur 4 Unterprogramme reserviert, spart SRAM
$swstack = 0                                                ' wird nicht gebraucht
$framesize = 0






DDRB = &B00010011
PORTB = &B11101100   'für nicht invertierte PWM
' PORTB = &B11111111   'für invertierte PWM
TCCR0A = &B10100001  'OC0A und OC0B nicht invertierte PWM
' TCCR0A = &B11110001  'OC0A und OC0B invertierte PWM
TCCR0B = &B00000010   'PWM-Freq = ca. 2kHz
' TCCR0B = &B00000011   'PWM-Freq = ca. 240 Hz
TCNT0 = 0
OCR0A = 0
OCR0B = 0
'PLLCSR = 0
'TCCR1 = &B00000101   'PWM-Freq = ca. 2kHz
' TCCR1 = &B00001000   'PWM-Freq = ca. 240 Hz
GTCCR = &B01100000   'OC1B nicht invertierte PWM
' GTCCR = &B01110000   'OC1B invertierte PWM
TCNT1 = 0
OCR1A = 0   'wird hier nur auf 0 gesetzt; wird sonst im Prog nicht benötigt
OCR1B = 0
OCR1C = 255   'der TOP-Wert für Timer1 im PWM-Modus
...
DO
OCR0A = 100
OCR0B = 200
OCR1B = 250
LOOP