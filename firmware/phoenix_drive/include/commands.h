/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define POSITION    'a'//send position command
#define ANGLE       'j'//read joint angle
#define ENCODER     'e'//set encoder type magnetic or analog
#define DIAGNOSTICS 'd'// Diagnostic info       'j'
#define WRITE_INVERT    'i'// invert the out input joint angle

#define WRITE_PID     'K'
#define LOOK_PID      'l'
#define LOOK_ID       'm'

#define WRITE_ID      'n'
#define WRITE_MAX     'o'// max safe rotation
#define WRITE_MIN     'p'//min safe rotation
#define WRITE_OFF     'q'// differance between actual ant joint angle
#define WRITE_OUTLIMIT 'r'// motor driver out put limit
#define WRITE_START   's'// define nuturalposition
#define WRITE_ENC     't'// define nuturalposition
#define UPDATE_PID  'u'//set pid valuse for joint



#endif

