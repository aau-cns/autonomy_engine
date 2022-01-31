Header header

byte type           # type of log message
string state        # current state of the autonomy
string msg          # message to log
string next_state   # state to change to next depending on action/message

## TYPES OF LOG
byte INFO=1
byte USER_INPUT=2
byte MESSAGE_RECEIVED=4
byte MESSAGE_SENT=8
byte STATE_CHANGE=16
byte SERVICE_CALLED=32
byte SERVICE_ANSWERED=64
