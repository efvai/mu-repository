#ifndef VIRTUAL_MOTOR_H
#define VIRTUAL_MOTOR_H

typedef enum {
    VM_CONNECT,
    VM_DISCONNECT,
    VM_LOAD_INC,
    VM_LOAD_DEC
} VM_COMMAND;

void virtual_motor_reinit();
void virtual_motor_reset();
char* virtual_motor_parse_command(char *cmd);
void virtual_motor_commander(VM_COMMAND command);
void virtual_motor_run(float duty_cycle);

#endif
