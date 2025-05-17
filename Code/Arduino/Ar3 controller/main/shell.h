#ifndef SHELL_H
#define SHELL_H

//============================ include library ========================================================================
#include <Arduino.h>
#include "global.h"

//============================ Program Constants ======================================================================
const int COMMAND_INDEX_NOT_FOUND = -1;  // used when command index not found
const int BLANK_LINE = -2;               // used to signal a blank line in the input file

struct Command {
  const char* name;
  bool (*func)(char* args);
};


//======================================== Structure Definitions ====================================
// structure to map command keyword string to a command index
typedef struct COMMAND {
  const int index;
  const char* strCommand;
} COMMAND;

//======================================== Global Variable Definitions ==============================

extern Command commands[];
extern int numCommands;

//======================================== Function Prototypes ======================================
/**
 * @brief Processes a command input string and executes the corresponding function.
 *
 * This function tokenizes the input string, extracts the command and its arguments, 
 * then searches for a matching command in the command table. If found, the corresponding 
 * function is executed.
 *
 * @param input Pointer to the input string containing the command and arguments.
 * @return `true` (1) if the command executes successfully, `false` (0) otherwise.
 */
bool processCommand(char* input);

//=====================================================================================================================
/**
 * @brief makes a string all upper case characters
 *
 * @param str:  the string memory address
 */
void makeStringUpperCase(char* str);

bool cmd_led(char* args);
bool cmd_motor(char* args);
bool cmd_status(char* args);
//=====================================================================================================================

#endif