
//============================ include library ========================================================================
#include "shell.h"



//============================ Program Constants ======================================================================

// Command table definition (defined only once in shell.cpp)
Command commands[] = {
  { "LED", cmd_led },
  { "MOTOR", cmd_motor },
  { "STATUS", cmd_status },
};

// Number of commands
int numCommands = sizeof(commands) / sizeof(commands[0]);

//=====================================================================================================================
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
bool processCommand(char* input){
  makeStringUpperCase(input);
  char* cmd = strtok(input, " ");  // Get command
  char* args = strtok(NULL, "");   // Get arguments
  bool bSuccess = true;

  if (cmd) {
    for (int i = 0; i < numCommands; i++) {
      if (strcmp(cmd, commands[i].name) == 0) {
        bSuccess = commands[i].func(args);
        if (!bSuccess) {
          print_error(COMMAND_FAILED, __func__, __LINE__, "HIGH", "Command: %s had faild it's execution", cmd);
          return 1;
        }
        return 0;
      }
    }
  }
  print_error(UNKNOWN_COMMAND, __func__, __LINE__, "WARNING", "Command: %s is unknow to the shell", cmd);
  return 0;
}

//=====================================================================================================================
/**
 * @brief makes a string all upper case characters
 *
 * @param str:  the string memory address
 */
void makeStringUpperCase(char* str) {
  if (str == NULL) return;  // safety!

  for (size_t i = 0; i < strlen(str); i++) str[i] = (char)toupper(str[i]);
}

//=====================================================================================================================
/**
 * @brief Handles the LED control command.
 *
 * This function turns the LED on or off based on the provided argument.
 *
 * @param args Pointer to the argument string (expected values: "ON" or "OFF").
 * @return `true` (1) if the command executes successfully, `false` (0) otherwise.
 */
bool cmd_led(char* args) {
  if (args && strcmp(args, "ON") == 0) {
    Serial.println("Turning LED ON");
    return 1;
  } else if (args && strcmp(args, "OFF") == 0) {
    Serial.println("Turning LED OFF");
    return 1;
  } else {
    Serial.println("ERROR: Usage LED ON/OFF");
    return 0;
  }
  return 1;
}

//=====================================================================================================================
/**
 * @brief Handles the motor speed command.
 *
 * This function takes an argument specifying the desired motor speed and 
 * prints it to the serial monitor.
 *
 * @param args Pointer to the argument string containing the motor speed value.
 * @return `true` (1) indicating the command was processed successfully.
 */
bool cmd_motor(char* args) {
  Serial.print("Setting motor speed: ");
  Serial.println(args);
  return 1;
}

//=====================================================================================================================
/**
 * @brief Retrieves and displays the system status.
 *
 * This function prints a message to the serial monitor indicating 
 * that the Arduino is running properly.
 *
 * @param args Unused parameter (can be NULL).
 * @return `true` (1) indicating the command was processed successfully.
 */
bool cmd_status(char* args) {
  Serial.println("Arduino running fine!");
  return 1;
}
