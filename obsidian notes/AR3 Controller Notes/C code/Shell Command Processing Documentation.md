## Overview

The `Shell` system is designed to process command-line input on an Arduino-compatible microcontroller. It tokenizes input strings, identifies valid commands, and executes corresponding functions, allowing for structured and extendable command processing.

## Constants
- **COMMAND_INDEX_NOT_FOUND** (`-1`): Indicates that a command index was not found.
- **BLANK_LINE** (`-2`): Signals a blank line in the input file.
## Structure Definitions
The system defines structures for command management:
### `Command`

```c
struct Command {
  const char* name;
  bool (*func)(char* args);
};
```

#### Structure Fields:
- **name**: The command name as a string.
- **func**: A function pointer to the command’s execution function.
### `COMMAND`

```c
typedef struct COMMAND {
  const int index;
  const char* strCommand;
} COMMAND;
```

#### Structure Fields:
- **index**: A unique integer identifier for the command.
- **strCommand**: The command keyword as a string.

## Global Variables

```c
Command commands[] = {
  { "LED", cmd_led },
  { "MOTOR", cmd_motor },
  { "STATUS", cmd_status },
};

int numCommands = sizeof(commands) / sizeof(commands[0]);
```
- **commands[]**: Array storing available commands.
- **numCommands**: Number of registered commands.

## Function Implementations

### `processCommand`

```c
bool processCommand(char* input) {
  makeStringUpperCase(input);
  char* cmd = strtok(input, " ");  // Get command
  char* args = strtok(NULL, "");   // Get arguments
  bool bSuccess = true;

  if (cmd) {
    for (int i = 0; i < numCommands; i++) {
      if (strcmp(cmd, commands[i].name) == 0) {
        bSuccess = commands[i].func(args);
        if (!bSuccess) {
          print_error(COMMAND_FAILED, __func__, __LINE__, "HIGH", "Command: %s had failed its execution", cmd);
          return 1;
        }
        return 0;
      }
    }
  }
  print_error(UNKNOWN_COMMAND, __func__, __LINE__, "WARNING", "Command: %s is unknown to the shell", cmd);
  return 0;
}
```
#### Description:
Processes an input string by extracting the command and arguments, searching for a match in the command table, and executing the corresponding function.
#### Parameters:
- `input` (char*): Pointer to the input string containing the command and arguments.
#### Returns:
- `true` (1) if the command executes successfully.
- `false` (0) if the command is not found or fails.

---
### `makeStringUpperCase`

```c
void makeStringUpperCase(char* str) {
  if (str == NULL) return;

  for (size_t i = 0; i < strlen(str); i++) str[i] = (char)toupper(str[i]);
}
```
#### Description:
Converts a string to uppercase.
#### Parameters:
- `str` (char*): Pointer to the string to be converted.
---
### Command Execution Functions

#### `cmd_led`

```c
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
```

Controls an LED based on the provided arguments.

#### `cmd_motor`

```c
bool cmd_motor(char* args) {
  Serial.print("Setting motor speed: ");
  Serial.println(args);
  return 1;
}
```

Controls a motor based on the provided arguments.

#### `cmd_status`

```c
bool cmd_status(char* args) {
  Serial.println("Arduino running fine!");
  return 1;
}
```

Returns system status.
## Usage Example

```c
char inputCommand[] = "LED ON";
if (processCommand(inputCommand)) {
    Serial.println("Command executed successfully.");
} else {
    Serial.println("Unknown command.");
}
```

## Notes

- Commands should be registered in `commands[]` for recognition.
- Arguments are passed as part of the input string and parsed within the command functions.
- String processing should be efficient to minimize processing delays.
    

## Future Improvements
- Introduce error handling for invalid or malformed commands.