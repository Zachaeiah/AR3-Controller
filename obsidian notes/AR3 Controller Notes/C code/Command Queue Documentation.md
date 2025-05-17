## Overview

The `CommandQueue` system provides a simple FIFO (First-In-First-Out) queue to store and process command strings in an embedded system running on an Arduino-compatible microcontroller. This system ensures efficient command management while preventing overflow issues.

## Constants

- **QUEUE_SIZE** (`16`): Defines the maximum number of commands the queue can store.
## Structure Definition

The `CommandQueue` struct maintains the state of the command queue:

```c
struct CommandQueue {
  char buffer[QUEUE_SIZE][CMD_BUFFER_SIZE];  // Stores up to QUEUE_SIZE commands
  int head;                                  // Index of the next command to process
  int tail;                                  // Index where the next incoming command is stored
  int count;                                 // Number of commands currently stored
};
```

### Structure Fields:

- **buffer**: A two-dimensional array storing up to `QUEUE_SIZE` commands, each with a maximum length of `CMD_BUFFER_SIZE`.
- **head**: Index of the next command to be processed (dequeued).
- **tail**: Index where the next incoming command will be stored (enqueued).
- **count**: Tracks the current number of stored commands.

## Global Variable

```c
extern CommandQueue commandQueue;
```

- A global instance of `CommandQueue`, allowing queue operations throughout the program.
## Function Descriptions

### `enqueueCommand`

```c
bool enqueueCommand(const char *cmd);
```

#### Description:

Adds a command to the queue for processing.
#### Parameters:
- `cmd` (const char*): Pointer to the command string to be enqueued.
#### Returns:
- `true` if the command was successfully added.
- `false` if the queue is full.

#### Behavior:
1. Checks if the queue has available space.
2. Copies the command into the queue buffer at the `tail` index.
3. Updates `tail` and `count` accordingly.
4. Returns `false` if the queue is full.
---
### `dequeueCommand`

```c
bool dequeueCommand(char *cmd);
```
#### Description:
Retrieves the next command from the queue.
#### Parameters:
- `cmd` (char*): Pointer to the buffer where the dequeued command will be stored.

#### Returns:
- `true` if a command was dequeued successfully.
- `false` if the queue is empty.

#### Behavior
1. Checks if the queue contains any commands.
2. Copies the oldest command from the `head` index into the provided buffer.
3. Updates `head` and `count` accordingly.
4. Returns `false` if the queue is empty.

## Function Implementations

### `enqueueCommand`

```c
bool enqueueCommand(const char *cmd) {
    if (commandQueue.count < QUEUE_SIZE) { // Ensure queue is not full
        strncpy(commandQueue.buffer[commandQueue.tail], cmd, CMD_BUFFER_SIZE - 1);
        commandQueue.buffer[commandQueue.tail][CMD_BUFFER_SIZE - 1] = '\0'; // Ensure null termination
        commandQueue.tail = (commandQueue.tail + 1) % QUEUE_SIZE;
        commandQueue.count++;
        return true;
    } else {
        print_error(QUEUE_FULL, __func__, __LINE__, "HIGH", "Queue Full - Dropping Command! ");
        return false;
    }
    return false;
}
```
#### Description:
Adds a command to the queue for processing.
#### Parameters:
- `cmd` (const char*): Pointer to the command string to be enqueued.
#### Returns:
- `true` if the command was successfully added.
- `false` if the queue is full.

#### Behavior:
1. Checks if the queue has available space.
2. Copies the command into the queue buffer at the `tail` index.
3. Ensures the string is null-terminated.
4. Updates `tail` and `count` accordingly.
5. Logs an error if the queue is full.


---
### `dequeueCommand`

```c
bool dequeueCommand(char *cmd) {
    if (commandQueue.count > 0) {
        strncpy(cmd, commandQueue.buffer[commandQueue.head], CMD_BUFFER_SIZE);
        commandQueue.head = (commandQueue.head + 1) % QUEUE_SIZE;
        commandQueue.count--;
        return true;
    }
    return false;
}
```

#### Description:
Retrieves the next command from the queue.
#### Parameters:
- `cmd` (char*): Pointer to the buffer where the dequeued command will be stored.
#### Returns:
- `true` if a command was dequeued successfully.
- `false` if the queue is empty.
#### Behavior:
1. Checks if the queue contains any commands.
2. Copies the oldest command from the `head` index into the provided buffer.
3. Updates `head` and `count` accordingly.
4. Returns `false` if the queue is empty.

## Usage Example

```c
char commandBuffer[CMD_BUFFER_SIZE];

if (enqueueCommand("MOVE X10 Y20")) {
    Serial.println("Command added to queue");
} else {
    Serial.println("Queue is full");
}

if (dequeueCommand(commandBuffer)) {
    Serial.print("Processing Command: ");
    Serial.println(commandBuffer);
} else {
    Serial.println("Queue is empty");
}
```

## Notes

- The queue follows a circular buffer implementation, ensuring efficient memory usage.
- Ensuring that commands do not exceed `CMD_BUFFER_SIZE` is crucial to avoid buffer overflow issues.
- The queue is designed for single-threaded environments like embedded systems.

## Future Improvements
- Implement a thread-safe version for multi-threaded environments.
- Add a function to clear the queue when necessary.
- Implement dynamic resizing if more commands need to be stored.