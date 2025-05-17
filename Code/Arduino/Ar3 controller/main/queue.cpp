//---------------------------- include library ------------------------------------------------------------------------
#include "queue.h"

//---------------------------- Program Constants ----------------------------------------------------------------------
CommandQueue commandQueue = {{{0}}, 0, 0, 0};

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Adds a command to the queue for processing.
 *
 * This function stores the given command in the command queue if there is space available.
 * If the queue is full, the command is not added.
 *
 * @param cmd Pointer to the command string to enqueue.
 * @return True if the command was successfully enqueued, false if the queue was full.
 */
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

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Retrieves the next command from the queue.
 *
 * This function removes the oldest command from the queue and copies it to the provided buffer.
 * If the queue is empty, no command is dequeued.
 *
 * @param cmd Pointer to the buffer where the dequeued command will be stored.
 * @return True if a command was dequeued, false if the queue was empty.
 */
bool dequeueCommand(char *cmd) {
    if (commandQueue.count > 0) {
        strncpy(cmd, commandQueue.buffer[commandQueue.head], CMD_BUFFER_SIZE);
        commandQueue.head = (commandQueue.head + 1) % QUEUE_SIZE;
        commandQueue.count--;
        return true;
    }
    return false;
}