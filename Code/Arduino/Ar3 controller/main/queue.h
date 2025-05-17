#ifndef QUEUE_H
#define QUEUE_H

//---------------------------- include library ------------------------------------------------------------------------
#include <Arduino.h>
#include "global.h"

//---------------------------- Program Constants ----------------------------------------------------------------------
#define QUEUE_SIZE 16

//---------------------------- Structure Definitions ------------------------------------------------------------------
struct CommandQueue {
  char buffer[QUEUE_SIZE][CMD_BUFFER_SIZE];  // Stores up to QUEUE_SIZE commands
  int head;                                  // Index of the next command to process
  int tail;                                  // Index where the next incoming command is stored
  int count;                                 // Number of commands currently stored
};

//----------------------------- Globals -------------------------------------------------------------------------------
extern CommandQueue commandQueue;

//----------------------------- Function Prototypes -------------------------------------------------------------------

/**
 * @brief Adds a command to the queue for processing.
 *
 * This function stores the given command in the command queue if there is space available.
 * If the queue is full, the command is not added.
 *
 * @param cmd Pointer to the command string to enqueue.
 * @return True if the command was successfully enqueued, false if the queue was full.
 */
bool enqueueCommand(const char *cmd);

/**
 * @brief Retrieves the next command from the queue.
 *
 * This function removes the oldest command from the queue and copies it to the provided buffer.
 * If the queue is empty, no command is dequeued.
 *
 * @param cmd Pointer to the buffer where the dequeued command will be stored.
 * @return True if a command was dequeued, false if the queue was empty.
 */
bool dequeueCommand(char *cmd);
#endif