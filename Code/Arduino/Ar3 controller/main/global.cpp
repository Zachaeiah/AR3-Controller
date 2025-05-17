#include <stddef.h>
#include "global.h"


//---------------------------- Program Constants ----------------------------------------------------------------------
File dataFile;  // Define the global variable

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Prints a formatted error message to the serial console and log file.
 *
 * This function formats and prints an error message with detailed information 
 * including the function where the error occurred, the line number, the severity 
 * of the error, and additional user-supplied error details. The message is printed 
 * to the serial console and, if a valid log file is open, written to the file as well.
 *
 * @param error_index - The index representing the specific error in a predefined 
 * @param funcError - The name of the function (obtained using `__func__`)
 * @param line - The line number in the code (typically supplied by `__LINE__`)
 * @param severity - The severity of the error message, such as "INFO", "WARNING", 
 *                   or "ERROR".
 * @param strError - A string containing the error message format,
 * @param ... - Additional arguments for the formatted error message, matching the 
 *              placeholders in `strError`.
 */
void print_error(int error_index, const char* funcError, int line, const char* severity, const char* strError, ...) {
  va_list args;                    // Variable argument list
  error_index = abs(error_index);  // Ensure a positive error index

  va_start(args, strError);  // Initialize variable argument list

  // Prepare the formatted error message
  char formattedMessage[512];
  snprintf(formattedMessage, sizeof(formattedMessage), "[Line: %d] [Function: %s] [Error Severity: %s] Error: %d - ", line, funcError, severity, error_index);

  // Append the formatted error message with additional arguments
  vsnprintf(formattedMessage + strlen(formattedMessage), sizeof(formattedMessage) - strlen(formattedMessage), strError, args);

  // Print the complete formatted error message using dsprintf
  dsprintf("%s\n", formattedMessage);

  va_end(args);  // End variable argument list
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Prints a formatted string to the serial console and logs it to a file if the file is open.
 *
 * This function allows formatted strings to be printed to both the serial console and, 
 * if a valid log file (`dataFile`) is open, written to the file. It also includes a 
 * persistent call count that is appended to each logged message to help track the 
 * order of log entries.
 *
 * @param fmt - The format string used for the message, similar to `printf`. 
 * @param ... - Additional arguments that will replace the placeholders in `fmt`.
 * @return int - The length of the formatted string that was printed, 
 */
int dsprintf(const char* fmt, ...) {

  static size_t call_count = 0;  // Persistent counter for function calls
  int len = 0;                   //the length of the printed string
  char buf[256];                 //print buffer on the stack
  va_list args;
  String logEntry;


  va_start(args, fmt);
  len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  logEntry = "L" + String(call_count) + ": " + buf;  // Add call count to log entry

  if (dataFile) {
    dataFile.print(logEntry);
    dataFile.flush();  // Ensure data is written
  } else {
    Serial.println("Error: Data file not open.");
  }

  Serial.print(logEntry);

  call_count++;  // Increment the call count
  return len;
}
