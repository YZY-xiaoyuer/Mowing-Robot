
#ifndef __SHELL_H
#define __SHELL_H
#include <stdarg.h>
#include <string.h>
#include "usart.h"
#include <cmsis_os.h>

#define KEY_ESC							(0X1B)


/*The length of the command entered by the keyboard*/
#define SHELL_COMMAND_MAX_LEN 			(32)

/*the max length of the console receive buffer*/
#define SHELL_RCV_BUFFER_LEN  			(SHELL_COMMAND_MAX_LEN)

/*the max length of the console send buffer*/
#define SHELL_SEND_BUFFER_LEN  			(128)

/* Number of save commands */
#define SHELL_CMD_BUFFER_LENGTH			(5)

/* The number of letters in the word */
#define SHELL_WORD_NUM_MAX				(5)

/* Maximum number of commands */
#define SHELL_CMD_MAX					(20)

/* Command function form */
typedef uint8_t(* p_cmd_func)(char * p0, char * p1, char * p2, char * p3);

extern int vsprintf(char * /*s*/, const char * /*format*/, va_list /*arg*/);


typedef enum _enum_shell_type {
    SHELL_CONSOLE = 0U,
    SHELL_REMOTE,
} E_enum_shell_type;

typedef struct _shell_word {
    char * p_addr;
    uint8_t len;
} ST_shell_word;

typedef struct _shell_sentence {
    ST_shell_word st_word[SHELL_WORD_NUM_MAX];
    uint8_t number;
} ST_shell_sentence;

typedef struct _cmd_handle {
    const char * p_key;
    p_cmd_func func;
    const char * p_help;
} ST_cmd_handle;

typedef struct _shell_cmd_buffer {
    uint32_t shell_cmd_len; 							/*The length of the command entered by the keyboard (can be deleted with the backspace key)*/
    char shell_cmd_buf[SHELL_COMMAND_MAX_LEN + 1];		/* Save the data received by the console */
} ST_shell_cmd_buffer;

typedef struct _shell_config_handle {
    uint32_t shell_rcv_buf_point;
    uint8_t shell_rcv_buf[SHELL_RCV_BUFFER_LEN];					/*the buffer of the date which receive frome the console*/
    uint8_t shell_cmd_buf_point;									/*the  point of the st_shell_cmd_buf*/
    ST_shell_cmd_buffer st_shell_cmd_buf[SHELL_CMD_BUFFER_LENGTH];
    uint8_t cmd_hanlde_point;
    ST_cmd_handle st_cmd_handle[SHELL_CMD_MAX];						/*the handle of the commands*/
	E_enum_shell_type	e_shell_type;
} ST_shell_config_handle;

typedef struct _shell_handle {
    uint8_t (*init)(void);
    uint8_t (*reg_cmd)(const char * p_key, p_cmd_func pf_func, const char * p_help);
    uint8_t (*rcv)(uint32_t len);
    uint8_t (*output)(uint8_t * data, uint32_t len);
	  uint8_t (*set_type)(E_enum_shell_type e_shell_type);
    uint8_t (*deinit)(void);
} ST_shell_handle;


extern osSemaphoreId Usart_Shell_SemHandle;

extern ST_shell_handle st_usart_shell;
extern uint8_t* p_usart_shell_rcv_buf;

#endif
