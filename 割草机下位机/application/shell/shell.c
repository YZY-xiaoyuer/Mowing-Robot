#include <cstdlib>
#include <cstdio>
#include <stdlib.h>
#include "shell.h"
#include "usart.h"
#include "usercmd.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"


#define KEY_BACK_CTL  	 				(0x08)
#define KEY_TAB        					(0x09)
#define KEY_BACK_CHAR  					(0x7F)


static ST_shell_config_handle st_usart_shell_cfg_handle = {0};
 uint8_t* p_usart_shell_rcv_buf = st_usart_shell_cfg_handle.shell_rcv_buf;


static uint8_t Shell_Send(uint8_t * p_data, uint32_t len);

static uint8_t Shell_Reg_Cmd(ST_shell_config_handle* pst_shell_config_hanld, const char * p_key, p_cmd_func func, const char * p_help);
static uint8_t Shell_Analyse_Command(ST_shell_config_handle* pst_shell_config_hanld, char * p_command, uint32_t cmdLen);
static uint8_t Shell_Rcv(ST_shell_config_handle* pst_shell_config_hanld,const uint8_t * p_data, uint32_t len);
static uint8_t Shell_Init(ST_shell_config_handle* pst_shell_config_hanld);
static uint8_t Shell_Output(const uint8_t *p_data, uint32_t len);
static uint8_t Shell_Deinit(ST_shell_config_handle* pst_shell_config_hanld);
static uint8_t Shell_Set_Type(ST_shell_config_handle* pst_shell_config_hanld, E_enum_shell_type e_shell_type);



static uint8_t Usart_Shell_Reg_Cmd(const char * p_key, p_cmd_func func, const char * p_help);
static uint8_t Usart_Shell_Rcv(uint32_t len);
static uint8_t Usart_Shell_Init(void);
static uint8_t Usart_Shell_Output(uint8_t *p_data, uint32_t len);
static uint8_t Usart_Shell_Deinit(void);
static uint8_t Usart_Shell_Set_Type(E_enum_shell_type e_shell_type);

ST_shell_handle st_usart_shell = {Usart_Shell_Init, Usart_Shell_Reg_Cmd, Usart_Shell_Rcv, Usart_Shell_Output, Usart_Shell_Set_Type, Usart_Shell_Deinit};

static uint8_t Shell_Send(uint8_t * p_data, uint32_t len)
{
    HAL_UART_Transmit_DMA(&huart1, p_data, len);
    return 0;
}

static uint8_t Shell_Set_Type(ST_shell_config_handle* pst_shell_config_hanld, E_enum_shell_type e_shell_type)
{
    pst_shell_config_hanld->e_shell_type = e_shell_type;
    return 0;
}

static uint8_t Shell_Reg_Cmd(ST_shell_config_handle* pst_shell_config_hanld, const char * p_key, p_cmd_func func, const char * p_help)
{
    if (pst_shell_config_hanld->cmd_hanlde_point == SHELL_CMD_MAX) {
        return 1;
    } else {
        pst_shell_config_hanld->st_cmd_handle[pst_shell_config_hanld->cmd_hanlde_point].p_key = p_key;
        pst_shell_config_hanld->st_cmd_handle[pst_shell_config_hanld->cmd_hanlde_point].func = func;
        pst_shell_config_hanld->st_cmd_handle[pst_shell_config_hanld->cmd_hanlde_point].p_help = p_help;
        pst_shell_config_hanld->cmd_hanlde_point++;
        return 0;
    }
}


static uint8_t Shell_Analyse_Command(ST_shell_config_handle* p_st_shell_config_hanld, char* p_command, uint32_t cmdLen)
{
    int	num = 0;
    uint8_t begin = 0;
    /* 0: indicates that the word header is being checked 1: the word tail is being queried */
    uint8_t state = 0;;
    ST_shell_sentence st_sentence = {0};
printf("\033[2J\033[1H");//--清屏指令
printf("\r\n==========================================================================");
printf("\r\n                     深圳市伽利略机器人有限公司     (2019)               =");	
printf("\r\n=                                                                        =");	
printf("\r\n=                      STM32F103CBT6 APP程序        (版本 2.0.0)         =");	
printf("\r\n=                                                                        =");	
printf("\r\n=                           小鱼儿飞丫飞                                 =");	
printf("\r\n==========================================================================");	
printf("\r\n请输入help获取相关信息....\r\n>>");

    /* Decompose the entire sentence into individual words */

    for (num = 0; num < cmdLen; num++) {
        if (state) {
            /* Querying word tail */
            if (p_command[num] == ' ') {
                /* When a space is found, the word has ended. */
                if (st_sentence.number < SHELL_WORD_NUM_MAX) {
                    st_sentence.st_word[st_sentence.number].p_addr = p_command + begin;
                    st_sentence.st_word[st_sentence.number].len = num - begin;
                    st_sentence.number++;
                }

                p_command[num] = '\0'; /* Replace the space with a cutoff to facilitate subsequent string comparisons. */
                state = 0;
            }
        } else {
            /* Query the beginning of a word */
            if ((p_command[num] >= 'a' && p_command[num]  <= 'z')
            || (p_command[num] >= 'A' && p_command[num]  <= 'Z')
            || (p_command[num] >= '0' && p_command[num] <= '9')) {
                begin = num;
                state = 1;
            }
        }
    }

    if (state) {
        /* last word */
        if (st_sentence.number < SHELL_WORD_NUM_MAX) {
            st_sentence.st_word[st_sentence.number].p_addr = p_command + begin;
            st_sentence.st_word[st_sentence.number].len = num - begin + 1;
            st_sentence.number++;
        }

        p_command[num + 1] = '\0';
    }

    if (st_sentence.number == 0) {
        return 1;
    }

    char * key = NULL;
    key = st_sentence.st_word[0].p_addr;
    ST_cmd_handle *pst_cmd = NULL;

    /* Response command function */
    if (((key[0] == 'H') || (key[0] == 'h'))
    && ((key[1] == 'E') || (key[1] == 'e'))
    && ((key[2] == 'L') || (key[2] == 'l'))
    && ((key[3] == 'P') || (key[3] == 'p'))) {
        static char str_help[516] = {0};
        memset(str_help, 0, 516);
        osDelay(20);
        strcat(str_help, "\r\n---------- 帮助 ------------\r\n");

        for (num = 0; num < p_st_shell_config_hanld->cmd_hanlde_point; num++) {
            pst_cmd = &(p_st_shell_config_hanld->st_cmd_handle[num]);
            char str_out[30] = {0};
            sprintf(str_out, "%d. %s ---- %s \r\n", num, pst_cmd->p_key, pst_cmd->p_help);
            strcat(str_help, str_out);
        }

        strcat(str_help, ">>");
        Shell_Send((uint8_t *)str_help, strlen(str_help));
        return 0;
    }

    /* Determine if the words are all numbers */
    if (strspn(key, "0123456789") == strlen(key)) {
        int cmd_num = 0;
        sscanf(key, "%d", &cmd_num);
        pst_cmd = &(p_st_shell_config_hanld->st_cmd_handle[cmd_num]);

        if(cmd_num <= p_st_shell_config_hanld->cmd_hanlde_point) {
            switch (st_sentence.number) {
                case 1:
                    pst_cmd->func(0, 0, 0, 0);
                    break;

                case 2:
                    pst_cmd->func(st_sentence.st_word[1].p_addr, 0, 0, 0);
                    break;

                case 3:
                    pst_cmd->func(st_sentence.st_word[1].p_addr, st_sentence.st_word[2].p_addr, 0, 0);
                    break;

                case 4:
                    pst_cmd->func(st_sentence.st_word[1].p_addr, st_sentence.st_word[2].p_addr, st_sentence.st_word[3].p_addr, 0);
                    break;

                default:
                    pst_cmd->func(st_sentence.st_word[1].p_addr, st_sentence.st_word[2].p_addr,
                    st_sentence.st_word[3].p_addr, st_sentence.st_word[4].p_addr);
                    break;
            }
        } else {
            Shell_Send((uint8_t *)"\r\n没有找到这个命令!\r\n>>",(uint32_t)strlen("\r\n没有找到这个命令!\r\n>>"));
        }
    } else {
        for (num = 0; num < p_st_shell_config_hanld->cmd_hanlde_point; num++) {
            pst_cmd = &(p_st_shell_config_hanld->st_cmd_handle[num]);

            if (strcmp(pst_cmd->p_key, key) == 0) {
                /* find the order */
                switch (st_sentence.number) {
                    case 1:
                        pst_cmd->func(0, 0, 0, 0);
                        break;

                    case 2:
                        pst_cmd->func(st_sentence.st_word[1].p_addr, 0, 0, 0);
                        break;

                    case 3:
                        pst_cmd->func(st_sentence.st_word[1].p_addr, st_sentence.st_word[2].p_addr, 0, 0);
                        break;

                    case 4:
                        pst_cmd->func(st_sentence.st_word[1].p_addr, st_sentence.st_word[2].p_addr, st_sentence.st_word[3].p_addr, 0);
                        break;

                    default:
                        break;
                }

                break;
            }
        }

        if (num >= p_st_shell_config_hanld->cmd_hanlde_point) {
            Shell_Send((uint8_t *)"\r\n没有找到这个命令!\r\n>>",(uint32_t)strlen("\r\n没有找到这个命令!\r\n>>"));

        }
    }

    return 0;
}


static uint8_t Shell_Rcv(ST_shell_config_handle* pst_shell_config_hanld, const uint8_t * p_data,  uint32_t len)
{
    for (volatile uint32_t i = 0; i < len; i++) {
        char* p_shell_cmd_buf = NULL;
        uint32_t* p_shell_cmd_len = NULL;
        p_shell_cmd_buf = pst_shell_config_hanld->st_shell_cmd_buf[pst_shell_config_hanld->shell_cmd_buf_point].shell_cmd_buf;
        p_shell_cmd_len = &(pst_shell_config_hanld->st_shell_cmd_buf[pst_shell_config_hanld->shell_cmd_buf_point].shell_cmd_len);

        switch (*(p_data + i)) {
            case '\r':

                /* Excuting an order */
                if(*p_shell_cmd_len) {
										uint32_t cmd_len = *p_shell_cmd_len;
                    Shell_Analyse_Command(pst_shell_config_hanld, p_shell_cmd_buf, cmd_len);
                    pst_shell_config_hanld->shell_cmd_buf_point++;

                    if(pst_shell_config_hanld->shell_cmd_buf_point >= SHELL_CMD_BUFFER_LENGTH) {
                        pst_shell_config_hanld->shell_cmd_buf_point = 0;
                    }
						
                   pst_shell_config_hanld->st_shell_cmd_buf[pst_shell_config_hanld->shell_cmd_buf_point].shell_cmd_len = 0;
                }

                Shell_Send((uint8_t *)"\r\n>>", 4);
                break;

            case '\n':
                break;

            case KEY_BACK_CTL:
            case KEY_BACK_CHAR:
                if (*p_shell_cmd_len) {
                    (*p_shell_cmd_len)--;
                    p_shell_cmd_buf[*p_shell_cmd_len] = 0;
                    Shell_Send((uint8_t *)"\b \b", 3);
                }

                break;

            case KEY_TAB:
                if (*p_shell_cmd_len >= (SHELL_COMMAND_MAX_LEN - 4)) {
                    return 1;
                }

                p_shell_cmd_buf[*p_shell_cmd_len] = ' ';
                (*p_shell_cmd_len)++;
                p_shell_cmd_buf[*p_shell_cmd_len] = ' ';
                (*p_shell_cmd_len)++;
                p_shell_cmd_buf[*p_shell_cmd_len] = ' ';
                (*p_shell_cmd_len)++;
                p_shell_cmd_buf[*p_shell_cmd_len] = ' ';
                (*p_shell_cmd_len)++;
                p_shell_cmd_buf[*p_shell_cmd_len] = '\0';
                Shell_Send((uint8_t *)"    ", 4);
                break;

            default:
                if (*p_shell_cmd_len >= (SHELL_COMMAND_MAX_LEN - 1)) {
                    return 1;
                }

                p_shell_cmd_buf[*p_shell_cmd_len] = p_data[i];
                (*p_shell_cmd_len)++;
                p_shell_cmd_buf[*p_shell_cmd_len] = '\0';
                /* echo */
                Shell_Send((uint8_t *)(p_data + i), 1);
        }
    }

    return 0;
}

static uint8_t Shell_Init(ST_shell_config_handle* pst_shell_config_hanld)
{
    return 0;
}
static uint8_t Shell_Deinit(ST_shell_config_handle* pst_shell_config_hanld)
{
    return 0;
}
static uint8_t Shell_Output(const uint8_t *p_data, uint32_t len)
{
    return  Shell_Send((uint8_t *)p_data, len);
}

static uint8_t Usart_Shell_Rcv(uint32_t len)
{
    return Shell_Rcv(&st_usart_shell_cfg_handle, st_usart_shell_cfg_handle.shell_rcv_buf, len);
}

static uint8_t Usart_Shell_Reg_Cmd(const char * p_key, p_cmd_func func, const char * p_help)
{
    return Shell_Reg_Cmd(&st_usart_shell_cfg_handle, p_key, func, p_help);
}

static uint8_t Usart_Shell_Set_Type(E_enum_shell_type e_shell_type)
{
    return Shell_Set_Type(&st_usart_shell_cfg_handle, e_shell_type);
}

static uint8_t Usart_Shell_Init(void)
{
    HAL_UART_Receive_IT(&huart1, st_usart_shell_cfg_handle.shell_rcv_buf, 1);
    Usart_Shell_Reg_Cmd("重启", Cmd_Reboot, "重启机器");
    Usart_Shell_Reg_Cmd("掉电", Cmd_Power_Off, "断开电源");
    Usart_Shell_Reg_Cmd("遥控", Cmd_Remote, "遥控机器，输入h获取帮助");
    Usart_Shell_Reg_Cmd("显示", Cmd_Display, "串口输出相关数据, 输入h获取帮助");
    Usart_Shell_Reg_Cmd("设置", Cmd_Set_Pid, "设置相关参数，输入h获取帮助");
    return Shell_Init(&st_usart_shell_cfg_handle);
}

static uint8_t Usart_Shell_Output( uint8_t *p_data, uint32_t len)
{
    return Shell_Output(p_data, len);
}

static uint8_t Usart_Shell_Deinit(void)
{
    return Shell_Deinit(&st_usart_shell_cfg_handle);
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);

    if (st_usart_shell_cfg_handle.e_shell_type == SHELL_CONSOLE) {
        osSemaphoreRelease(Usart_Shell_SemHandle);
        HAL_UART_Receive_IT(&huart1, st_usart_shell_cfg_handle.shell_rcv_buf, 1);
    }else if (st_usart_shell_cfg_handle.e_shell_type == SHELL_REMOTE) {
     HAL_UART_Receive_IT(&huart1, &cmd_remote_rcv, 1);
    }
		HAL_Delay(1);
}


