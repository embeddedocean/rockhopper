/*
 * console.h
 *
 * Created: 5/24/2017 
 *  Author: chris
 */ 


#ifndef CONSOLE_H_
#define CONSOLE_H_

void setup_console(void);
int console_gets(char *str);
int read_console_input(void);
int console_prompt_int(const char *str, int value, int timeout);




#endif /* INCFILE1_H_ */