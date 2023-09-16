/*
 * print_to_string.h
 *
 *  Created on: 16 сент. 2023 г.
 *      Author: Ilya
 */

#ifndef INC_PRINT_TO_STRING_H_
#define INC_PRINT_TO_STRING_H_

typedef struct
{
	void (*PrintFloat)(char* res_str, const char* prestr, float* nums, const char **delims, int* precision, int size);
	void (*PrintInteger)(char* res_str, const char* prestr, int* nums, const char **delims, int size);
	void (*PrintStrings)(char* res_str, const char **strs, int size);
	void (*PrintString)(char* res_str, const char* prestr, const char* str);
	int (*StrLen)(char* str);
}PrintToString;

extern PrintToString* print2str_drv;

#endif /* INC_PRINT_TO_STRING_H_ */
