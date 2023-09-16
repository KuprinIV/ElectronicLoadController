/*
 * print_to_string.c
 *
 *  Created on: 16 сент. 2023 г.
 *      Author: Ilya
 */
#include "main.h"
#include "print_to_string.h"

// driver functions
static void printFloat(char* res_str, const char* prestr, float* nums, const char **delims, int* precision, int size);
static void printInteger(char* res_str, const char* prestr, int* nums, const char **delims, int size);
static void printStrings(char* res_str, const char **strs, int size);
static void printString(char* res_str, const char* prestr, const char* str);
static int strLen(char* str);

// inner functions
static int pow_10(int exp);
static int getDigitsNum(int number);

// variables
PrintToString p2s = {
		printFloat,
		printInteger,
		printStrings,
		printString,
		strLen
};

PrintToString* print2str_drv = &p2s;

/**
  * @brief  Print float numbers array to string
  * @param  res_str - pointer to result string
  * @param  prestr - string data before numbers
  * @param  nums - float numbers array
  * @param  delims - string delimiters past numbers
  * @param  precision - float numbers printing precision (numbers after dot)
  * @param  size - float values number
  * @retval None
  */
static void printFloat(char* res_str, const char* prestr, float* nums, const char** delims, int *precision, int size)
{
    int prestr_len = strLen((char*)prestr);
    int delim_len = 0;
    int index = 0;
    int tempf = 0;
    int mul = 1;
    int digit = 0;
    int digits_num = 0;
    int is_less_one = 0;

    // copy pre-string to result string
    for(int i = 0; i < prestr_len; i++)
    {
        res_str[index++] = prestr[i];
    }

    // print float numbers
    for(int i = 0; i < size; i++)
    {
       // calculate precision multiplier for converting to int
       mul = pow_10(precision[i]);

       // check number less 1
        if(nums[i] < 1.0f)
        {
            is_less_one = 1;
        }
        else
        {
            is_less_one = 0;
        }
       // convert float number to integer with set precision
       tempf = (int)(nums[i]*mul*10);
       // make tempf round
       if((tempf % 10) >= 5)
       {
           tempf = tempf/10+1;
       }
       else
       {
           tempf /= 10;
       }
       // check is number negative
       if(tempf < 0)
       {
            // print minus to result string
            res_str[index++] = '-';
            tempf = -tempf;
       }
       digits_num = getDigitsNum(tempf);
       if(tempf < mul)
       {
           if(tempf > 0)
           {
               digits_num += getDigitsNum(mul/tempf);
               for(int n = 0; n < precision[i]; n++)
               {
                   if(tempf == pow_10(n))
                   {
                       digits_num--;
                       break;
                   }
               }
           }
           else
           {
               digits_num += precision[i]+1;
           }
       }

       // print digits to result string
       for(int j = 0; j < digits_num; j++)
       {
           // get current digit
           if(j == digits_num-precision[i]) res_str[index++] = '.';
           digit = (tempf/pow_10(digits_num-1-j))%10;
           if(digit == 0 && (j < digits_num-precision[i]-1 && !is_less_one)) continue; // skip leading zero
           res_str[index++] = (char)(digit+0x30);
       }

       // print delimiter to result string
       if(delims != NULL)
       {
           delim_len = strLen((char*)delims[i]);
           for(int k = 0; k < delim_len; k++)
           {
               res_str[index++] = delims[i][k];
           }
       }
    }
    // print zero-symbol
    res_str[index++] = '\0';
}

/**
  * @brief  Print integer numbers array to string
  * @param  res_str - pointer to result string
  * @param  prestr - string data before numbers
  * @param  nums - integer numbers array
  * @param  delims - string delimiters past numbers
  * @param  size - integer values number
  * @retval None
  */
static void printInteger(char* res_str, const char* prestr, int* nums, const char **delims, int size)
{
    int prestr_len = strLen((char*)prestr);
    int delim_len = 0;
    int index = 0;
    int digit = 0;
    int digits_num = 0;

    // copy pre-string to result string
    for(int i = 0; i < prestr_len; i++)
    {
        res_str[index++] = prestr[i];
    }

    // print integer numbers
    for(int i = 0; i < size; i++)
    {
       // check is number negative
       if(nums[i] < 0)
       {
            // print minus to result string
            res_str[index++] = '-';
            nums[i] = -nums[i];
       }
       digits_num = getDigitsNum(nums[i]);

       // print digits to result string
       for(int j = 0; j < digits_num; j++)
       {
           // get current digit
           digit = (nums[i]/pow_10(digits_num-1-j))%10;
           res_str[index++] = (char)(digit+0x30);
       }

       // print delimiter to result string
       if(delims != NULL)
       {
           delim_len = strLen((char*)delims[i]);
           for(int k = 0; k < delim_len; k++)
           {
               res_str[index++] = delims[i][k];
           }
       }
    }
    // print zero-symbol
    res_str[index++] = '\0';
}

/**
  * @brief  Print strings array to string
  * @param  res_str - pointer to result string
  * @param  strs - strings array
  * @param  size - strings array length
  * @retval None
  */
static void printStrings(char* res_str, const char** strs, int size)
{
    int str_len = 0;
    int index = 0;
    // print strings
    if(strs != NULL)
    {
        for(int i = 0; i < size; i++)
        {
            str_len = strLen((char*)strs[i]);
            for(int j = 0; j < str_len; j++)
            {
                res_str[index++] = strs[i][j];
            }
        }
    }
    // print zero-symbol
    res_str[index++] = '\0';
}

/**
  * @brief  Print string to string
  * @param  res_str - pointer to result string
  * @param  prestr - string data before numbers
  * @param  str - printing string data
  * @retval None
  */
static void printString(char* res_str, const char* prestr, const char* str)
{
	int prestr_len = strLen((char*)prestr);
    int str_len = strLen((char*)str);
    int index = 0;

    // copy pre-string to result string
    for(int i = 0; i < prestr_len; i++)
    {
        res_str[index++] = prestr[i];
    }

    // copy string to result string
    for(int i = 0; i < str_len; i++)
    {
        res_str[index++] = str[i];
    }
    // print zero-symbol
    res_str[index++] = '\0';
}

/**
  * @brief  Get string length
  * @param  str - pointer to string
  * @retval string symbols number
  */
static int strLen(char* str)
{
    int len = 0;
    while((*str++) != '\0')
    {
        len++;
    }
    return len;
}

/**
  * @brief  Calculate power degree of 10
  * @param  exp - power degree
  * @retval calculated value
  */
static int pow_10(int exp)
{
    int res = 1;
    for(int i = 0; i < exp; i++)
    {
        res *= 10;
    }
    return res;
}

/**
  * @brief  Calculate digits number in integer value
  * @param  number - integer value
  * @retval calculated value
  */
static int getDigitsNum(int number)
{
    int res_num = 0;
    int temp_num = number;
    while(temp_num > 0)
    {
        temp_num /= 10;
        res_num++;
    }
    return res_num;
}

