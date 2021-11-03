/*******************************************************************************
*     File Name :                        matrix.h
*     Library/Module Name :              MatrixComputation
*     Author :                           Marc Pony(marc_pony@163.com)
*     Create Date :                      2021/6/28
*     Abstract Description :             矩阵运算库头文件
*******************************************************************************/

#ifndef __MATRIX_H__
#define __MATRIX_H__

/*******************************************************************************
* (1)Debug Switch Section
*******************************************************************************/

/*******************************************************************************
* (2)Include File Section
*******************************************************************************/
#include "common.h"

/*******************************************************************************
* (3)Macro Define Section
*******************************************************************************/

/*******************************************************************************
* (4)Struct(Data Types) Define Section
*******************************************************************************/

/*******************************************************************************
* (5)Prototype Declare Section
*******************************************************************************/

REAL PickEleInMat(MATRIX *a, int r, int c);
void print_matrix(MATRIX *a, STRING string);

/**********************************************************************************************
Function: creat_matrix
Description: 创建矩阵
Input: 矩阵行数rows，列数columns
Output: 错误号指针errorID，栈指针S
Input_Output: 无
Return: 矩阵指针
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
MATRIX *creat_matrix(_IN INTEGER rows, _IN INTEGER columns, _OUT ERROR_ID *errorID, _OUT STACKS *S);

/**********************************************************************************************
Function: creat_multiple_matrices
Description: 创建多个矩阵
Input: 矩阵行数rows，列数columns，个数count
Output: 错误号指针errorID，栈指针S
Input_Output: 无
Return: 矩阵指针
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
MATRIX *creat_multiple_matrices(_IN INTEGER rows, _IN INTEGER columns, _IN INTEGER count, _OUT ERROR_ID *errorID, _OUT STACKS *S);

/**********************************************************************************************
Function: creat_zero_matrix
Description: 创建零矩阵
Input: 矩阵行数rows，列数columns
Output: 错误号指针errorID，栈指针S
Input_Output: 无
Return: 矩阵指针
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
MATRIX *creat_zero_matrix(_IN INTEGER rows, _IN INTEGER columns, _OUT ERROR_ID *errorID, _OUT STACKS *S);

/**********************************************************************************************
Function: creat_eye_matrix
Description: 创建单位矩阵
Input: 矩阵行数rows，列数columns
Output: 错误号指针errorID，栈指针S
Input_Output: 无
Return: 矩阵指针
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
MATRIX *creat_eye_matrix(_IN INTEGER n, _OUT ERROR_ID *errorID, _OUT STACKS *S);

/**********************************************************************************************
Function: matrix_add
Description: 矩阵A + 矩阵B = 矩阵C
Input: 矩阵A,矩阵B
Output: 矩阵C
Input_Output: 无
Return: 错误号
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
ERROR_ID matrix_add(_IN MATRIX *A, _IN MATRIX *B, _OUT MATRIX *C);

/**********************************************************************************************
Function: matrix_subtraction
Description: 矩阵A - 矩阵B = 矩阵C
Input: 矩阵A,矩阵B
Output: 矩阵C
Input_Output: 无
Return: 错误号
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
ERROR_ID matrix_subtraction(_IN MATRIX *A, _IN MATRIX *B, _OUT MATRIX *C);

/**********************************************************************************************
Function: matrix_multiplication
Description: 矩阵乘法C = A * B
Input: 矩阵A,矩阵B
Output: 矩阵C
Input_Output: 无
Return: 错误号
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
ERROR_ID matrix_multiplication(_IN MATRIX *A, _IN MATRIX *B, _OUT MATRIX *C);

/**********************************************************************************************
Function: matrix_inverse
Description: 矩阵求逆
Input: 矩阵A
Output: 矩阵A的逆矩阵
Input_Output: 无
Return: 错误号
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
ERROR_ID matrix_inverse(_IN MATRIX *A, _OUT MATRIX *invA);

/**********************************************************************************************
Function: matrix_transpose
Description: 矩阵转置
Input: 矩阵A
Output: 矩阵A的转置
Input_Output: 无
Return: 错误号
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
ERROR_ID matrix_transpose(_IN MATRIX *A, _OUT MATRIX *transposeA);

/**********************************************************************************************
Function: matrix_trace
Description: 矩阵的迹
Input: 矩阵A
Output: 矩阵A的迹
Input_Output: 无
Return: 错误号
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
ERROR_ID matrix_trace(_IN MATRIX *A, _OUT REAL *trace);

/**********************************************************************************************
Function: lup_decomposition
Description: n行n列矩阵LUP分解PA = L * U
Input: n行n列矩阵A
Output: n行n列下三角矩阵L，n行n列上三角矩阵U，n行n列置换矩阵P
Input_Output: 无
Return: 错误号
Author: Marc Pony(marc_pony@163.com)
参考：https://zhuanlan.zhihu.com/p/84210687
***********************************************************************************************/
ERROR_ID lup_decomposition(_IN MATRIX *A, _OUT MATRIX *L, _OUT MATRIX *U, _OUT MATRIX *P);

/**********************************************************************************************
Function: solve_matrix_equation_by_lup_decomposition
Description: LUP分解解矩阵方程AX=B,其中A为n行n列矩阵，B为n行m列矩阵，X为n行m列待求矩阵(写到矩阵B)
Input: n行n列矩阵A
Output: 无
Input_Output: n行m列矩阵B(即n行m列待求矩阵X)
Return: 错误号
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
ERROR_ID solve_matrix_equation_by_lup_decomposition(_IN MATRIX *A, _IN_OUT MATRIX *B);

#endif
