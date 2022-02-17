#ifndef TYPES_H_
#define TYPES_H_

typedef int8_t(*PTRFX_retINT8_T)(void);
typedef uint8_t(*PTRFX_retUINT8_T)(void);
typedef uint16_t(*PTRFX_retUINT16_T)(void);
typedef void (*PTRFX_retVOID)(void);
typedef void (*PTRFX_retVOID_arg1_UINT8_T)(uint8_t);
typedef void (*PTRFX_retVOID_arg1_INT8_T)(int8_t);
typedef void (*PTRFX_retVOID_arg1_PCHAR)(char *);
typedef unsigned char BOOL;


#endif // TYPES_H_
