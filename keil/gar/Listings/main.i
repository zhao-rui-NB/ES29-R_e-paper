#line 1 "..\\USER\\main.cpp"
#line 1 "C:\\Keil\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "C:\\Keil\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdio.h"



 

#line 2 "..\\USER\\main.cpp"
#line 1 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
 









 






































 















 

 
 
 



 



 
typedef enum IRQn
{
     

    NonMaskableInt_IRQn   = -14,     
    HardFault_IRQn        = -13,     
    SVCall_IRQn           = -5,      
    PendSV_IRQn           = -2,      
    SysTick_IRQn          = -1,      

     
    BOD_IRQn              = 0,       
    WDT_IRQn              = 1,       
    EINT0_IRQn            = 2,       
    EINT1_IRQn            = 3,       
    GPABC_IRQn            = 4,       
    GPDEF_IRQn            = 5,       
    PWM0_IRQn             = 6,       
    PWM1_IRQn             = 7,       
    TMR0_IRQn             = 8,       
    TMR1_IRQn             = 9,       
    TMR2_IRQn             = 10,      
    TMR3_IRQn             = 11,      
    UART0_IRQn            = 12,      
    UART1_IRQn            = 13,      
    SPI0_IRQn             = 14,      
    SPI1_IRQn             = 15,      
    SPI2_IRQn             = 16,      
    HIRC_IRQn             = 17,      
    I2C0_IRQn             = 18,      
    I2C1_IRQn             = 19,      
    SC2_IRQn              = 20,      
    SC0_IRQn              = 21,      
    SC1_IRQn              = 22,      
    USBD_IRQn             = 23,      
    LCD_IRQn              = 25,      
    PDMA_IRQn             = 26,      
    I2S_IRQn              = 27,      
    PDWU_IRQn             = 28,      
    ADC_IRQn              = 29,      
    DAC_IRQn              = 30,      
    RTC_IRQn              = 31       
} IRQn_Type;






 

 






   


#line 1 "..\\Library\\CMSIS\\Include\\core_cm0.h"
 




 

























 











#line 1 "C:\\Keil\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdint.h"






 
#line 45 "..\\Library\\CMSIS\\Include\\core_cm0.h"

















 




 



 

 













#line 120 "..\\Library\\CMSIS\\Include\\core_cm0.h"



 







#line 162 "..\\Library\\CMSIS\\Include\\core_cm0.h"

#line 1 "..\\Library\\CMSIS\\Include\\core_cmInstr.h"
 




 

























 












 



 

 
#line 1 "..\\Library\\CMSIS\\Include\\cmsis_armcc.h"
 




 

























 










 



 

 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 263 "..\\Library\\CMSIS\\Include\\cmsis_armcc.h"


#line 297 "..\\Library\\CMSIS\\Include\\cmsis_armcc.h"



 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 



__attribute__((always_inline)) static __inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;
  int32_t s = 4   * 8 - 1;  

  result = value;                       
  for (value >>= 1U; value; value >>= 1U)
  {
    result <<= 1U;
    result |= value & 1U;
    s--;
  }
  result <<= s;                         
  return(result);
}








 



#line 649 "..\\Library\\CMSIS\\Include\\cmsis_armcc.h"

   


 



 

#line 731 "..\\Library\\CMSIS\\Include\\cmsis_armcc.h"
 


#line 54 "..\\Library\\CMSIS\\Include\\core_cmInstr.h"

 
#line 84 "..\\Library\\CMSIS\\Include\\core_cmInstr.h"

   

#line 164 "..\\Library\\CMSIS\\Include\\core_cm0.h"
#line 1 "..\\Library\\CMSIS\\Include\\core_cmFunc.h"
 




 

























 












 



 

 
#line 54 "..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 
#line 84 "..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 

#line 165 "..\\Library\\CMSIS\\Include\\core_cm0.h"
















 
#line 198 "..\\Library\\CMSIS\\Include\\core_cm0.h"

 






 
#line 214 "..\\Library\\CMSIS\\Include\\core_cm0.h"

 




 










 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:28;               
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:15;               
    uint32_t T:1;                         
    uint32_t _reserved1:3;                
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t _reserved0:1;                
    uint32_t SPSEL:1;                     
    uint32_t _reserved1:30;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 



 







 



 
typedef struct
{
  volatile uint32_t ISER[1U];                
        uint32_t RESERVED0[31U];
  volatile uint32_t ICER[1U];                
        uint32_t RSERVED1[31U];
  volatile uint32_t ISPR[1U];                
        uint32_t RESERVED2[31U];
  volatile uint32_t ICPR[1U];                
        uint32_t RESERVED3[31U];
        uint32_t RESERVED4[64U];
  volatile uint32_t IP[8U];                  
}  NVIC_Type;

 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
        uint32_t RESERVED0;
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
        uint32_t RESERVED1;
  volatile uint32_t SHP[2U];                 
  volatile uint32_t SHCSR;                   
} SCB_Type;

 















 



























 















 









 






 



 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 








 
 







 






 







 


 







 

 










 









 


 



 





 

 
 









 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] = ((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )]  = ((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )]  & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
  else
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FAUL << 16U) |
                 (1UL << 2U));
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 2) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 










#line 141 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\system_Nano100Series.h"
 









 












 

#line 32 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\system_Nano100Series.h"


extern uint32_t SystemCoreClock;         
extern uint32_t CyclesPerUs;             









 

extern void SystemCoreClockUpdate (void);
extern uint32_t SysGet_PLLClockFreq(void);








 
#line 142 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 143 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"

 
 
 



 


#pragma anon_unions




 



 

typedef struct
{


    












 
    volatile const  uint32_t RESULT[18];


    
















































































 
    volatile uint32_t CR;

    





























































 
    volatile uint32_t CHEN;

    



























 
    volatile uint32_t CMPR0;

    



























 
    volatile uint32_t CMPR1;

    







































 
    volatile uint32_t SR;
    uint32_t RESERVE0[1];


    









 
    volatile const  uint32_t PDMA;

    



















 
    volatile uint32_t PWRCTL;

    




















 
    volatile uint32_t CALCTL;

    










 
    volatile uint32_t CALWORD;

    

































 
    volatile uint32_t SMPLCNT0;

    
















 
    volatile uint32_t SMPLCNT1;

} ADC_T;




 






























































































































   
   


 



 

typedef struct
{


    





































































 
    volatile uint32_t PWRCTL;

    
























 
    volatile uint32_t AHBCLK;

    























































































 
    volatile uint32_t APBCLK;

    

























 
    volatile const  uint32_t CLKSTATUS;

    


















 
    volatile uint32_t CLKSEL0;

    













































 
    volatile uint32_t CLKSEL1;

    

























































 
    volatile uint32_t CLKSEL2;

    


















 
    volatile uint32_t CLKDIV0;

    










 
    volatile uint32_t CLKDIV1;

    




















 
    volatile uint32_t PLLCTL;

    













 
    volatile uint32_t FRQDIV;

    







































 
    volatile uint32_t MCLKO;

    










 
    volatile  uint32_t WK_INTSTS;

} CLK_T;




 
































































































































































































































































   
   


 



 

typedef struct
{


    

























 
    volatile uint32_t CTL0;

    







 
    volatile uint32_t DATA0;

    

















 
    volatile uint32_t STS0;
    uint32_t RESERVE0[1];


    

























 
    volatile uint32_t CTL1;

    







 
    volatile uint32_t DATA1;

    

















 
    volatile uint32_t STS1;
    uint32_t RESERVE1[1];


    

















 
    volatile uint32_t COMCTL;

} DAC_T;




 


































   
   


 



 

typedef struct
{


    


































 
    volatile uint32_t EBICON;

    





















 
    volatile uint32_t EXTIME;

} EBI_T;




 































   
   


 



 

typedef struct
{


    


































 
    volatile uint32_t ISPCON;

    









 
    volatile uint32_t ISPADR;

    









 
    volatile uint32_t ISPDAT;

    












 
    volatile uint32_t ISPCMD;

    










 
    volatile uint32_t ISPTRG;

    










 
    volatile const  uint32_t DFBADR;
    uint32_t RESERVE0[10];


    


















 
    volatile uint32_t ISPSTA;

} FMC_T;




 

















































   
   


 



 

typedef struct
{


    









 
    volatile const  uint32_t PDID;

    






























 
    volatile uint32_t RST_SRC;

    


































 
    volatile uint32_t IPRST_CTL1;

    








































































 
    volatile uint32_t IPRST_CTL2;
    uint32_t RESERVE0[4];


    









 
    volatile uint32_t TEMPCTL;
    uint32_t RESERVE1[3];


    






















































 
    volatile uint32_t PA_L_MFP;

    

























































 
    volatile uint32_t PA_H_MFP;

    




















































 
    volatile uint32_t PB_L_MFP;

    























































 
    volatile uint32_t PB_H_MFP;

    



















































 
    volatile uint32_t PC_L_MFP;

    















































 
    volatile uint32_t PC_H_MFP;

    

















































 
    volatile uint32_t PD_L_MFP;

    






























 
    volatile uint32_t PD_H_MFP;

    































 
    volatile uint32_t PE_L_MFP;

    






























 
    volatile uint32_t PE_H_MFP;

    



























 
    volatile uint32_t PF_L_MFP;
    uint32_t RESERVE2[1];


    










 
    volatile uint32_t PORCTL;

    






















































 
    volatile uint32_t BODCTL;

    




























 
    volatile  uint32_t BODSTS;

    




























 
    volatile uint32_t Int_VREFCTL;
    uint32_t RESERVE3[4];


    



































 
    volatile uint32_t IRCTRIMCTL;

    
















 
    volatile uint32_t IRCTRIMIEN;

    























 
    volatile uint32_t IRCTRIMINT;
    uint32_t RESERVE4[29];


    









 
    volatile uint32_t RegLockAddr;

} SYS_T;




 






































































































































































































































































































































































































































































   
   


 



 

typedef struct
{


    






















































































































 
    volatile uint32_t PMD;

    











 
    volatile uint32_t OFFD;

    











 
    volatile uint32_t DOUT;

    















 
    volatile uint32_t DMASK;

    









 
    volatile const  uint32_t PIN;

    















 
    volatile uint32_t DBEN;

    

















 
    volatile uint32_t IMD;

    






































































































































































































































































































 
    volatile uint32_t IER;

    














 
    volatile uint32_t ISRC;

    












 
    volatile uint32_t PUEN;

} GPIO_T;


typedef struct
{
    

































 
    volatile uint32_t DBNCECON;
} GP_DB_T;




 








































































































































































   




 










   
   


 



 

typedef struct
{


    

































 
    volatile uint32_t CON;

    












 
    volatile uint32_t INTSTS;

    


















 
    volatile const  uint32_t STATUS;

    









 
    volatile uint32_t DIV;

    















 
    volatile uint32_t TOUT;

    
















 
    volatile uint32_t DATA;

    

















 
    volatile uint32_t SADDR0;

    

















 
    volatile uint32_t SADDR1;
    uint32_t RESERVE0[2];


    












 
    volatile uint32_t SAMASK0;

    












 
    volatile uint32_t SAMASK1;
    uint32_t RESERVE1[4];


    









 
    volatile uint32_t WKUPCON;

    










 
    volatile  uint32_t WKUPSTS;

} I2C_T;




 
































































   
   


 



 

typedef struct
{


    
























































































 
    volatile uint32_t CTRL;

    















 
    volatile uint32_t CLKDIV;

    






































 
    volatile uint32_t INTEN;

    

































































































 
    volatile uint32_t STATUS;

    











 
    volatile  uint32_t TXFIFO;

    











 
    volatile const  uint32_t RXFIFO;

} I2S_T;




 




















































































































































   
   


 



 

typedef struct
{


    








 
    volatile const  uint32_t IRQSRC[32];


    








 
    volatile uint32_t NMI_SEL;

    















 
    volatile uint32_t MCU_IRQ;

} INTR_T;




 










   
   


 



 

typedef struct
{


    









































 
    volatile uint32_t CTL;

    








































 
    volatile uint32_t DISPCTL;

    














 
    volatile uint32_t MEM_0;

    














 
    volatile uint32_t MEM_1;

    














 
    volatile uint32_t MEM_2;

    














 
    volatile uint32_t MEM_3;

    














 
    volatile uint32_t MEM_4;

    














 
    volatile uint32_t MEM_5;

    














 
    volatile uint32_t MEM_6;

    














 
    volatile uint32_t MEM_7;

    














 
    volatile uint32_t MEM_8;

    














 
    volatile uint32_t MEM_9;

    



















 
    volatile uint32_t FCR;

    













 
    volatile uint32_t FCSTS;

} LCD_T;




 















































































































































































   
   


 



 


typedef struct
{


    



















































 
    volatile uint32_t CTL;

    









 
    volatile uint32_t DMASAR;
    uint32_t RESERVE0[1];


    








 
    volatile uint32_t DMABCR;
    uint32_t RESERVE1[1];


    








 
    volatile const  uint32_t DMACSAR;
    uint32_t RESERVE2[1];


    









 
    volatile const  uint32_t DMACBCR;

    












 
    volatile uint32_t DMAIER;

    



















 
    volatile uint32_t DMAISR;
    uint32_t RESERVE3[22];


    











 
    volatile uint32_t WDATA;

    








 
    volatile uint32_t SEED;

    








 
    volatile const  uint32_t CHECKSUM;

} DMA_CRC_T;


typedef struct
{


    






























 
    volatile uint32_t GCRCSR;

    












































 
    volatile uint32_t DSSR0;

    





















 
    volatile uint32_t DSSR1;

    






























 
    volatile const  uint32_t GCRISR;

} DMA_GCR_T;


typedef struct
{
    


















































 
    volatile uint32_t CSR;

    









 
    volatile uint32_t SAR;

    









 
    volatile uint32_t DAR;

    









 
    volatile uint32_t BCR;
    uint32_t RESERVE0[1];


    








 
    volatile const  uint32_t CSAR;

    








 
    volatile const  uint32_t CDAR;

    









 
    volatile const  uint32_t CBCR;

    


















 
    volatile uint32_t IER;

    





























 
    volatile uint32_t ISR;

    











 
    volatile uint32_t TCR;

} PDMA_T;



typedef struct
{


    



























 
    volatile uint32_t CSR;

    








 
    volatile uint32_t SAR;

    








 
    volatile uint32_t DAR;

    









 
    volatile uint32_t BCR;
    uint32_t RESERVE0[1];


    








 
    volatile const  uint32_t CSAR;

    








 
    volatile const  uint32_t CDAR;

    








 
    volatile const  uint32_t CBCR;

    












 
    volatile uint32_t IER;

    


















 
    volatile uint32_t ISR;
    uint32_t RESERVE1[1];


    










 
    volatile uint32_t SASOCR;

    








 
    volatile uint32_t DASOCR;

} VDMA_T;





 





























































   





 



































































   





 






































































   





 























































   

   


 



 

typedef struct
{


    


















 
    volatile uint32_t PRES;

    






















 
    volatile uint32_t CLKSEL;

    






















































 
    volatile uint32_t CTL;

    


















 
    volatile uint32_t INTEN;

    


































 
    volatile uint32_t INTSTS;

    






















 
    volatile uint32_t OE;
    uint32_t RESERVE0[1];


    


























 
    volatile uint32_t DUTY0;

    















 
    volatile const  uint32_t DATA0;
    uint32_t RESERVE1[1];


    


























 
    volatile uint32_t DUTY1;

    















 
    volatile const  uint32_t DATA1;
    uint32_t RESERVE2[1];


    


























 
    volatile uint32_t DUTY2;

    















 
    volatile const  uint32_t DATA2;
    uint32_t RESERVE3[1];


    
























 
    volatile uint32_t DUTY3;

    















 
    volatile const  uint32_t DATA3;
    uint32_t RESERVE4[3];


    




































































































 
    volatile uint32_t CAPCTL;

    






































 
    volatile uint32_t CAPINTEN;

    


























































 
    volatile uint32_t CAPINTSTS;

    










 
    volatile const  uint32_t CRL0;

    










 
    volatile const  uint32_t CFL0;

    










 
    volatile const  uint32_t CRL1;

    










 
    volatile const  uint32_t CFL1;

    










 
    volatile const  uint32_t CRL2;

    










 
    volatile const  uint32_t CFL2;

    










 
    volatile const  uint32_t CRL3;

    










 
    volatile const  uint32_t CFL3;

    


















 
    volatile const  uint32_t PDMACH0;

    


















 
    volatile const  uint32_t PDMACH2;

} PWM_T;




 

























































































































































































































































































































































































































   
   


 



 

typedef struct
{


    














 
    volatile  uint32_t INIR;

    












 
    volatile  uint32_t AER;

    


























 
    volatile uint32_t FCR;

    












 
    volatile uint32_t TLR;

    












 
    volatile uint32_t CLR;

    










 
    volatile uint32_t TSSR;

    














 
    volatile uint32_t DWR;

    












 
    volatile uint32_t TAR;

    












 
    volatile uint32_t CAR;

    









 
    volatile const  uint32_t LIR;

    















 
    volatile uint32_t RIER;

    
























 
    volatile uint32_t RIIR;

    






















 
    volatile uint32_t TTR;
    uint32_t RESERVE0[2];


    























 
    volatile uint32_t SPRCTL;

    









 
    volatile uint32_t SPR[20];

} RTC_T;




 

































































































































































































   
   


 



 

typedef struct
{


    union
    {
        








 
        volatile const  uint32_t  RBR;
        








 
        volatile  uint32_t  THR;
    };

    



















































































 
    volatile uint32_t CTL;

    






















































































 
    volatile uint32_t ALTCTL;

    









 
    volatile uint32_t EGTR;

    











 
    volatile uint32_t RFTMR;

    
















 
    volatile uint32_t ETUCR;

    




















































 
    volatile uint32_t IER;

    
















































 
    volatile uint32_t ISR;

    







































































 
    volatile uint32_t TRSR;

    








































































 
    volatile uint32_t PINCSR;

    










 
    volatile uint32_t TMR0;

    










 
    volatile uint32_t TMR1;

    










 
    volatile uint32_t TMR2;

    


























 
    volatile uint32_t UACTL;

    








 
    volatile const  uint32_t TDRA;

    










 
    volatile const  uint32_t TDRB;

} SC_T;




 

































































































































































































































































































   
   


 



 

typedef struct
{


    































































































 
    volatile uint32_t CTL;

    



















































 
    volatile uint32_t STATUS;

    














 
    volatile uint32_t CLKDIV;

    

















































 
    volatile uint32_t SSR;

    










 
    volatile const  uint32_t RX0;

    










 
    volatile const  uint32_t RX1;
    uint32_t RESERVE0[2];


    












 
    volatile  uint32_t TX0;

    












 
    volatile  uint32_t TX1;
    uint32_t RESERVE1[3];


    











 
    volatile uint32_t VARCLK;

    


























 
    volatile uint32_t DMA;

    
































 
    volatile uint32_t FFCTL;
} SPI_T;




 






































































































































































   
   


 



 

typedef struct
{


    






































































































































 
    volatile uint32_t CTL;

    









 
    volatile uint32_t PRECNT;

    













 
    volatile uint32_t CMPR;

    














 
    volatile uint32_t IER;

    



























 
    volatile uint32_t ISR;

    








 
    volatile const  uint32_t DR;

    









 
    volatile const  uint32_t TCAP;
} TIMER_T;





 
























































































   


   



 



 

typedef struct
{


    union
    {

        








 
        volatile const  uint32_t  RBR;


        








 
        volatile  uint32_t  THR;
    };

    






















































 
    volatile uint32_t CTL;

    










































 
    volatile uint32_t TLCTL;

    

































 
    volatile uint32_t IER;

    












































 
    volatile  uint32_t ISR;

    
















































 
    volatile  uint32_t TRSR;

    















































 
    volatile  uint32_t FSR;

    























 
    volatile uint32_t MCSR;

    



















 
    volatile uint32_t TMCTL;

    












 
    volatile uint32_t BAUD;
    uint32_t RESERVE0[2];


    
















 
    volatile uint32_t IRCR;

    

















































 
    volatile uint32_t ALT_CTL;

    











 
    volatile uint32_t FUN_SEL;

} UART_T;




 

















































































































































































































































   
   


 



 



 
typedef struct
{


    








 
    volatile uint32_t BUFSEG;

    
















 
    volatile uint32_t MXPLD;

    

































 
    volatile uint32_t CFG;
    uint32_t RESERVE;

} USBD_EP_T;

typedef struct
{


    





























 
    volatile uint32_t CTL;

    

















 
    volatile const  uint32_t BUSSTS;

    


















 
    volatile uint32_t INTEN;

    

















































 
    volatile uint32_t INTSTS;

    







 
    volatile uint32_t FADDR;

    

































 
    volatile const  uint32_t EPSTS;

    









 
    volatile uint32_t BUFSEG;

    












 
    volatile const  uint32_t EPSTS2;


    USBD_EP_T EP[8];

    uint32_t RESERVE0;

    





















 
    volatile uint32_t PDMA;

} USBD_T;




 
































































































































































   
   


 



 

typedef struct
{


    






































 
    volatile uint32_t CTL;

    









 
    volatile uint32_t IER;

    



























 
    volatile uint32_t ISR;

} WDT_T;




 































   
   


 



 

typedef struct
{


    










 
    volatile  uint32_t RLD;

    




















 
    volatile uint32_t CR;

    










 
    volatile uint32_t IER;

    












 
    volatile uint32_t STS;

    








 
    volatile const  uint32_t VAL;

} WWDT_T;




 




























   
   





#pragma no_anon_unions





 
 






 

#line 11622 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"

#line 11635 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"

#line 11658 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"

   





 
#line 11691 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"

#line 11713 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"

   

   




 

typedef volatile unsigned char  vu8;        
typedef volatile unsigned short vu16;       
typedef volatile unsigned long  vu32;       





 







 







 








 







 








 







 







 






 








 







 








 







 







 






 


   

 
 
 



 











 
#line 11901 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"

 










   

   






 
 
 
#line 1 "..\\Library\\StdDriver\\inc\\sys.h"
 









 










 



 



 

 
 
 
#line 59 "..\\Library\\StdDriver\\inc\\sys.h"

 
 
 

 





 














 




 




 

#line 104 "..\\Library\\StdDriver\\inc\\sys.h"

#line 113 "..\\Library\\StdDriver\\inc\\sys.h"

#line 121 "..\\Library\\StdDriver\\inc\\sys.h"

#line 129 "..\\Library\\StdDriver\\inc\\sys.h"





















 
#line 158 "..\\Library\\StdDriver\\inc\\sys.h"

#line 165 "..\\Library\\StdDriver\\inc\\sys.h"

#line 172 "..\\Library\\StdDriver\\inc\\sys.h"

#line 179 "..\\Library\\StdDriver\\inc\\sys.h"

#line 187 "..\\Library\\StdDriver\\inc\\sys.h"

#line 195 "..\\Library\\StdDriver\\inc\\sys.h"

#line 202 "..\\Library\\StdDriver\\inc\\sys.h"

#line 209 "..\\Library\\StdDriver\\inc\\sys.h"

 
#line 217 "..\\Library\\StdDriver\\inc\\sys.h"

#line 224 "..\\Library\\StdDriver\\inc\\sys.h"

#line 231 "..\\Library\\StdDriver\\inc\\sys.h"

#line 238 "..\\Library\\StdDriver\\inc\\sys.h"

#line 245 "..\\Library\\StdDriver\\inc\\sys.h"

#line 252 "..\\Library\\StdDriver\\inc\\sys.h"













 
#line 272 "..\\Library\\StdDriver\\inc\\sys.h"

#line 279 "..\\Library\\StdDriver\\inc\\sys.h"












#line 298 "..\\Library\\StdDriver\\inc\\sys.h"

#line 306 "..\\Library\\StdDriver\\inc\\sys.h"

#line 314 "..\\Library\\StdDriver\\inc\\sys.h"

#line 323 "..\\Library\\StdDriver\\inc\\sys.h"

 
#line 331 "..\\Library\\StdDriver\\inc\\sys.h"

#line 338 "..\\Library\\StdDriver\\inc\\sys.h"

































 
#line 378 "..\\Library\\StdDriver\\inc\\sys.h"







#line 391 "..\\Library\\StdDriver\\inc\\sys.h"




























 

















#line 443 "..\\Library\\StdDriver\\inc\\sys.h"

#line 450 "..\\Library\\StdDriver\\inc\\sys.h"













 
























 

























 
























 





















   



 






 







 







 







 







 







 







 








 









 









 









 







 







 







 







 







 







 








 









 








 








 









 









 







 







 














 









 









 









 
static __inline void SYS_UnlockReg(void)
{
    while(((SYS_T *) (((uint32_t)0x50000000) + 0x00000))->RegLockAddr != (0x1ul << (0)))
    {
        ((SYS_T *) (((uint32_t)0x50000000) + 0x00000))->RegLockAddr = 0x59;
        ((SYS_T *) (((uint32_t)0x50000000) + 0x00000))->RegLockAddr = 0x16;
        ((SYS_T *) (((uint32_t)0x50000000) + 0x00000))->RegLockAddr = 0x88;
    }
}







 
static __inline void SYS_LockReg(void)
{
    ((SYS_T *) (((uint32_t)0x50000000) + 0x00000))->RegLockAddr = 0;
}

void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t  SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);
void SYS_EnableIRCTrim(uint32_t u32TrimSel,uint32_t u32TrimEnInt);
void SYS_DisableIRCTrim(void);
   

   

   







 


#line 11926 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\clk.h"
 









 











 



 



 


#line 41 "..\\Library\\StdDriver\\inc\\clk.h"

 
#line 53 "..\\Library\\StdDriver\\inc\\clk.h"


 
#line 62 "..\\Library\\StdDriver\\inc\\clk.h"

 
#line 90 "..\\Library\\StdDriver\\inc\\clk.h"

 
#line 98 "..\\Library\\StdDriver\\inc\\clk.h"


 






 


































 












































 
#line 194 "..\\Library\\StdDriver\\inc\\clk.h"

 



 



 














#line 232 "..\\Library\\StdDriver\\inc\\clk.h"

 


 


 
#line 267 "..\\Library\\StdDriver\\inc\\clk.h"


 
 
 
#line 282 "..\\Library\\StdDriver\\inc\\clk.h"

#line 291 "..\\Library\\StdDriver\\inc\\clk.h"
 
 
 
#line 300 "..\\Library\\StdDriver\\inc\\clk.h"

#line 327 "..\\Library\\StdDriver\\inc\\clk.h"
   




 
void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_GetPLLClockFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
int32_t CLK_SysTickDelay(uint32_t us);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);

   

   

   







 
#line 11927 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\adc.h"
 









 











 



 



 

#line 77 "..\\Library\\StdDriver\\inc\\adc.h"

   




 







 











 











 











 










 










 







 







 




















 
#line 196 "..\\Library\\StdDriver\\inc\\adc.h"






 
















 
#line 229 "..\\Library\\StdDriver\\inc\\adc.h"






 








 







 







 










 












 











 









 









 







 







 


void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask);
void ADC_Close(ADC_T *adc);
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param);
void ADC_DisableHWTrigger(ADC_T *adc);
void ADC_EnableTimerTrigger(ADC_T *adc,
                            uint32_t u32Source,
                            uint32_t u32PDMACnt);
void ADC_DisableTimerTrigger(ADC_T *adc);
void ADC_SetExtraSampleTime(ADC_T *adc,
                            uint32_t u32ChNum,
                            uint32_t u32SampleTime);
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);



   

   

   







 
#line 11928 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\dac.h"
 









 











 



 




 
#line 39 "..\\Library\\StdDriver\\inc\\dac.h"





   





 








 
#line 67 "..\\Library\\StdDriver\\inc\\dac.h"







 







 











 











 








 
#line 122 "..\\Library\\StdDriver\\inc\\dac.h"











 








 
#line 149 "..\\Library\\StdDriver\\inc\\dac.h"







 
#line 163 "..\\Library\\StdDriver\\inc\\dac.h"

void DAC_Open(DAC_T *dac, uint32_t u32Ch, uint32_t u32TrgSrc);
void DAC_Close(DAC_T *dac, uint32_t u32Ch);
int DAC_SetDelayTime(DAC_T *dac, uint32_t u32Delay);

   

   

   







 
#line 11929 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\fmc.h"
 










 










 



 




 


 
 
 














 
 
 
#line 62 "..\\Library\\StdDriver\\inc\\fmc.h"





   




 

 
 
 






 







 







 







 







 







 







 







 







 







 







 







 


 
 
 

extern void FMC_Close(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_GetBootSource(void);
extern void FMC_Open(void);
extern uint32_t FMC_Read(uint32_t u32Addr);
extern uint32_t FMC_ReadCID(void);
extern uint32_t FMC_ReadPID(void);
extern uint32_t FMC_ReadUCID(uint32_t u32Index);
extern uint32_t FMC_ReadUID(uint32_t u32Index);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void FMC_SetVectorPageAddr(uint32_t u32PageAddr);
extern uint32_t FMC_GetVectorPageAddr(void);
extern int32_t FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);


   

   

   







 
#line 11930 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\ebi.h"
 










 










 



 




 


 
 
 




 



 
 
 
#line 55 "..\\Library\\StdDriver\\inc\\ebi.h"

 
 
 
#line 66 "..\\Library\\StdDriver\\inc\\ebi.h"


   




 

 
 
 






 








 







 








 







 








 


 
 
 

void EBI_Open(uint32_t u32Bank, uint32_t u32DataWidth, uint32_t u32TimingClass, uint32_t u32BusMode, uint32_t u32CSActiveLevel);
void EBI_Close(uint8_t u32Bank);
void EBI_SetBusTiming(uint32_t u32Bank, uint32_t u32TimingConfig, uint32_t u32MclkDiv);


   

   

   







 
#line 11931 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\gpio.h"
 









 











 



 



 


 
 
 




 
 
 






 
 
 



 
 
 






#line 81 "..\\Library\\StdDriver\\inc\\gpio.h"














 
#line 113 "..\\Library\\StdDriver\\inc\\gpio.h"

#line 130 "..\\Library\\StdDriver\\inc\\gpio.h"

#line 147 "..\\Library\\StdDriver\\inc\\gpio.h"

#line 164 "..\\Library\\StdDriver\\inc\\gpio.h"

#line 181 "..\\Library\\StdDriver\\inc\\gpio.h"

#line 188 "..\\Library\\StdDriver\\inc\\gpio.h"

   



 










 











 











 











 











 











 











 












 



















 










 











 











 











 










 













 












 














 












 



void GPIO_SetMode(GPIO_T *gpio, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *gpio, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *gpio, uint32_t u32Pin);



   

   

   







 
#line 11932 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\i2c.h"
 









 











 



 



 









   




 







 







 








 
static __inline int32_t I2C_STOP(I2C_T *i2c)
{
    int32_t tout = (SystemCoreClock / 10);

    i2c->CON |= ((0x1ul << (4)) | (0x1ul << (2)));
    while((i2c->CON & (0x1ul << (2))) && (tout-- > 0));
    if (i2c->CON & (0x1ul << (2)))
        return -1;
    return 0;
}







 
static __inline int32_t I2C_WAIT_READY(I2C_T *i2c)
{
    int32_t tout = (SystemCoreClock / 10);

    while(!(i2c->INTSTS & (0x1ul << (0))) && (tout-- > 0));
    if (!(i2c->INTSTS & (0x1ul << (0))))
        return -1;
    i2c->INTSTS |= (0x1ul << (0));
    return 0;
}






 








 







 









 







 









 







 


uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
void I2C_Close(I2C_T *i2c);
void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetBusClockFreq(I2C_T *i2c);
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
void I2C_ClearIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_GetData(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);
void I2C_EnableWakeup(I2C_T *i2c);
void I2C_DisableWakeup(I2C_T *i2c);

   

   

   







 
#line 11933 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\crc.h"
 









 











 



 



 

 
 
 





 
 
 





 
 
 





   



 









 










 










 










 










 










 










 



 
void CRC_Open(uint32_t u32Mode, uint32_t u32Attribute, uint32_t u32Seed, uint32_t u32DataLen);
void CRC_StartDMATransfer(uint32_t u32SrcAddr, uint32_t u32ByteCount);
uint32_t CRC_GetChecksum(void);


   

   

   







 
#line 11934 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\pdma.h"
 









 











 



 



 

 
 
 




 
 
 
#line 49 "..\\Library\\StdDriver\\inc\\pdma.h"

 
 
 
#line 66 "..\\Library\\StdDriver\\inc\\pdma.h"

#line 80 "..\\Library\\StdDriver\\inc\\pdma.h"



   



 










 











 












 












 












 












 












 
#line 185 "..\\Library\\StdDriver\\inc\\pdma.h"










 


void PDMA_Open(uint32_t u32Mask);
void PDMA_Close(void);
void PDMA_SetTransferCnt(uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void PDMA_SetTransferAddr(uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Periphral, uint32_t u32ScatterEn, uint32_t u32DescAddr);
void PDMA_SetTimeOut(uint32_t u32Ch, uint32_t u32OnOff, uint32_t u32TimeOutCnt);
void PDMA_Trigger(uint32_t u32Ch);
void PDMA_EnableInt(uint32_t u32Ch, uint32_t u32Mask);
void PDMA_DisableInt(uint32_t u32Ch, uint32_t u32Mask);

   

   

   







 
#line 11935 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\pwm.h"
 









 











 



 



 
#line 59 "..\\Library\\StdDriver\\inc\\pwm.h"

   




 








 
#line 85 "..\\Library\\StdDriver\\inc\\pwm.h"








 








 











 















 











 















 






uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                 uint32_t u32ChannelNum,
                                 uint32_t u32Frequency,
                                 uint32_t u32DutyCycle);
uint32_t PWM_ConfigCaptureChannel (PWM_T *pwm,
                                   uint32_t u32ChannelNum,
                                   uint32_t u32UnitTimeNsec,
                                   uint32_t u32CaptureEdge);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_DisableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_ClearCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t PWM_GetCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnablePDMA(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32RisingFirst, uint32_t u32Mode);
void PWM_DisablePDMA(PWM_T *pwm, uint32_t u32ChannelNum);

   

   

   







 
#line 11936 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\rtc.h"
 









 











 



 




 
















#line 57 "..\\Library\\StdDriver\\inc\\rtc.h"

#line 65 "..\\Library\\StdDriver\\inc\\rtc.h"






   




 



 
typedef struct
{
    uint32_t u32Year;           
    uint32_t u32Month;          
    uint32_t u32Day;            
    uint32_t u32DayOfWeek;      
    uint32_t u32Hour;           
    uint32_t u32Minute;         
    uint32_t u32Second;         
    uint32_t u32TimeScale;      
    uint32_t u32AmPm;           
} S_RTC_TIME_DATA_T;

   




 









 










 










 









 









 









 









 









 










 









 









 



void RTC_Open(S_RTC_TIME_DATA_T *sPt);
void RTC_Close(void);
void RTC_32KCalibration(int32_t i32FrequencyX100);
void RTC_SetTickPeriod(uint32_t u32TickSelection);
void RTC_EnableInt(uint32_t u32IntFlagMask);
void RTC_DisableInt(uint32_t u32IntFlagMask);
uint32_t RTC_GetDayOfWeek(void);
void RTC_DisableTamperDetection(void);
void RTC_EnableTamperDetection(uint32_t u32PinCondition);
void RTC_SetAlarmTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetAlarmDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day);
void RTC_SetTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day, uint32_t u32DayOfWeek);
void RTC_SetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_GetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_GetDateAndTime(S_RTC_TIME_DATA_T *sPt);



   


   

   









 



#line 11937 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\sc.h"
 









 











 



 



 
#line 38 "..\\Library\\StdDriver\\inc\\sc.h"

#line 49 "..\\Library\\StdDriver\\inc\\sc.h"


   




 


















 



















 








 
#line 118 "..\\Library\\StdDriver\\inc\\sc.h"








 
#line 140 "..\\Library\\StdDriver\\inc\\sc.h"







 
#line 161 "..\\Library\\StdDriver\\inc\\sc.h"







 
#line 182 "..\\Library\\StdDriver\\inc\\sc.h"






 








 









 







 
static __inline void SC_SetTxRetry(SC_T *sc, uint32_t u32Count)
{
    
    sc->CTL &= ~((0x7ul << (20)) | (0x1ul << (23)));

    if(u32Count != 0)
    {
        sc->CTL |= ((u32Count - 1) << (20)) | (0x1ul << (23));
    }
}






 
static __inline void  SC_SetRxRetry(SC_T *sc, uint32_t u32Count)
{
    
    sc->CTL &= ~((0x7ul << (16)) | (0x1ul << (19)));

    if(u32Count != 0)
    {
        sc->CTL |= ((u32Count - 1) << (16)) | (0x1ul << (19));
    }
}


uint32_t SC_IsCardInserted(SC_T *sc);
void SC_ClearFIFO(SC_T *sc);
void SC_Close(SC_T *sc);
void SC_Open(SC_T *sc, uint32_t u32CardDet, uint32_t u32PWR);
void SC_ResetReader(SC_T *sc);
void SC_SetBlockGuardTime(SC_T *sc, uint32_t u32BGT);
void SC_SetCharGuardTime(SC_T *sc, uint32_t u32CGT);
void SC_StopAllTimer(SC_T *sc);
void SC_StartTimer(SC_T *sc, uint32_t u32TimerNum, uint32_t u32Mode, uint32_t u32ETUCount);
void SC_StopTimer(SC_T *sc, uint32_t u32TimerNum);


   

   

   







 
#line 11938 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\scuart.h"
 









 











 



 



 













   




 

 






 









 









 








 









 









 



 






 









 










 










 









 


 











 












 














 











 










 











 


void SCUART_Close(SC_T* sc);
uint32_t SCUART_Open(SC_T* sc, uint32_t u32baudrate);
uint32_t SCUART_Read(SC_T* sc, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
uint32_t SCUART_SetLineConfig(SC_T* sc, uint32_t u32Baudrate, uint32_t u32DataWidth, uint32_t u32Parity, uint32_t  u32StopBits);
void SCUART_SetTimeoutCnt(SC_T* sc, uint32_t u32TOC);
void SCUART_Write(SC_T* sc,uint8_t *pu8TxBuf, uint32_t u32WriteBytes);

   

   

   







 
#line 11939 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\spi.h"
 









 











 



 




 

















#line 56 "..\\Library\\StdDriver\\inc\\spi.h"


   




 






 







 







 







 







 







 









 









 









 







 






 








 








 








 








 








 








 










 







 







 








 







 







 








 
static __inline void SPI_SET_DATA_WIDTH(SPI_T *spi, uint32_t u32Width)
{
    if(u32Width == 32)
        u32Width = 0;

    spi->CTL = (spi->CTL & ~(0x1ful << (3))) | (u32Width << (3));
}








 







 







 







 







 







 







 







 







 







 


uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_ClearRxFIFO(SPI_T *spi);
void SPI_ClearTxFIFO(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void SPI_EnableFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
void SPI_DisableFIFO(SPI_T *spi);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_EnableWakeup(SPI_T *spi);
void SPI_DisableWakeup(SPI_T *spi);
   

   

   







 
#line 11940 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\timer.h"
 









 











 



 



 





















   




 







 









 








 







 
static __inline void TIMER_Start(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (0));
}





 
static __inline void TIMER_Stop(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (0));
}






 
static __inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (2));
}





 
static __inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (2));
}






 
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (22));
}





 
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (22));
}






 
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (14));
}





 
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (14));
}





 
static __inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->IER |= (0x1ul << (0));
}





 
static __inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->IER &= ~(0x1ul << (0));
}





 
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->IER |= (0x1ul << (1));
}





 
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->IER &= ~(0x1ul << (1));
}







 
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return(timer->ISR & (0x1ul << (0)) ? 1 : 0);
}





 
static __inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->ISR = (0x1ul << (0));
}







 
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return(timer->ISR & (0x1ul << (1)) ? 1 : 0);
}





 
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->ISR = (0x1ul << (1));
}







 
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->ISR & (0x1ul << (4)) ? 1 : 0);
}





 
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->ISR = (0x1ul << (4));
}





 
static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->TCAP;
}





 
static __inline uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->DR;
}

uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);
void TIMER_EnableFreqCounter(TIMER_T *timer,
                             uint32_t u32DropCount,
                             uint32_t u32Timeout,
                             uint32_t u32EnableInt);
void TIMER_DisableFreqCounter(TIMER_T *timer);
void TIMER_SetTriggerSource(TIMER_T *timer, uint32_t u32Src);
void TIMER_SetTriggerTarget(TIMER_T *timer, uint32_t u32Mask);

   

   

   







 
#line 11941 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\uart.h"
 









 












 



 



 


 
 
 


























 
 
 



 
 
 






   




 








 









 











 









 










 








 









 









 








 









 










 











 











 









 









 



















 


















 





















 








 
__inline void UART_CLEAR_RTS(UART_T* uart)
{
    uart->MCSR |= (0x1ul << (0));
}






 
__inline void UART_SET_RTS(UART_T* uart)
{
    uart->MCSR &= ~(0x1ul << (0));
}






 








 



void UART_ClearIntFlag(UART_T* uart, uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart );
void UART_DisableFlowCtrl(UART_T* uart );
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_EnableFlowCtrl(UART_T* uart );
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction);
void UART_SelectRS485Mode(UART_T* uart, uint32_t u32Mode, uint32_t u32Addr);
void UART_SelectLINMode(UART_T* uart, uint32_t u32Mode, uint32_t u32BreakLength);
uint32_t UART_Write(UART_T* uart,uint8_t *pu8TxBuf, uint32_t u32WriteBytes);


   

   

   







 








#line 11942 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\usbd.h"
 








 






 



 



 
typedef struct s_usbd_info
{
    uint8_t *gu8DevDesc;                 
    uint8_t *gu8ConfigDesc;              
    uint8_t **gu8StringDesc;             
    uint8_t **gu8HidReportDesc;          
    uint32_t *gu32HidReportSize;         
    uint32_t *gu32ConfigHidDescIdx;      
} S_USBD_INFO_T;


extern S_USBD_INFO_T gsInfo;


   



 




#line 57 "..\\Library\\StdDriver\\inc\\usbd.h"


extern volatile uint32_t g_usbd_UsbConfig;

 




 
#line 78 "..\\Library\\StdDriver\\inc\\usbd.h"

 
#line 87 "..\\Library\\StdDriver\\inc\\usbd.h"

 



 
#line 99 "..\\Library\\StdDriver\\inc\\usbd.h"

 







 


















#line 140 "..\\Library\\StdDriver\\inc\\usbd.h"















   




 










 












 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 







 






 







 







 






 











 










 












 















 
static __inline void USBD_MemCopy(uint8_t *dest, uint8_t *src, int32_t size)
{
    while (size--) *dest++ = *src++;
}









 
static __inline void USBD_SetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    int i;

    for (i=0; i<8; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) (((uint32_t)0x40000000) + 0x60000))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *) (u32CfgAddr));

        if((u32Cfg & 0xf) == epnum)
        {
            *((volatile uint32_t *) (u32CfgAddr)) = (u32Cfg | (0x1ul << (9)));
            break;
        }
    }
}








 
static __inline void USBD_ClearStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    int i;

    for (i=0; i<8; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) (((uint32_t)0x40000000) + 0x60000))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *) (u32CfgAddr));

        if((u32Cfg & 0xf) == epnum)
        {
            *((volatile uint32_t *) (u32CfgAddr)) = (u32Cfg & ~(0x1ul << (9)));
            break;
        }
    }
}









 
static __inline uint32_t USBD_GetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    int i;

    for (i=0; i<8; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) (((uint32_t)0x40000000) + 0x60000))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *) (u32CfgAddr));

        if((u32Cfg & 0xf) == epnum)
            break;
    }
    return (u32Cfg & (0x1ul << (9)));
}


 
extern volatile uint8_t g_usbd_RemoteWakeupEn;

typedef void (*VENDOR_REQ)(void);  

typedef void (*CLASS_REQ)(void);  

typedef void (*SET_INTERFACE_REQ)(uint32_t u32AltInterface);  
typedef void (*SET_CONFIG_CB)(void);        

 
void USBD_Open(S_USBD_INFO_T *param, CLASS_REQ pfnClassReq, SET_INTERFACE_REQ pfnSetInterface);
void USBD_Start(void);
void USBD_GetSetupPacket(uint8_t *buf);
void USBD_ProcessSetupPacket(void);
void USBD_StandardRequest(void);
void USBD_PrepareCtrlIn(uint8_t *pu8Buf, uint32_t u32Size);
void USBD_CtrlIn(void);
void USBD_PrepareCtrlOut(uint8_t *pu8Buf, uint32_t u32Size);
void USBD_CtrlOut(void);
void USBD_SwReset(void);
void USBD_SetVendorRequest(VENDOR_REQ pfnVendorReq);
void USBD_SetConfigCallback(SET_CONFIG_CB pfnSetConfigCallback);
void USBD_LockEpStall(uint32_t u32EpBitmap);


   

   

   




 
#line 11943 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\wdt.h"
 









 











 



 



 
#line 40 "..\\Library\\StdDriver\\inc\\wdt.h"






   




 






 







 







 








 








 








 







 






 
static __inline void WDT_Close(void)
{
    ((WDT_T *) (((uint32_t)0x40000000) + 0x04000))->CTL = 0;
    return;
}





 
static __inline void WDT_EnableInt(void)
{
    ((WDT_T *) (((uint32_t)0x40000000) + 0x04000))->IER = (0x1ul << (0));
    return;
}





 
static __inline void WDT_DisableInt(void)
{
    ((WDT_T *) (((uint32_t)0x40000000) + 0x04000))->IER = 0;
    return;
}

void  WDT_Open(uint32_t u32TimeoutInterval,
               uint32_t u32ResetDelay,
               uint32_t u32EnableReset,
               uint32_t u32EnableWakeup);

   

   

   







 
#line 11944 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\wwdt.h"
 









 











 



 



 
#line 48 "..\\Library\\StdDriver\\inc\\wwdt.h"


   




 






 







 








 








 







 










 



void WWDT_Open(uint32_t u32PreScale, uint32_t u32CmpValue, uint32_t u32EnableInt);


   

   

   







 
#line 11945 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\i2s.h"
 









 










 



 



 





 



 



 


 



 
#line 60 "..\\Library\\StdDriver\\inc\\i2s.h"

#line 69 "..\\Library\\StdDriver\\inc\\i2s.h"

 



 



   



 








 
static __inline void I2S_ENABLE_TX_ZCD(I2S_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == 0)
        i2s->CTRL |= (0x1ul << (16));
    else
        i2s->CTRL |= (0x1ul << (17));
}









 
static __inline void I2S_DISABLE_TX_ZCD(I2S_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == 0)
        i2s->CTRL &= ~(0x1ul << (16));
    else
        i2s->CTRL &= ~(0x1ul << (17));
}






 







 







 







 







 







 







 







 







 







 







 







 










 








 







 








 








 







 







 


uint32_t I2S_Open(I2S_T *i2s, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32Channels, uint32_t u32DataFormat, uint32_t u32AudioInterface);
void I2S_Close(I2S_T *i2s);
void I2S_EnableInt(I2S_T *i2s, uint32_t u32Mask);
void I2S_DisableInt(I2S_T *i2s, uint32_t u32Mask);
uint32_t I2S_EnableMCLK(I2S_T *i2s, uint32_t u32BusClock);
void I2S_DisableMCLK(I2S_T *i2s);
void I2S_SetFIFO(I2S_T *i2s, uint32_t u32TxThreshold, uint32_t u32RxThreshold);

   


   

   







 

#line 11946 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"
#line 1 "..\\Library\\StdDriver\\inc\\lcd.h"
 









 








#line 21 "..\\Library\\StdDriver\\inc\\lcd.h"





 



 




 


 
 
 



#line 52 "..\\Library\\StdDriver\\inc\\lcd.h"

#line 59 "..\\Library\\StdDriver\\inc\\lcd.h"





#line 72 "..\\Library\\StdDriver\\inc\\lcd.h"

#line 81 "..\\Library\\StdDriver\\inc\\lcd.h"












   




 
typedef enum
{
    LCD_C_TYPE = 0,           
    LCD_EXTERNAL_R_TYPE = 1,  
    LCD_INTERNAL_R_TYPE = 2,  
    LCD_EXTERNAL_C_TYPE = 3   
} LCD_PanelType;

   




 







 








 








 








 








 








 


uint32_t LCD_EnableFrameCounter(uint32_t u32Count);
void LCD_DisableFrameCounter(void);
uint32_t LCD_EnableBlink(uint32_t u32ms);
void LCD_DisableBlink(void);
void LCD_EnableInt(uint32_t IntSrc);
void LCD_DisableInt(uint32_t IntSrc);
uint32_t LCD_Open(uint32_t u32DrivingType, uint32_t u32ComNum, uint32_t u32BiasLevel, uint32_t u32FramerateDiv, uint32_t u32DrivingVol);
void LCD_SetPixel(uint32_t u32Com, uint32_t u32Seg, uint32_t u32OnFlag);
void LCD_SetAllPixels(uint32_t u32OnOff);
void LCD_Close(void);








 
static __inline void LCD_EnableDisplay(void)
{
     
    ((LCD_T *) (((uint32_t)0x40000000) + 0xB0000))->CTL |= (0x1ul << (0));
}








 
static __inline void LCD_DisableDisplay(void)
{
     
    ((LCD_T *) (((uint32_t)0x40000000) + 0xB0000))->CTL &= ~(0x1ul << (0));
}



   


   

   










 


#line 11947 "..\\Library\\Device\\Nuvoton\\Nano100Series\\Include\\Nano100Series.h"



 

#line 3 "..\\USER\\main.cpp"
#line 1 "..\\USER\\UC8581_driver.h"



#line 5 "..\\USER\\UC8581_driver.h"

#line 1 "..\\USER\\img.h"


    
#line 5 "..\\USER\\img.h"
#line 6 "..\\USER\\img.h"
const uint8_t img[] ={
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC0,0X07,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X01,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X01,0X00,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X1F,0XF0,0X7F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X3F,0XFC,0X7F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XFF,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XFF,0XFF,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XFF,0XFF,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XFF,0XFF,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XFF,0XFF,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X3F,0XFE,0X7F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X1F,0XFC,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X87,0XF0,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF0,0X00,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF0,0X00,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0X00,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF0,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X0F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X3F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X7F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X3F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X3F,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X3F,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X00,0X01,0XFF,0XFF,0XFF,0XFF,0XF8,0X1F,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XF0,0X3F,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X00,0X00,0X00,0X1F,0XFF,0XFF,0XE0,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X1F,0XE0,0X00,0X01,0XFF,0XFF,0X81,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X1F,0XFF,0XE0,0X00,0X3F,0XFF,0X03,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X1F,0XFF,0XFF,0X80,0X07,0XFC,0X0F,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X1F,0XFF,0XFF,0XF8,0X01,0XF0,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X1F,0XFF,0XFF,0XFF,0X00,0X40,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X1F,0XFF,0XFF,0XFF,0XE0,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X1F,0XFF,0XFF,0XFF,0XF0,0X03,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X1F,0XFF,0XFF,0XFF,0XC0,0X01,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X1F,0XFF,0XFF,0XFE,0X00,0X40,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X1F,0XFF,0XFF,0XE0,0X01,0XE0,0X3F,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFE,0X1F,0XFF,0XFC,0X00,0X0F,0XF8,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0X1F,0XFF,0X00,0X00,0XFF,0XFC,0X0F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0X1C,0X00,0X00,0X0F,0XFF,0XFF,0X07,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0X00,0X00,0X00,0XFF,0XFF,0XFF,0X83,0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0X00,0X00,0X3F,0XFF,0XFF,0XFF,0XC1,0XFF,0XF8,0X7C,0X00,0X00,0X3F,0XFF,
0XFF,0XFE,0X00,0X3F,0XFF,0XFF,0XFF,0XFF,0XE0,0XFF,0XF8,0X7C,0X00,0X00,0X3F,0XFF,
0XFF,0XFE,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XF0,0X7F,0XF8,0XFC,0X00,0X00,0X3F,0XFF,
0XFF,0XFE,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0X3F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0X1F,0XFF,0XFF,0XFF,0X9F,0XFF,0XFC,0X3F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0X7F,0X1F,0XFF,0XFE,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X00,0X7E,0X3F,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X00,0X7C,0X3F,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0X7C,0X00,0X7C,0X7E,0X3F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0X3C,0X00,0X78,0X7E,0X3F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0X3C,0X7F,0XF8,0XFC,0X00,0X00,0X00,0X1F,0XFF,0XFC,0X00,0X00,0X3F,0XFF,
0XFF,0XFE,0X3C,0X7F,0XF1,0XFC,0X00,0X00,0X00,0X1F,0XFF,0XFC,0X00,0X00,0X3F,0XFF,
0XFF,0XFE,0X3C,0X7F,0XE1,0XF8,0X00,0X00,0X00,0X1F,0XFF,0XFC,0X00,0X00,0X3F,0XFF,
0XFF,0XFE,0X3C,0X7F,0XE3,0XF8,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,0XFF,
0XFF,0XFE,0X3C,0X78,0XE3,0XF0,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0XFF,
0XFF,0XFE,0X3C,0X78,0XE7,0XF1,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XFE,0X3F,0XFF,
0XFF,0XFE,0X3C,0X78,0XF7,0XE1,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XFF,0X3F,0XFF,
0XFF,0XFE,0X3C,0X78,0XFF,0XE3,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XFF,0X3F,0XFF,
0XFF,0XFE,0X3C,0X78,0XFF,0XC7,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XFF,0X3F,0XFF,
0XFF,0XFE,0X3C,0X78,0XFF,0X87,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XFF,0X3F,0XFF,
0XFF,0XFE,0X3C,0X78,0XFF,0X8F,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XFE,0X3F,0XFF,
0XFF,0XFE,0X3C,0X78,0XFD,0X1F,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XFE,0X3F,0XFF,
0XFF,0XFE,0X3C,0X78,0XFC,0X1F,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XFC,0X3F,0XFF,
0XFF,0XE0,0X00,0X78,0XF8,0X3F,0X8F,0X3C,0X79,0XFF,0XFF,0XFC,0X00,0X00,0X7F,0XFF,
0XFF,0XE0,0X00,0X78,0XFC,0X3F,0X8F,0X3C,0X79,0XFF,0XFF,0XFC,0X00,0X00,0XFF,0XFF,
0XFF,0XE0,0X00,0X78,0XFC,0X3F,0X8F,0X3C,0X79,0XFF,0XFF,0XFC,0X00,0X03,0XFF,0XFF,
0XFF,0XE0,0X00,0X78,0XFE,0X1F,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X78,0XFF,0X0F,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X78,0XFF,0X8F,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X78,0XFF,0XC7,0X8F,0X3C,0X79,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X78,0XE7,0XC3,0X8F,0X3C,0X79,0XFF,0XFF,0XFC,0X7F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X78,0XC7,0XE3,0X8F,0X3C,0X79,0XFF,0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X78,0XC3,0XE1,0X8F,0X3C,0X79,0XFF,0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X78,0XE1,0XF1,0X8F,0X3C,0X79,0XFF,0XFF,0XFC,0X7F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X78,0XF1,0XF8,0X8F,0X3C,0X79,0XFF,0XFF,0XFE,0X7F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X7F,0XF0,0XF8,0X8F,0X1C,0X78,0XFF,0XFF,0XFF,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X7F,0XF8,0XFC,0X00,0X00,0X00,0X1F,0XFF,0XFF,0X87,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X7F,0XF8,0X7C,0X00,0X00,0X00,0X1F,0XFF,0XFC,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFC,0X7F,0XFC,0X7C,0X00,0X00,0X00,0X1F,0XFF,0XFC,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFC,0X00,0X7C,0X3E,0X3F,0XFF,0XFF,0XFF,0XFF,0XFC,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFC,0X00,0X7E,0X3E,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0X00,0X7E,0X1F,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X3F,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X7F,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XF8,0X3F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XF8,0X0F,0XF8,0X00,0X00,0X00,0X07,0XFC,0X7F,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFC,0X07,0XF8,0X00,0X00,0X00,0X01,0XFC,0X7F,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFF,0X03,0XF8,0X00,0X00,0X00,0X00,0XFC,0X7F,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,
0XFF,0XFF,0XC0,0XF8,0X00,0X00,0X00,0X00,0XFC,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XE0,0X78,0X7F,0XF8,0XFF,0XF8,0X7C,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF0,0X38,0X7F,0XF8,0X7F,0XF8,0X7C,0X7F,0XFF,0XFF,0XE0,0X07,0XFF,0XFF,
0XFF,0XFF,0XFC,0X78,0X7F,0XFC,0X7F,0XF8,0X7C,0X7F,0XFF,0XFF,0X80,0X01,0XFF,0XFF,
0XFF,0XFF,0XFE,0X78,0X7F,0XFC,0X3F,0XF8,0X7C,0X7F,0XFF,0XFF,0X00,0X00,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF8,0X7F,0X3E,0X3F,0XF8,0X7C,0X7F,0XFF,0XFE,0X1F,0XF8,0X7F,0XFF,
0XFF,0XFF,0XFF,0XF8,0X7F,0X0E,0X1F,0XF8,0X7C,0X7F,0XFF,0XFE,0X3F,0XFC,0X7F,0XFF,
0XFF,0XFF,0XFF,0XF8,0X7F,0X1E,0X1F,0XF8,0X7C,0X7F,0XFF,0XFC,0X7F,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XF8,0X7F,0X1F,0X1F,0XF8,0X7C,0X7F,0XFF,0XFC,0X7F,0XFE,0X3F,0XFF,
0XFF,0XE0,0X00,0X00,0X7E,0X1F,0X0F,0XFF,0XFC,0X7F,0XFF,0XFC,0XFF,0XFF,0X3F,0XFF,
0XFF,0XE0,0X00,0X00,0X7E,0X3F,0X8F,0XFF,0XFC,0X7F,0XFF,0XFC,0XFF,0XFF,0X3F,0XFF,
0XFF,0XE0,0X00,0X00,0X7E,0X3F,0X87,0XFF,0XFC,0X7F,0XFF,0XFC,0XFF,0XFF,0X3F,0XFF,
0XFF,0XE0,0X00,0X00,0X7E,0X3F,0X87,0XFF,0XFC,0X7F,0XFF,0XFC,0XFF,0XFF,0X3F,0XFF,
0XFF,0XFF,0XFF,0XF8,0X7C,0X3F,0XC7,0XFF,0XFC,0X7F,0XFF,0XFC,0XFF,0XFF,0X3F,0XFF,
0XFF,0XFF,0XFF,0XF8,0X7C,0X3F,0XC3,0XFF,0XFC,0X7F,0XFF,0XFC,0X7F,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XF8,0X7C,0X7F,0XC3,0XFF,0XFC,0X7F,0XFF,0XFC,0X3F,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XF8,0X7C,0X7F,0XE3,0XFF,0XFC,0X7F,0XFF,0XFE,0X1F,0XF8,0X7F,0XFF,
0XFF,0XFF,0XFF,0XF8,0X7C,0X7F,0XE1,0XFF,0XFC,0X7F,0XFF,0XFF,0X0F,0XF0,0XFF,0XFF,
0XFF,0XFF,0XFC,0X78,0X7C,0X7F,0XE1,0XFF,0XFC,0X7F,0XFF,0XFF,0X80,0X00,0XFF,0XFF,
0XFF,0XFF,0XF0,0X38,0X78,0X7F,0XF1,0XFF,0XF8,0X7F,0XFF,0XFF,0XC0,0X03,0XFF,0XFF,
0XFF,0XFF,0XC0,0X38,0X00,0X07,0XF3,0XFF,0XF8,0X7F,0XFF,0XFF,0XF0,0X0F,0XFF,0XFF,
0XFF,0XFF,0X80,0XF8,0X00,0X00,0X03,0XFF,0XF8,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0X03,0XF8,0X00,0X00,0X00,0X7F,0XF8,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF8,0X0F,0XF8,0X00,0X00,0X00,0X0F,0XF8,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF8,0X3F,0XFF,0XFF,0XFF,0X80,0X03,0XF8,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF8,0X7F,0XFF,0XFF,0XFF,0XFC,0X00,0XF8,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X78,0X7F,0XFF,0XFF,0X80,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE0,0X78,0X7F,0XFF,0XFE,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC7,0XFF,0XFF,0XF8,0XF8,0XFF,0XFF,0XFE,0X00,0X01,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC7,0XFF,0XFF,0XFE,0XF8,0XFF,0XFF,0XFC,0X3C,0XF8,0XFF,0XFF,
0XFF,0XFF,0XF1,0XFF,0XC7,0XFE,0X1F,0XFF,0XF0,0XFF,0XFF,0XFC,0X7C,0XFE,0X7F,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFE,0X1F,0XFF,0XF0,0XFF,0XFF,0XFC,0XFC,0XFE,0X3F,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFE,0X1F,0XFF,0XF0,0XFF,0XFF,0XFC,0XFC,0XFF,0X3F,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFE,0X1F,0XFF,0XF0,0XFF,0XFF,0XFC,0XFC,0XFF,0X3F,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFE,0X1F,0XFF,0XE1,0XFF,0XFF,0XFC,0XFC,0X7F,0X3F,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFE,0X1F,0XFF,0XE1,0XFF,0XFF,0XFC,0XFC,0X7F,0X3F,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFE,0X1F,0XFF,0XE1,0XFF,0XFF,0XFC,0X7E,0X7E,0X3F,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFE,0X1F,0XFF,0XC3,0XFF,0XFF,0XFE,0X7E,0X3C,0X3F,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC0,0X00,0X00,0X00,0X03,0XFF,0XFF,0XFE,0X3F,0X00,0X3F,0XFF,
0XFF,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X07,0XFF,0XFF,0XFF,0XFF,0X00,0X7F,0XFF,
0XFF,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X07,0XFF,0XFF,0XFF,0XFF,0XC0,0XFF,0XFF,
0XFF,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XE0,0X00,0X00,0X07,0XFF,0XFF,0XFF,0X0F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFF,0XFF,0XFE,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFF,0XFF,0XFC,0X3F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFF,0XFF,0XF8,0X3F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFF,0XFF,0XF0,0X7F,0XFF,0XFF,0XFF,0XF0,0X00,0X3F,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFF,0XFF,0XC0,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XE0,0XFF,0X81,0XFF,0XFF,0XFF,0XFE,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XE0,0X00,0X07,0XFF,0XFF,0XFF,0XFC,0X0F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XE0,0X00,0X03,0XFF,0XFF,0XFF,0XFC,0X3F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XE0,0X00,0X00,0X3F,0XFF,0XFF,0XFC,0X7F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFF,0XF0,0X00,0X03,0XFF,0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF0,0XFF,0XC7,0XFF,0XFF,0XE0,0X00,0X7F,0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF1,0XFF,0XC7,0XFF,0XFF,0XFF,0X00,0X3F,0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC7,0XFF,0XFF,0XFF,0XE0,0X3F,0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC7,0XFF,0XFF,0XFF,0XFE,0X7F,0XFF,0XFC,0X7F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X7F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X3F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF0,0X00,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF0,0X00,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0X00,0X00,0X00,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XFF,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X3F,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X0F,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X07,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X43,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X70,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X78,0X7E,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7C,0X1E,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0X0E,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0X86,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0XE0,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0XF0,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0XF8,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0X7F,0XFE,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X3F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
};


const uint8_t img2[] = { 
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0X00,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0X00,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0X00,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0X00,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0X00,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0X00,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X00,0X00,0X03,0XF0,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X00,0X00,0X03,0XF0,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X00,0X38,0X03,0XF0,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X01,0XF8,0X03,0XF0,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X0F,0XF8,0X03,0XF0,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X7F,0XF8,0X03,0XF0,0X1F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X03,0XFF,0XF8,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X1F,0XFF,0XF8,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X7F,0XFF,0XF0,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X7F,0XFF,0X80,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X7F,0XFC,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X7F,0XF8,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X3F,0XFE,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X07,0XFF,0XC0,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0XFF,0XF8,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X1F,0XF8,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X1F,0XF8,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X7F,0XF8,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X03,0XFF,0XF8,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X1F,0XFF,0XE0,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X7F,0XFF,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X7F,0XFC,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X7F,0XF0,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X7F,0XFC,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X1F,0XFF,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X03,0XFF,0XE0,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0XFF,0XF8,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X3F,0XFF,0X80,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X07,0XFF,0XC0,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X01,0XFF,0XE0,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X7C,0X00,0X7F,0XF0,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X7F,0X00,0X0F,0XF8,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X7F,0XC0,0X07,0XFC,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X7F,0XE0,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X7F,0XF8,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X3F,0XFE,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X07,0XFF,0XC0,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X01,0XFF,0XF0,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X83,0XFF,0X00,0X7F,0XF8,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X81,0XFE,0X00,0X1F,0XF8,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0XFF,0X00,0X1F,0XF8,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X7F,0XC0,0X7F,0XF8,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X3F,0XF9,0XFF,0XF0,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0X1F,0XFF,0XFF,0X80,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X3F,0X0B,0XFF,0XFE,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X3F,0X00,0XFF,0XF8,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X3F,0X00,0X3F,0XE0,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X3F,0X00,0X0F,0X80,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X3F,0X00,0X02,0X00,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X3F,0X00,0X00,0X00,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X18,0XFF,0XFF,0XFE,0X7F,0XF9,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X01,0XF8,0XFF,0XFF,0XF8,0X1F,0XF0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X3F,0XF8,0XFF,0XFF,0XE0,0X0F,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XE0,0XFF,0XFF,0XE0,0X07,0X80,0X7F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFC,0X00,0XFF,0XFF,0XF0,0X03,0XC0,0X1F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0X00,0XFF,0XFF,0XF8,0X00,0XE0,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X3F,0XF0,0XC0,0X00,0X00,0X00,0X70,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X03,0XF8,0XC0,0X00,0X00,0X00,0X18,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X07,0XF8,0XC0,0X00,0X00,0X00,0X04,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0XE0,0XC0,0X00,0X00,0X00,0X06,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0X00,0XC0,0X00,0X00,0X00,0X07,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFC,0X00,0XFF,0XFF,0XFF,0XFC,0X07,0XC3,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XE0,0XF8,0XFF,0XFF,0XFE,0X0F,0XE3,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X1F,0XF8,0XF8,0X3F,0XFF,0X06,0X00,0X13,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X01,0XF8,0XE0,0X03,0XFF,0X06,0X00,0X1F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X18,0XE0,0X00,0X01,0X06,0X00,0X1F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XE0,0X00,0X01,0X06,0X00,0X1F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X70,0X00,0XF8,0X00,0X01,0X06,0X00,0X1F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XF9,0X80,0XFE,0X00,0X01,0X06,0X0F,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XF9,0XC0,0XFF,0XC0,0X01,0X06,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCC,0XC0,0XFF,0XFF,0XC1,0X06,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCC,0XC0,0XFF,0XFF,0XC1,0X06,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XC0,0XFF,0X8F,0XC1,0X06,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0X80,0XFC,0X00,0X01,0X06,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0X00,0XFE,0X00,0X01,0X06,0X0F,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFE,0X00,0X01,0X06,0X00,0X1F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X40,0XFD,0X00,0X01,0X06,0X00,0X1F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X03,0XC0,0XF8,0X80,0X01,0X06,0X00,0X1F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X1F,0XC0,0XE0,0X40,0XFF,0X02,0X00,0X1F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0X80,0XC0,0X40,0XFF,0X00,0X00,0X1F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFC,0X00,0XC0,0X20,0X7E,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFC,0X00,0XE0,0X00,0X7E,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0X80,0XF0,0X07,0XF0,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X1F,0XC0,0XF8,0X03,0X00,0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X03,0XC0,0XFC,0X00,0X00,0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X40,0XFE,0X00,0X00,0X01,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X1E,0X00,0XFF,0X00,0X00,0X7F,0X80,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0X80,0XFF,0X00,0X00,0X7F,0X83,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0X80,0XFC,0X00,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XED,0XC0,0XF8,0X00,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCC,0XC0,0XF0,0X03,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCC,0XC0,0XE0,0X07,0XE0,0X00,0X03,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCF,0XC0,0XE0,0X1F,0XFC,0X00,0X03,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCF,0X80,0XE0,0X3F,0XFF,0XE0,0X03,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X0E,0X00,0XF8,0X7F,0XFF,0XFF,0X83,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFC,0XFF,0XFF,0XFF,0X83,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X43,0X00,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC7,0X80,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCF,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCC,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFC,0XC0,0XFF,0XFF,0XFF,0XE0,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X78,0XC0,0XFF,0XFF,0XFF,0XE0,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X30,0X40,0XFF,0XFF,0XFF,0XE0,0X00,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XF0,0X7F,0XFF,0XE0,0X00,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XFC,0XF0,0X7F,0X83,0XE0,0X00,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XFC,0XF0,0X60,0X83,0XFF,0XE0,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XFC,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X01,0XC0,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0XC0,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0XC0,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XC0,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0X80,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0X00,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X70,0X00,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XF9,0X80,0XF0,0X60,0X83,0XFF,0XE0,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XF9,0XC0,0XF0,0X60,0X83,0XFF,0XE0,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCC,0XC0,0XF0,0X60,0X83,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCC,0XC0,0XF0,0X60,0X83,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XC0,0XF0,0X60,0X83,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0X80,0XF0,0X60,0X83,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0X00,0XF0,0X60,0X83,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XF0,0X60,0X83,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XF0,0X60,0X83,0XFF,0XE0,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XC0,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XC0,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XC0,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X01,0XC0,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0XC0,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0XC0,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XF0,0X60,0X83,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X1E,0X00,0XC0,0X00,0X03,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0X80,0XC0,0X00,0X03,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0X80,0XC0,0X00,0X03,0X84,0X20,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XED,0XC0,0XC0,0X00,0X03,0XFF,0XE0,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCC,0XC0,0XC0,0X00,0X03,0XE0,0X00,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCC,0XC0,0XC0,0X00,0X03,0XE0,0X00,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCF,0XC0,0XFF,0XFF,0XFF,0XE0,0X00,0X83,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCF,0X80,0XFF,0XFF,0XFF,0XE0,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X0E,0X00,0XFF,0XFF,0XFF,0XE0,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XE0,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XF8,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XF8,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XF8,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC3,0X18,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC3,0X18,0XFF,0XF0,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC3,0X18,0XFF,0XF0,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC3,0X18,0XFF,0XF0,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC0,0X18,0XFF,0XF0,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XF0,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XF0,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XFC,0XFF,0XFE,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XFC,0XFF,0XFE,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XFC,0XFF,0XFE,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFE,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFE,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X1E,0X00,0XFF,0XFE,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0X80,0XFF,0XFE,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0X80,0XFF,0XFE,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XED,0XC0,0XFF,0XFE,0X07,0X03,0XC0,0X7F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCC,0XC0,0XFC,0X00,0X00,0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCC,0XC0,0XF8,0X00,0X00,0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCF,0XC0,0XF0,0X00,0X00,0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCF,0X80,0XE0,0X00,0X00,0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X0E,0X00,0XE0,0X00,0X00,0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XE0,0X00,0X00,0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X1E,0X00,0XE0,0X3E,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0X80,0XE0,0X7E,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0X80,0XE0,0X7E,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XE1,0XC0,0XE0,0X7E,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC0,0XC0,0XE0,0X7E,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC0,0XC0,0XE0,0X7E,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC0,0XC0,0XE0,0X7E,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X40,0XC0,0XE0,0X7E,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XE0,0X7E,0X07,0X83,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0XC0,0XE0,0X7E,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0XF0,0XE0,0X7E,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XF0,0XE0,0X7E,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XF0,0XE0,0X3E,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC0,0XC0,0XE0,0X02,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC0,0XC0,0XE0,0X00,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XE0,0X01,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XC0,0XF0,0X01,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XC0,0XF8,0X01,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XC0,0XFE,0X03,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X01,0XC0,0XFF,0XC3,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X1E,0X00,0XFF,0XFF,0XF8,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0X80,0XFF,0XFF,0XF8,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0X80,0XFF,0XFF,0XF8,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XE1,0XC0,0XFF,0XFF,0XF8,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC0,0XC0,0XFF,0XFF,0XF8,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XE1,0XC0,0XFF,0XFF,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0XC0,0XFF,0XFF,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0X80,0XFF,0XFF,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X1E,0X00,0XFF,0XFF,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0X7F,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XC0,0XFC,0X7F,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XC0,0XE0,0X7F,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XC0,0XC0,0X7F,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X01,0X80,0XC0,0X7F,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0XC0,0XC0,0X7F,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0XC0,0XC0,0X7F,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XC0,0XC0,0X7F,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0X80,0XE0,0X7F,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0X00,0XE0,0X7F,0XF8,0X1F,0XFC,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XE0,0X00,0X00,0X00,0X3C,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XE0,0X00,0X00,0X00,0X3C,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XD8,0XE0,0X00,0X00,0X00,0X1C,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XD8,0XF0,0X00,0X00,0X00,0X1C,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFF,0XD8,0XF0,0X00,0X00,0X00,0X0C,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XF8,0X00,0X00,0X00,0X04,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XF8,0X1C,0X04,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X1E,0X00,0XFF,0XFF,0XF8,0X1E,0X00,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0X80,0XFF,0XFF,0XF8,0X1E,0X00,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X7F,0X80,0XFF,0XFF,0XF8,0X1F,0X00,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XE1,0XC0,0XFF,0XFF,0XF8,0X1F,0X00,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC0,0XC0,0XFF,0XFF,0XF8,0X1F,0X80,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC0,0XC0,0XFF,0XFF,0XF8,0X1F,0XC0,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC0,0XC0,0XFF,0XFF,0XF8,0X1F,0XC0,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X40,0XC0,0XFF,0XFF,0XF8,0X1F,0XE0,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XF8,0X1F,0XF0,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X43,0X00,0XFF,0XFF,0XF8,0X1F,0XF8,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XC7,0X80,0XFF,0XFF,0XF8,0X1F,0XF8,0X07,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCF,0XC0,0XFF,0XFF,0XF8,0X1F,0XFC,0X0F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XCC,0XC0,0XFF,0XFF,0XF8,0X1F,0XFE,0X1F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0XFC,0XC0,0XFF,0XFF,0XF8,0X1F,0XFF,0X3F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X78,0XC0,0XFF,0XFF,0XF8,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X30,0X40,0XFF,0XFF,0XF8,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XF8,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XF8,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF};






#line 7 "..\\USER\\UC8581_driver.h"


#line 16 "..\\USER\\UC8581_driver.h"








void UC8151_io_init(){
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04000)) , (0x00000020)  , 0x0UL);  
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04000)) , (0x00000010)  , 0x1UL);  
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04040)) , (0x00001000) , 0x1UL);  
    
    
    
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04040)) , (0x00000010)  , 0x1UL);  

    SPI_Open(((SPI_T *) (((uint32_t)0x40000000) + 0xD0000)), (0x0), ((0x1ul << (2))), 8 , 500000);
    SPI_EnableAutoSS(((SPI_T *) (((uint32_t)0x40000000) + 0xD0000)), (0x1), (0x0));
}


void wait_busy(){
    while (!(*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x40*(0))) + ((5)<<2))))){}
}
void UC8151_write(uint8_t type , uint8_t payload){
    (*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x40*(1))) + ((12)<<2)))) = type;
    CLK_SysTickDelay(1);
    ( (((SPI_T *) (((uint32_t)0x40000000) + 0xD0000)))->TX0 = payload );
    ( (((SPI_T *) (((uint32_t)0x40000000) + 0xD0000)))->CTL |= (0x1ul << (0)) );
    while(( ((((SPI_T *) (((uint32_t)0x40000000) + 0xD0000)))->CTL & (0x1ul << (0))) == (0x1ul << (0)) ? 1:0)){}
    CLK_SysTickDelay(1);
}


void UC8151_init(){
    
    (*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x40*(1))) + ((4)<<2)))) = 0;
    CLK_SysTickDelay(1000);

    (*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x40*(0))) + ((4)<<2)))) = 1;
    CLK_SysTickDelay(1000);
    (*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x40*(0))) + ((4)<<2)))) = 0;
    CLK_SysTickDelay(1000);
    (*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x40*(0))) + ((4)<<2)))) = 1;
    CLK_SysTickDelay(1000);

    UC8151_write(0 ,0x04);
    CLK_SysTickDelay(100);
    wait_busy();

    UC8151_write(0  , 0x00);
    UC8151_write(1 , 0x0f);
    UC8151_write(1 , 0x89);


    UC8151_write(0  , 0x61);
    UC8151_write(1 , 0x80);
    UC8151_write(1 , 0x01);
    UC8151_write(1 , 0x28);

    UC8151_write(0  , 0X50);
    UC8151_write(1 , 0xB7);
                            
                            
                            

}

void UC8151_send_frame(const uint8_t* black_img ,const uint8_t* red_img){
    UC8151_write(0,0x10);
    
    for(int y=0 ; y<296 ; y++){
        for(int x=0 ; x<128 ; x++){
            if(black_img)
                UC8151_write(1 , black_img[ y*128 + x]);
            else
                UC8151_write(1 , 0xFF);
        }
    }

    
    UC8151_write(0 , 0x13);
    for(int y=0 ; y<296 ; y++){
        for(int x=0 ; x<128 ; x++){
            if(red_img)
                UC8151_write(1 , red_img[ y*128 + x] );
            else
                UC8151_write(1 , 0xFF);
        }
    }
    UC8151_write(0 , 0x12);
    wait_busy();  


}
    









#line 4 "..\\USER\\main.cpp"


void SYS_Init(void){
    SYS_UnlockReg();

    CLK_SetCoreClock(42000000);



    CLK_SetModuleClock(((1UL<<31)|(1<<29)|(3<<25) |( 0<<20)|(0<<18)|(0xF<<10) |( 8<<5)|(16) ), (0x3UL<<(0)), (((1-1)<< (8)) & (0xful << (8))));
    CLK_SetModuleClock(((1UL<<31)|(2<<29)|(1<<25) |(22<<20)|(0<<18)|(0x0<<10)|( 0<<5)|(14) ) , (0x1UL<<(20)), 0);


    CLK_EnableModuleClock(((1UL<<31)|(1<<29)|(3<<25) |( 0<<20)|(0<<18)|(0xF<<10) |( 8<<5)|(16) ));
    CLK_EnableModuleClock(((1UL<<31)|(2<<29)|(1<<25) |(22<<20)|(0<<18)|(0x0<<10)|( 0<<5)|(14) ));
    
    
    
    SystemCoreClockUpdate();


    

    
    ((SYS_T *) (((uint32_t)0x50000000) + 0x00000))->PA_H_MFP &= ~( (0x7ul << (28)) | (0x7ul << (24)));
    ((SYS_T *) (((uint32_t)0x50000000) + 0x00000))->PA_H_MFP |= ((6UL<<(28))|(6UL<<(24)));
    
    
    ((SYS_T *) (((uint32_t)0x50000000) + 0x00000))->PA_H_MFP &= ~((0x7ul << (12))        | (0x7ul << (4))       | (0x7ul << (4)));
    ((SYS_T *) (((uint32_t)0x50000000) + 0x00000))->PA_H_MFP |=   (4UL<<(12)) | (4UL<<(4)) | (4UL<<(0));
    
    SYS_LockReg();
}



int main(){

    SYS_Init();
     
    UART_Open(((UART_T *) (((uint32_t)0x40000000) + 0x50000)), 115200);

    UC8151_io_init();
    UC8151_init();



    printf("start send frame \n ");
    UC8151_send_frame(img , 0);


    
    

    printf("finish \n\n");


    int i=0;
    while(1){
        i++;
    }

}
