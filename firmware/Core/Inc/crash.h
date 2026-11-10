/*************************************************************
 * @file crash.h
 * @author MicroOrange
 * @brief 
 * @date 2022/12/27
 *************************************************************/

#ifndef F4_25R3916_CRASH_H
#define F4_25R3916_CRASH_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SORG_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
#define sorg_assert(expr) ((void)0U)
#endif



#ifdef __cplusplus
}
#endif

#endif //F4_25R3916_CRASH_H
