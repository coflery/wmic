#ifndef _DELAY_H
#define _DELAY_H

#define INT_100MS 10    /* SystemCoreClock / 10           100ms中断一次 */
#define INT_10MS 100    /* SystemCoreClock / 100          10ms 中断一次 */
#define INT_1MS 1000    /* SystemCoreClock / 1000         1ms  中断一次 */
#define INT_100US 10000 /* SystemCoreClock / 10000        100us中断一次 */
#define INT_10US 100000 /* SystemCoreClock / 100000       10us 中断一次 */
#define INT_1US 1000000 /* SystemCoreClock / 1000000      1us  中断一次 */

extern volatile uint32_t SystemTime;

void Delay_Init(uint32_t TimeBase);
void Delay_us(uint32_t us);
void Delay_ms(uint16_t ms);
void Delay_cycles(uint32_t cycle);
void Delay_nop(void);

#endif