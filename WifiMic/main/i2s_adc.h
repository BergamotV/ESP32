#ifndef __I2S_ADC_H__
#define __I2S_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif


extern void example_i2s_init();
extern void read_i2s_adc( uint8_t *d_buff, uint32_t len, size_t *bytes_read );



#ifdef __cplusplus
}
#endif

#endif /*#ifndef __UDP_PERF_H__*/