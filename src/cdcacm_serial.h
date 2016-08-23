#ifdef __cplusplus
extern "C" {
#endif

void cdc_acm_tx();

void cdc_acm_rx();

void init_cdc_acm();

void cdcacm_setup(void);

void baudrate_reset(void);

void usb_serial(void);

#ifdef __cplusplus
}
#endif
