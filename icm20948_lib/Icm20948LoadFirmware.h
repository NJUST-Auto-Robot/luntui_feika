
#ifndef INV_ICM20948_LOAD_FIRMWARE_H__
#define INV_ICM20948_LOAD_FIRMWARE_H__

/** @defgroup	icm20948_load_firmware	load_firmware
    @ingroup 	SmartSensor_driver
    @{
*/
#ifdef __cplusplus
extern "C"
{
#endif

/* forward declaration */
struct inv_icm20948;

/** @brief Loads the DMP firmware from SRAM
* @param[in] data  pointer where the image 
* @param[in] size  size if the image
* @param[in] load_addr  address to loading the image
* @return 0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_firmware_load(struct inv_icm20948 * s, const unsigned char *data, unsigned short size, unsigned short load_addr);

#ifdef __cplusplus
}
#endif
#endif // INV_ICM20948_LOAD_FIRMWARE_H__

/** @} */
