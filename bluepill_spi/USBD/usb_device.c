#include <assert.h>
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_desc.h"


/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

void USB_Device_Init (void)
{
   /* Init Device Library, add supported class and start the library. */
   assert(USBD_Init (&hUsbDeviceFS, &FS_Desc, DEVICE_FS) == USBD_OK);   
   assert(USBD_RegisterClass (&hUsbDeviceFS, &USBD_CDC) == USBD_OK);
   assert(USBD_CDC_RegisterInterface (&hUsbDeviceFS, &USBD_Interface_fops_FS) == USBD_OK);
   assert(USBD_Start (&hUsbDeviceFS) == USBD_OK);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
