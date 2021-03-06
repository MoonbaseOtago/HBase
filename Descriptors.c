/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#include "Descriptors.h"

const USB_Descriptor_Device_t DeviceDescriptor PROGMEM =
{
	Header:                 {Size: sizeof(USB_Descriptor_Device_t), Type: DTYPE_Device},
		
	USBSpecification:       VERSION_BCD(01.20),
	Class:                  0x02,
	SubClass:               0x00,
	Protocol:               0x00,
				
	Endpoint0Size:          8,
		
	VendorID:               0x1d50,
	ProductID:              0x60bc,
	ReleaseNumber:          0x0000,
		
	ManufacturerStrIndex:   0x01,
	ProductStrIndex:        0x02,
	SerialNumStrIndex:      0x03,
		
	NumberOfConfigurations: 1
};
	
const USB_Descriptor_Configuration_t ConfigurationDescriptor PROGMEM =
{
	Config:
		{
			Header:                 {Size: sizeof(USB_Descriptor_Configuration_Header_t), Type: DTYPE_Configuration},

			TotalConfigurationSize: sizeof(USB_Descriptor_Configuration_t),
			TotalInterfaces:        2,
				
			ConfigurationNumber:    1,
			ConfigurationStrIndex:  NO_DESCRIPTOR_STRING,
				
			ConfigAttributes:       (USB_CONFIG_ATTR_BUSPOWERED | USB_CONFIG_ATTR_SELFPOWERED),
			
			MaxPowerConsumption:    USB_CONFIG_POWER_MA(100)
		},
		
	CCI_Interface:
		{
			Header:                 {Size: sizeof(USB_Descriptor_Interface_t), Type: DTYPE_Interface},

			InterfaceNumber:        0,
			AlternateSetting:       0,
			
			TotalEndpoints:         1,
				
			Class:                  0x02,
			SubClass:               0x02,
			Protocol:               0x01,
				
			InterfaceStrIndex:      NO_DESCRIPTOR_STRING
		},

	CDC_Functional_IntHeader:
		{
			FuncHeader:             {Header:
										{Size: sizeof(CDC_FUNCTIONAL_DESCRIPTOR(2)),
										 Type: 0x24},
									 SubType: 0x00},
			
			Data:                   {0x10, 0x01}
		},

	CDC_Functional_CallManagement:
		{
			FuncHeader:             {Header:
										{Size: sizeof(CDC_FUNCTIONAL_DESCRIPTOR(2)),
										 Type: 0x24},
									 SubType: 0x01},
			
			Data:                   {0x03, 0x01}
		},

	CDC_Functional_AbstractControlManagement:
		{
			FuncHeader:             {Header:
										{Size: sizeof(CDC_FUNCTIONAL_DESCRIPTOR(1)),
										 Type: 0x24},
									 SubType: 0x02},
			
			Data:                   {0x06}
		},
		
	CDC_Functional_Union:
		{
			FuncHeader:             {Header:
										{Size: sizeof(CDC_FUNCTIONAL_DESCRIPTOR(2)),
										 Type: 0x24},
									 SubType: 0x06},
			
			Data:                   {0x00, 0x01}
		},	

	ManagementEndpoint:
		{
			Header:                 {Size: sizeof(USB_Descriptor_Endpoint_t), Type: DTYPE_Endpoint},
										 
			EndpointAddress:        (ENDPOINT_DESCRIPTOR_DIR_IN | CDC_NOTIFICATION_EPNUM),
			Attributes:       		EP_TYPE_INTERRUPT,
			EndpointSize:           CDC_NOTIFICATION_EPSIZE,
			PollingIntervalMS:		0x02
		},

	DCI_Interface:
		{
			Header:                 {Size: sizeof(USB_Descriptor_Interface_t), Type: DTYPE_Interface},

			InterfaceNumber:        1,
			AlternateSetting:       0,
			
			TotalEndpoints:         2,
				
			Class:                  0x0A,
			SubClass:               0x00,
			Protocol:               0x00,
				
			InterfaceStrIndex:      NO_DESCRIPTOR_STRING
		},

	DataOutEndpoint:
		{
			Header:                 {Size: sizeof(USB_Descriptor_Endpoint_t), Type: DTYPE_Endpoint},
										 
			EndpointAddress:        (ENDPOINT_DESCRIPTOR_DIR_OUT | CDC_RX_EPNUM),
			Attributes:       		EP_TYPE_BULK,
			EndpointSize:           CDC_TXRX_EPSIZE,
			PollingIntervalMS:		0x00
		},
		
	DataInEndpoint:
		{
			Header:                 {Size: sizeof(USB_Descriptor_Endpoint_t), Type: DTYPE_Endpoint},
										 
			EndpointAddress:        (ENDPOINT_DESCRIPTOR_DIR_IN | CDC_TX_EPNUM),
			Attributes:       		EP_TYPE_BULK,
			EndpointSize:           CDC_TXRX_EPSIZE,
			PollingIntervalMS:		0x00
		}
};

const USB_Descriptor_String_t LanguageString PROGMEM =
{
	Header:                 {Size: USB_STRING_LEN(1), Type: DTYPE_String},
		
	UnicodeString:          {LANGUAGE_ID_ENG}
};

const USB_Descriptor_String_t ManufacturerString PROGMEM =
{
	Header:                 {Size: USB_STRING_LEN(80), Type: DTYPE_String},
		
	UnicodeString:          L"Moonbase Otago http://www.moonbaseotago.com/cheaprf/programmer1/programmer1.html"
};

const USB_Descriptor_String_t ProductString PROGMEM =
{
	Header:                 {Size: USB_STRING_LEN(17), Type: DTYPE_String},
		
	UnicodeString:          L"CC25xx programmer"
};

const USB_Descriptor_String_t SerialNumberString PROGMEM =
{
	Header:                 {Size: USB_STRING_LEN(1), Type: DTYPE_String},
		
	UnicodeString:          L"3"
};

bool USB_GetDescriptor(const uint16_t wValue, const uint8_t wIndex,
                       void** const DescriptorAddress, uint16_t* const DescriptorSize)
{
	void*    Address = NULL;
	uint16_t Size    = 0;

	switch (wValue >> 8)
	{
		case DTYPE_Device:
			Address = DESCRIPTOR_ADDRESS(DeviceDescriptor);
			Size    = sizeof(USB_Descriptor_Device_t);
			break;
		case DTYPE_Configuration:
			Address = DESCRIPTOR_ADDRESS(ConfigurationDescriptor);
			Size    = sizeof(USB_Descriptor_Configuration_t);
			break;
		case DTYPE_String:
			switch (wValue & 0xFF)
			{
				case 0x00:
					Address = DESCRIPTOR_ADDRESS(LanguageString);
					Size    = pgm_read_byte(&LanguageString.Header.Size);
					break;
				case 0x01:
					Address = DESCRIPTOR_ADDRESS(ManufacturerString);
					Size    = pgm_read_byte(&ManufacturerString.Header.Size);
					break;
				case 0x02:
					Address = DESCRIPTOR_ADDRESS(ProductString);
					Size    = pgm_read_byte(&ProductString.Header.Size);
					break;
				case 0x03:
					Address = DESCRIPTOR_ADDRESS(SerialNumberString);
					Size    = pgm_read_byte(&SerialNumberString.Header.Size);
					break;
			}
			
			break;
	}
	
	if (Address != NULL)
	{
		*DescriptorAddress = Address;
		*DescriptorSize    = Size;

		return true;
	}
		
	return false;
}
