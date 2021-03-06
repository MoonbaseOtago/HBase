/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#include "HIDParser.h"

uint8_t ProcessHIDReport(const uint8_t* ReportData, uint16_t ReportSize, HID_ReportInfo_t* const ParserData)
{
	HID_StateTable_t  StateTable[HID_STATETABLE_STACK_DEPTH];
	HID_StateTable_t* CurrStateTable               = &StateTable[0];
	uint16_t          UsageStack[HID_USAGE_STACK_DEPTH];
	uint8_t           UsageStackSize               = 0;
	uint16_t          BitOffsetIn                  = 0x00;
	uint16_t          BitOffsetOut                 = 0x00;
#if defined(HID_ENABLE_FEATURE_PROCESSING)
	uint16_t          BitOffsetFeature             = 0x00;
#endif
	CollectionPath_t* CurrCollectionPath           = NULL;

	memset((void*)ParserData, 0x00, sizeof(HID_ReportInfo_t)); 

	while (ReportSize)
	{
		uint32_t ReportItemData = 0;
		
		switch (*ReportData & DATA_SIZE_MASK)
		{
			case DATA_SIZE_4:
				ReportItemData = *((uint32_t*)(ReportData + 1));
				break;
			case DATA_SIZE_2:
				ReportItemData = *((uint16_t*)(ReportData + 1));
				break;
			case DATA_SIZE_1:
				ReportItemData = *((uint8_t*)(ReportData + 1));
				break;
		}

		switch (*ReportData & (TYPE_MASK | TAG_MASK))
		{
			case (TYPE_GLOBAL | TAG_GLOBAL_PUSH):
				if (CurrStateTable == &StateTable[HID_STATETABLE_STACK_DEPTH])
				  return HID_PARSE_HIDStackOverflow;
	
				memcpy((CurrStateTable - 1),
				       CurrStateTable,
				       sizeof(HID_ReportItem_t));

				CurrStateTable++;
				break;
			case (TYPE_GLOBAL | TAG_GLOBAL_POP):
				if (CurrStateTable == &StateTable[0])
				  return HID_PARSE_HIDStackUnderflow;
				  
				CurrStateTable--;
				break;
			case (TYPE_GLOBAL | TAG_GLOBAL_USAGEPAGE):
				CurrStateTable->Attributes.Usage.Page       = ReportItemData;
				break;
			case (TYPE_GLOBAL | TAG_GLOBAL_LOGICALMIN):
				CurrStateTable->Attributes.Logical.Minimum  = ReportItemData;
				break;
			case (TYPE_GLOBAL | TAG_GLOBAL_LOGICALMAX):
				CurrStateTable->Attributes.Logical.Maximum  = ReportItemData;
				break;
			case (TYPE_GLOBAL | TAG_GLOBAL_PHYSMIN):
				CurrStateTable->Attributes.Physical.Minimum = ReportItemData;
				break;
			case (TYPE_GLOBAL | TAG_GLOBAL_PHYSMAX):
				CurrStateTable->Attributes.Physical.Maximum = ReportItemData;
				break;
			case (TYPE_GLOBAL | TAG_GLOBAL_UNITEXP):
				CurrStateTable->Attributes.Unit.Exponent    = ReportItemData;
				break;
			case (TYPE_GLOBAL | TAG_GLOBAL_UNIT):
				CurrStateTable->Attributes.Unit.Type        = ReportItemData;
				break;
			case (TYPE_GLOBAL | TAG_GLOBAL_REPORTSIZE):
				CurrStateTable->Attributes.BitSize          = ReportItemData;
				break;
			case (TYPE_GLOBAL | TAG_GLOBAL_REPORTCOUNT):
				CurrStateTable->ReportCount                 = ReportItemData;
				break;
			case (TYPE_LOCAL | TAG_LOCAL_USAGE):
				if (UsageStackSize == HID_USAGE_STACK_DEPTH)
				  return HID_PARSE_UsageStackOverflow;
			
				UsageStack[UsageStackSize++] = ReportItemData;
				break;
			case (TYPE_LOCAL | TAG_LOCAL_USAGEMIN):
				CurrStateTable->Attributes.Usage.MinMax.Minimum = ReportItemData;
				break;
			case (TYPE_LOCAL | TAG_LOCAL_USAGEMAX):
				CurrStateTable->Attributes.Usage.MinMax.Maximum = ReportItemData;
				break;
			case (TYPE_MAIN | TAG_MAIN_COLLECTION):
				if (CurrCollectionPath == NULL)
				{
					CurrCollectionPath = &ParserData->CollectionPaths[0];
				}
				else
				{
					CollectionPath_t* ParentCollectionPath = CurrCollectionPath;
			
					CurrCollectionPath = &ParserData->CollectionPaths[1];

					while (CurrCollectionPath->Parent != NULL);
					{
						if (CurrCollectionPath == &ParserData->CollectionPaths[HID_MAX_COLLECTIONS])
						  return HID_PARSE_InsufficientCollectionPaths;
					
						CurrCollectionPath++;
					}

					CurrCollectionPath->Parent = ParentCollectionPath;
				}
				
				CurrCollectionPath->Type = ReportItemData;
				
				if (UsageStackSize)
				{
					CurrCollectionPath->Usage = UsageStack[0];

					for (uint8_t i = 1; i <= UsageStackSize; i++)
					  UsageStack[i - 1] = UsageStack[i];
					  
					UsageStackSize--;
				}
				else
				{
					CurrCollectionPath->Usage = 0;
				}
				
				break;
			case (TYPE_MAIN | TAG_MAIN_ENDCOLLECTION):
				if (CurrCollectionPath == NULL)
				  return HID_PARSE_UnexpectedEndCollection;
		
				CurrCollectionPath = CurrCollectionPath->Parent;

				break;
			case (TYPE_MAIN | TAG_MAIN_INPUT):
			case (TYPE_MAIN | TAG_MAIN_OUTPUT):
#if defined(HID_ENABLE_FEATURE_PROCESSING)
			case (TYPE_MAIN | TAG_MAIN_FEATURE):
#endif
				for (uint8_t ReportItemNum = 0; ReportItemNum < CurrStateTable->ReportCount; ReportItemNum++)
				{
					HID_ReportItem_t* CurrReportItem = &ParserData->ReportItems[ParserData->TotalReportItems];
				
					if (ParserData->TotalReportItems == HID_MAX_REPORTITEMS)
					  return HID_PARSE_InsufficientReportItems;
				  
					memcpy(&CurrReportItem->Attributes,
					       &CurrStateTable->Attributes,
					       sizeof(HID_ReportItem_Attributes_t));

					CurrReportItem->ItemFlags      = ReportItemData;
					CurrReportItem->CollectionPath = CurrCollectionPath;

					if (UsageStackSize)
					{
						CurrReportItem->Attributes.Usage.Usage = UsageStack[0];

						for (uint8_t i = 1; i < UsageStackSize; i++)
						  UsageStack[i - 1] = UsageStack[i];
						  
						UsageStackSize--;
					}
					else
					{
						CurrReportItem->Attributes.Usage.Usage = 0;
					}
											
					switch (*ReportData & TAG_MASK)
					{
						case TAG_MAIN_INPUT:
							CurrReportItem->ItemType  = REPORT_ITEM_TYPE_In;
							CurrReportItem->BitOffset = BitOffsetIn;
								
							BitOffsetIn += CurrStateTable->Attributes.BitSize;
							
							break;
						case TAG_MAIN_OUTPUT:
							CurrReportItem->ItemType  = REPORT_ITEM_TYPE_Out;
							CurrReportItem->BitOffset = BitOffsetOut;
								
							BitOffsetOut += CurrStateTable->Attributes.BitSize;
							
							break;
#if defined(HID_ENABLE_FEATURE_PROCESSING)
						case TAG_MAIN_FEATURE:
							CurrReportItem->ItemType  = REPORT_ITEM_TYPE_Feature;						
							CurrReportItem->BitOffset = BitOffsetFeature;
								
							BitOffsetFeature += CurrStateTable->Attributes.BitSize;		

							break;
#endif
					}
					
#if !defined(HID_INCLUDE_CONSTANT_DATA_ITEMS)
					if (!(ReportItemData & IOF_CONSTANT))
					  ParserData->TotalReportItems++;
#else
					ParserData->TotalReportItems++;
#endif
				}
				
				UsageStackSize = 0;
				
				break;
		}
	  
		if ((*ReportData & TYPE_MASK) == TYPE_MAIN)
		{
			CurrStateTable->Attributes.Usage.MinMax.Minimum = 0;
			CurrStateTable->Attributes.Usage.MinMax.Maximum = 0;
			UsageStackSize = 0;
		}
		
		switch (*ReportData & DATA_SIZE_MASK)
		{
			case DATA_SIZE_4:
				ReportSize -= 5;
				ReportData += 5;
				break;
			case DATA_SIZE_2:
				ReportSize -= 3;
				ReportData += 3;
				break;
			case DATA_SIZE_1:
				ReportSize -= 2;
				ReportData += 2;
				break;
			case DATA_SIZE_0:
				ReportSize -= 1;
				ReportData += 1;
				break;
		}
	}
	
	return HID_PARSE_Sucessful;
}

void GetReportItemInfo(const uint8_t* ReportData, HID_ReportItem_t* const ReportItem)
{
	uint16_t DataBitsRem  = ReportItem->Attributes.BitSize;
	uint16_t CurrentBit   = ReportItem->BitOffset;
	uint32_t BitMask      = (1 << 0);

	ReportItem->Value = 0;

	while (DataBitsRem--)
	{
		if (ReportData[CurrentBit / 8] & (1 << (CurrentBit % 8)))
		  ReportItem->Value |= BitMask;
		
		CurrentBit++;
		BitMask <<= 1;
	}
}

void SetReportItemInfo(uint8_t* const ReportData, const HID_ReportItem_t* ReportItem)
{
	uint16_t DataBitsRem  = ReportItem->Attributes.BitSize;
	uint16_t CurrentBit   = ReportItem->BitOffset;
	uint32_t BitMask      = (1 << 0);

	while (DataBitsRem--)
	{
		if (ReportItem->Value & (1 << (CurrentBit % 8)))
		  ReportData[CurrentBit / 8] |= BitMask;

		CurrentBit++;
		BitMask <<= 1;
	}
}
