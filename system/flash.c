#include "flash.h"

//��FLASH�ж�ȡһ���ֽ�
u32 STMFLASH_ReadWord(u32 addr){
	
		return *(vu32*)addr;

}

//��ȡĳ����ַ���ڵ�����
uint16_t STMFLASH_GetFlastSector(u32 addr){

		if(addr<ADDR_FLASH_SECTOR_1) return FLASH_Sector_0;
		else if(addr<ADDR_FLASH_SECTOR_2) return  FLASH_Sector_1;
		else if(addr<ADDR_FLASH_SECTOR_3) return  FLASH_Sector_2;
		else if(addr<ADDR_FLASH_SECTOR_4) return  FLASH_Sector_3;
		else if(addr<ADDR_FLASH_SECTOR_5) return  FLASH_Sector_4;
		else if(addr<ADDR_FLASH_SECTOR_6) return  FLASH_Sector_5;
		else if(addr<ADDR_FLASH_SECTOR_7) return  FLASH_Sector_6;
		else if(addr<ADDR_FLASH_SECTOR_8) return  FLASH_Sector_7;
		else if(addr<ADDR_FLASH_SECTOR_9) return  FLASH_Sector_8;
		else if(addr<ADDR_FLASH_SECTOR_10) return  FLASH_Sector_9;
		else if(addr<ADDR_FLASH_SECTOR_11) return  FLASH_Sector_10;
		else return FLASH_Sector_11;


}

//��ָ����ַд��ָ�����ݵĳ���

void STMFLASH_Write(u32 WriteAddr,u32 *	pBuffer,u32 NumToWrite){
	
			FLASH_Status status=FLASH_COMPLETE;
			u32 addrx=0;
			u32 endaddr=0;
		
			if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return; //����ĵ�ַ
			FLASH_Unlock();
			FLASH_DataCacheCmd(DISABLE);
			
			addrx=WriteAddr;                        //��ʼд�ĵ�ַ
			endaddr=WriteAddr+NumToWrite*4;					//����д�ĵ�ַ
	
			if(addrx<0x1FFF0000){                    //���洢������Ҫ����
				
				while(addrx<endaddr){
					
						if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)
						{
							status=FLASH_EraseSector(STMFLASH_GetFlastSector(addrx),VoltageRange_3);
							if(status!=FLASH_COMPLETE)break;
			
						}else addrx+=4;
					}
				
				
				
				}
			if(status==FLASH_COMPLETE){
					
				while(WriteAddr<endaddr){
				
						if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE){
									break;
						}
						else WriteAddr+=4;pBuffer++;
				
				}	
			
			
			}
				
			FLASH_DataCacheCmd(ENABLE);
			FLASH_Lock();
			
			}

//��ָ��λ�ö�����
void STMFLASH_Read(u32 addr,u32 *pbuffer,u32 NumToRead){

		u32 i;
	  for(i=0;i<NumToRead;i++){
			pbuffer[i]=STMFLASH_ReadWord(addr);
			addr+=4;
		}
}			



