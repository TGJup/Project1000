#ifndef __Serial_MSG_H
#define __Serial_MSG_H

#define FC_Packet_Head1      0xEB     
#define FC_Packet_Head2      0xFE     
#define FC_Packet_Len        19       
#define FC_Packet_Check_Len  18    
#define FC_Address           0x92
#define FC_Rsp_Len           6
#define FC_Send_Cycle        80
#define FC_Zero_Time         3000

#define EP_Packet_Head1      0xA5     
#define EP_Packet_Head2      0x41    
#define EP_Packet_Len        13      
#define EP_CMD_0x90_ASK      0x90
#define EP_CMD_0x98_ASK      0x98

#define EP_Packet_Check_Len  12       
#define EP_12V_Address       0x01
#define EP_28V_Address       0x02
#define EP_Rsp_Len           13
#define EP_Report_ADR        0x01
#define EP_Send_Cycle        30

#define EP_GET_UI            0U
#define EP_GET_STATUS        1U

/*EP_Voltage_Status*/
 
#define BIT_TOTAL_OVER_1   0x10  
#define BIT_TOTAL_OVER_2   0x20  
#define BIT_TOTAL_UNDER_1  0x40  
#define BIT_TOTAL_UNDER_2  0x80  

#define MASK_OP    (BIT_TOTAL_OVER_1 | BIT_TOTAL_OVER_2)    
#define MASK_UP    (BIT_TOTAL_UNDER_1 | BIT_TOTAL_UNDER_2)  
/*EP_Current_Status*/
#define BIT_CHAR_OVERCUR_1    0x01  
#define BIT_CHAR_OVERCUR_2    0x02  
#define BIT_DIS_OVERCUR_1     0x04  
#define BIT_DIS_OVERCUR_2     0x08  

#define MASK_CHAR_OVERCUR    (BIT_CHAR_OVERCUR_1 | BIT_CHAR_OVERCUR_2) 
#define MASK_DIS_OVERCUR     (BIT_DIS_OVERCUR_1 | BIT_DIS_OVERCUR_2)   
#define MASK_ALL_OVERCUR     (MASK_CHAR_OVERCUR | MASK_DIS_OVERCUR)    

#endif
