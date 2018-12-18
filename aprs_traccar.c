
#include <string.h>
#include <stdio.h>
#include <api_os.h>
#include <api_gps.h>
#include <api_event.h>
#include <api_hal_uart.h>
#include <api_debug.h>
#include "buffer.h"
#include "gps_parse.h"
#include "math.h"
#include "gps.h"
#include "api_hal_pm.h"
#include "time.h"
#include "api_info.h"
#include "assert.h"
#include "api_socket.h"
#include "api_network.h"
#include "api_hal_gpio.h"//goio的头文件
#include "api_hal_adc.h"//adc的头文件
/**
 * gps tracker, use an open source tracker server traccar:https://www.traccar.org/
 * the server in the code(`#define SERVER_IP   "ss.neucrack.com"`) may invalid someday, you can download the server and deploy youself
 * How to use: 
 *          compile and download to A9G dev board, open browser, access http://ss.neucrack.com:8082 ,
 *          then register and login, add devices and the number is IMEI e.g. `867959033006999`, finally the position of your device will be show in the map
 *          
 * @attention The code below just a demo, please read and check the code carefully before copy to your project directly(DO NOT copy directly)!!!!!!
 * 
 * 
 */
#define SERVER_IP   "118.31.*.*"
#define SERVER_PORT  5055
#define APRS_IP "202.141.*.*"
#define APRS_PORT 14580

#define GPS_NMEA_LOG_FILE_PATH "/t/gps_nmea.log"



#define MAIN_TASK_STACK_SIZE    (2048 * 2)
#define MAIN_TASK_PRIORITY      0
#define MAIN_TASK_NAME          "GPS Test Task"

static HANDLE gpsTaskHandle = NULL;
bool isGpsOn = true;
bool networkFlag = false;


// const uint8_t nmea[]="$GNGGA,000021.263,2228.7216,N,11345.5625,E,0,0,,153.3,M,-3.3,M,,*4E\r\n$GPGSA,A,1,,,,,,,,,,,,,,,*1E\r\n$BDGSA,A,1,,,,,,,,,,,,,,,*0F\r\n$GPGSV,1,1,00*79\r\n$BDGSV,1,1,00*68\r\n$GNRMC,000021.263,V,2228.7216,N,11345.5625,E,0.000,0.00,060180,,,N*5D\r\n$GNVTG,0.00,T,,M,0.000,N,0.000,K,N*2C\r\n";

void EventDispatch(API_Event_t* pEvent)
{
    switch(pEvent->id)
    {
        case API_EVENT_ID_NO_SIMCARD:
            Trace(10,"!!NO SIM CARD%d!!!!",pEvent->param1);
            networkFlag = false;
            break;
        case API_EVENT_ID_NETWORK_REGISTER_SEARCHING:
            Trace(2,"network register searching");
            networkFlag = false;
            break;
        case API_EVENT_ID_NETWORK_REGISTER_DENIED:
            Trace(2,"network register denied");
        case API_EVENT_ID_NETWORK_REGISTER_NO:
            Trace(2,"network register no");
            break;
        case API_EVENT_ID_GPS_UART_RECEIVED:
            // Trace(1,"received GPS data,length:%d, data:%s,flag:%d",pEvent->param1,pEvent->pParam1,flag);
            GPS_Update(pEvent->pParam1,pEvent->param1);
            break;
        case API_EVENT_ID_NETWORK_REGISTERED_HOME:
        case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
        {
            uint8_t status;
            Trace(2,"network register success");
            bool ret = Network_GetAttachStatus(&status);
            if(!ret)
                Trace(1,"get attach staus fail");
            Trace(1,"attach status:%d",status);
            if(status == 0)
            {
                ret = Network_StartAttach();
                if(!ret)
                {
                    Trace(1,"network attach fail");
                }
            }
            else
            {
                Network_PDP_Context_t context = {
                    .apn        ="cmnet",
                    .userName   = ""    ,
                    .userPasswd = ""
                };
                Network_StartActive(context);
            }
            break;
        }
        case API_EVENT_ID_NETWORK_ATTACHED:
            Trace(2,"network attach success");
            Network_PDP_Context_t context = {
                .apn        ="cmnet",
                .userName   = ""    ,
                .userPasswd = ""
            };
            Network_StartActive(context);
            break;

        case API_EVENT_ID_NETWORK_ACTIVATED:
            Trace(2,"network activate success");
            networkFlag = true;
            break;
        
        case API_EVENT_ID_UART_RECEIVED:
            if(pEvent->param1 == UART1)
            {
                uint8_t data[pEvent->param2+1];
                data[pEvent->param2] = 0;
                memcpy(data,pEvent->pParam1,pEvent->param2);
                Trace(1,"uart received data,length:%d,data:%s",pEvent->param2,data);
                if(strcmp(data,"close") == 0)
                {
                    Trace(1,"close gps");
                    GPS_Close();
                    isGpsOn = false;
                }
                else if(strcmp(data,"open") == 0)
                {
                    Trace(1,"open gps");
                    GPS_Open(NULL);
                    isGpsOn = true;
                }
            }
            break;
        default:
            break;
    }
}
//
int Http_Post_Udp(const char* domain, int port,const char* path,uint8_t* body, uint16_t bodyLen, char* retBuffer, int bufferLen)
{
    uint8_t ip[16];
    bool flag = false;
    uint16_t recvLen = 0;

    //connect server
    memset(ip,0,sizeof(ip));
    if(DNS_GetHostByName2(domain,ip) != 0)
    {
        Trace(2,"get ip error");
        return -1;
    }
    // Trace(2,"get ip success:%s -> %s",domain,ip);
    char* servInetAddr = ip;
    char* temp = OS_Malloc(2048);
    if(!temp)
    {
        Trace(2,"malloc fail");
        return -1;
    }
    snprintf(temp,2048,"BG4WLG-2>A9G:%s\r\n\r\n",path,domain,bodyLen);//拼接字符串
    char* pData = temp;
	//char* pData = "BG4WLG-2>A9G:=3645.72N/11704.51E=\r\n\r\n";
	//char* pData = path;
    int fd = socket(AF_INET, SOCK_STREAM, 1);//1 表示udp 0标识tcp
    if(fd < 0){
        Trace(2,"socket fail");
        OS_Free(temp);
        return -1;
    }
    // Trace(2,"fd:%d",fd);

    struct sockaddr_in sockaddr;
    memset(&sockaddr,0,sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(port);
    inet_pton(AF_INET,servInetAddr,&sockaddr.sin_addr);

    int ret = connect(fd, (struct sockaddr*)&sockaddr, sizeof(struct sockaddr_in));
    if(ret < 0){
        Trace(2,"socket connect fail");
        OS_Free(temp);
        return -1;
    }
    // Trace(2,"socket connect success");
    Trace(2,"send request:%s",pData);
    ret = send(fd, pData, strlen(pData), 0);
    if(ret < 0){
        Trace(2,"socket send fail");
        OS_Free(temp);
        return -1;
    }
    ret = send(fd, body, bodyLen, 0);
    if(ret < 0){
        Trace(2,"socket send fail");
        OS_Free(temp);
        return -1;
    }
    // Trace(2,"socket send success");

    struct fd_set fds;
    struct timeval timeout={12,0};
    FD_ZERO(&fds);
    FD_SET(fd,&fds);
    while(!flag)
    {
        ret = select(fd+1,&fds,NULL,NULL,&timeout);
        // Trace(2,"select return:%d",ret);
        switch(ret)
        {
            case -1:
                Trace(2,"select error");
                flag = true;
                break;
            case 0:
                Trace(2,"select timeout");
                flag = true;
                break;
            default:
                if(FD_ISSET(fd,&fds))
                {
                    memset(retBuffer,0,bufferLen);
                    ret = recv(fd,retBuffer,bufferLen,0);
                    recvLen += ret;
                    if(ret < 0)
                    {
                        Trace(2,"recv error");
                        flag = true;
                        break;
                    }
                    else if(ret == 0)
                    {
                        Trace(2,"ret == 0");
                        break;
                    }
                    else if(ret < 1352)
                    {
                        GPS_DEBUG_I("recv len:%d,data:%s",recvLen,retBuffer);
                        close(fd);
                        OS_Free(temp);
                        return recvLen;
                    }
                }
                break;
        }
    }
    close(fd);
    OS_Free(temp);
    return -1;
}
//http post with no header
int Http_Post(const char* domain, int port,const char* path,uint8_t* body, uint16_t bodyLen, char* retBuffer, int bufferLen)
{
    uint8_t ip[16];
    bool flag = false;
    uint16_t recvLen = 0;

    //connect server
    memset(ip,0,sizeof(ip));
    if(DNS_GetHostByName2(domain,ip) != 0)
    {
        Trace(2,"get ip error");
        return -1;
    }
    // Trace(2,"get ip success:%s -> %s",domain,ip);
    char* servInetAddr = ip;
    char* temp = OS_Malloc(2048);
    if(!temp)
    {
        Trace(2,"malloc fail");
        return -1;
    }
    snprintf(temp,2048,"POST %s HTTP/1.1\r\nContent-Type: application/x-www-form-urlencoded\r\nConnection: Keep-Alive\r\nHost: %s\r\nContent-Length: %d\r\n\r\n",
                            path,domain,bodyLen);
    char* pData = temp;
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(fd < 0){
        Trace(2,"socket fail");
        OS_Free(temp);
        return -1;
    }
    // Trace(2,"fd:%d",fd);

    struct sockaddr_in sockaddr;
    memset(&sockaddr,0,sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(port);
    inet_pton(AF_INET,servInetAddr,&sockaddr.sin_addr);

    int ret = connect(fd, (struct sockaddr*)&sockaddr, sizeof(struct sockaddr_in));
    if(ret < 0){
        Trace(2,"socket connect fail");
        OS_Free(temp);
        return -1;
    }
    // Trace(2,"socket connect success");
    Trace(2,"send request:%s",pData);
    ret = send(fd, pData, strlen(pData), 0);
    if(ret < 0){
        Trace(2,"socket send fail");
        OS_Free(temp);
        return -1;
    }
    ret = send(fd, body, bodyLen, 0);
    if(ret < 0){
        Trace(2,"socket send fail");
        OS_Free(temp);
        return -1;
    }
    // Trace(2,"socket send success");

    struct fd_set fds;
    struct timeval timeout={12,0};
    FD_ZERO(&fds);
    FD_SET(fd,&fds);
    while(!flag)
    {
        ret = select(fd+1,&fds,NULL,NULL,&timeout);
        // Trace(2,"select return:%d",ret);
        switch(ret)
        {
            case -1:
                Trace(2,"select error");
                flag = true;
                break;
            case 0:
                Trace(2,"select timeout");
                flag = true;
                break;
            default:
                if(FD_ISSET(fd,&fds))
                {
                    memset(retBuffer,0,bufferLen);
                    ret = recv(fd,retBuffer,bufferLen,0);
                    recvLen += ret;
                    if(ret < 0)
                    {
                        Trace(2,"recv error");
                        flag = true;
                        break;
                    }
                    else if(ret == 0)
                    {
                        Trace(2,"ret == 0");
                        break;
                    }
                    else if(ret < 1352)
                    {
                        GPS_DEBUG_I("recv len:%d,data:%s",recvLen,retBuffer);
                        close(fd);
                        OS_Free(temp);
                        return recvLen;
                    }
                }
                break;
        }
    }
    close(fd);
    OS_Free(temp);
    return -1;
}
char* GetDoubleStr(double value)
{
    char buf[32]={0};//长度可以自定义
    sprintf(buf,"%.8f",value);//保留8位小数，不够补0
    int index = 0;
    int len = strlen(buf);
    for(int i = len-1;i>0;i--)
    {
        if(buf[i] == '0')
            continue;
        else
        {
            if(buf[i] == '.') index = i;           
            else index = i+1;
            break;
        }
    }
    buf[index] = '\0';
    return buf;
}

uint8_t buffer[1024],buffer2[400];
uint8_t aprsbuf[1024],aprsStr[9],aprsStr1[8],aprsStr2[8];
double oldLon=0.0,oldLat=0.0;
bool latFlag = true,lonFlag = true,timeFlag = true;
//char buf[20];
int time1=0,time2=0;
void gps_testTask(void *pData)
{
    GPS_Info_t* gpsInfo = Gps_GetInfo();
    

    while(!networkFlag)
    {
        Trace(1,"wait for gprs regiter complete");
        OS_Sleep(2000);
    }

    //open GPS hardware(UART2 open either)
    GPS_Init();
    GPS_SaveLog(true,GPS_NMEA_LOG_FILE_PATH);
    // if(!GPS_ClearLog())
    //     Trace(1,"open file error, please check tf card");
    GPS_Open(NULL);

    //wait for gps start up, or gps will not response command
    while(gpsInfo->rmc.latitude.value == 0)
        OS_Sleep(1000);
    

    // set gps nmea output interval
    for(uint8_t i = 0;i<5;++i)
    {
        bool ret = GPS_SetOutputInterval(10000);
        Trace(1,"set gps ret:%d",ret);
        if(ret)
            break;
        OS_Sleep(1000);
    }
    
    if(!GPS_GetVersion(buffer,150))
        Trace(1,"get gps firmware version fail");
    else
        Trace(1,"gps firmware version:%s",buffer);

    // if(!GPS_SetFixMode(GPS_FIX_MODE_LOW_SPEED))
        // Trace(1,"set fix mode fail");

    if(!GPS_SetOutputInterval(1000))
        Trace(1,"set nmea output interval fail");
    
    Trace(1,"init ok");
	/**
	*gpio  配置
	*/
	GPIO_LEVEL statusNow;//获取阵脚的高低电平0、1
	GPIO_config_t gpioLedBlue = {
        .mode         = GPIO_MODE_INPUT,//设置为输入模式
        .pin          = GPIO_PIN25,//设置为25号阵脚
        .defaultLevel = GPIO_LEVEL_LOW//阵脚默认电平：低电平
    };
	PM_PowerEnable(GPIO_PIN25,true);//开启阵脚功能，必须在初始化之前执行
	GPIO_Init(gpioLedBlue);//初始化阵脚。
	
	//ADC配置
	uint16_t value = 0, mV = 0;//value为读取到的数值，mv为读取到的电压
    ADC_Config_t config = {
        .channel = ADC_CHANNEL_0,//设置通道为0
        .samplePeriod = ADC_SAMPLE_PERIOD_100MS//设置采样周期为100ms
    };
    ADC_Init(config);//初始化配置
    while(1)
    {
        if(isGpsOn)
        {
			time1 = time(NULL);
			//memset(timeBuf,0,sizeof(timeBuf));
			//snprintf(timeBuf,sizeof(timeBuf),"%d",time1);
			
			/* itoa(time1,buf,10);
			UART_Write(UART1,"lon-tim-",8);	
			UART_Write(UART1,buf,strlen(buf));
			UART_Write(UART1,"\r\n",2); */	
            //show fix info
            uint8_t isFixed = gpsInfo->gsa[0].fix_type > gpsInfo->gsa[1].fix_type ?gpsInfo->gsa[0].fix_type:gpsInfo->gsa[1].fix_type;
            char* isFixedStr;            
            if(isFixed == 2)
                isFixedStr = "2D fix";
            else if(isFixed == 3)
            {
                if(gpsInfo->gga.fix_quality == 1)
                    isFixedStr = "3D fix";
                else if(gpsInfo->gga.fix_quality == 2)
                    isFixedStr = "3D/DGPS fix";
            }
            else
                isFixedStr = "no fix";

            //convert unit ddmm.mmmm to degree(°) 
            int temp = (int)(gpsInfo->rmc.latitude.value/gpsInfo->rmc.latitude.scale/100);
            double latitude = temp+(double)(gpsInfo->rmc.latitude.value - temp*gpsInfo->rmc.latitude.scale*100)/gpsInfo->rmc.latitude.scale/60.0;
            temp = (int)(gpsInfo->rmc.longitude.value/gpsInfo->rmc.longitude.scale/100);
            double longitude = temp+(double)(gpsInfo->rmc.longitude.value - temp*gpsInfo->rmc.longitude.scale*100)/gpsInfo->rmc.longitude.scale/60.0;		
			
			GPIO_Get(GPIO_PIN25,&statusNow);//获取针脚的高低电平
            if(ADC_Read(ADC_CHANNEL_0, &value, &mV))//获取adc的数值
			{
				value = value;
				mV = mV;
			}
			//int aprsint = (int)(gpsInfo->rmc.longitude.value/gpsInfo->rmc.longitude.scale/100);
			double aprsLon = (double)(((int)(gpsInfo->rmc.longitude.value/100.0))/100.0);
			double aprsLat = (double)(((int)(gpsInfo->rmc.latitude.value/100.0))/100.0);
			//判断整数部分是否大于100
			if(aprsLon>=100.0)
			{
				//经度
				memset(aprsStr,0,sizeof(aprsStr));
				snprintf(aprsStr,sizeof(aprsStr),"%f",aprsLon);
				//纬度
				memset(aprsStr1,0,sizeof(aprsStr1));
				snprintf(aprsStr1,sizeof(aprsStr1),"%f",aprsLat);				
				snprintf(aprsbuf,sizeof(aprsbuf),"=%sN/%sE=ADCVal:%d,ADCmV:%d,GPIO:%d",aprsStr1,aprsStr,value,mV,statusNow);
			}
			if(aprsLon<100.0)
			{
				//经度
				memset(aprsStr2,0,sizeof(aprsStr2));
				snprintf(aprsStr2,sizeof(aprsStr2),"%f",aprsLon);
				//纬度
				memset(aprsStr1,0,sizeof(aprsStr1));
				snprintf(aprsStr1,sizeof(aprsStr1),"%f",aprsLat);				
				snprintf(aprsbuf,sizeof(aprsbuf),"=%sN/%sE=ADCVal:%d,ADCmV:%d,GPIO:%d",aprsStr1,aprsStr2,value,mV,statusNow);
				
			}
			
			UART_Write(UART1,"lon-apr-",8);	
			UART_Write(UART1,aprsbuf,strlen(aprsbuf));
            UART_Write(UART1,"\r\n\r\n",4);
            snprintf(buffer,sizeof(buffer),"GPS fix mode:%d, BDS fix mode:%d, fix quality:%d, satellites tracked:%d, gps sates total:%d, is fixed:%s, coordinate:WGS84, Latitude:%f, Longitude:%f, unit:degree,altitude:%f&gm:%d&adcVal:%d&adcmV:%d",gpsInfo->gsa[0].fix_type, gpsInfo->gsa[1].fix_type,
                                                                gpsInfo->gga.fix_quality,gpsInfo->gga.satellites_tracked, gpsInfo->gsv[0].total_sats, isFixedStr, latitude,longitude,gpsInfo->gga.altitude,statusNow,value,mV);
            //show in tracer
            Trace(1,buffer);
            //send to UART1
            UART_Write(UART1,buffer,strlen(buffer));
            UART_Write(UART1,"\r\n\r\n",4);
			
            char* requestPath = buffer2;
            uint8_t percent;
            uint16_t v = PM_Voltage(&percent);
            Trace(1,"power:%d %d",v,percent);
            memset(buffer,0,sizeof(buffer));
            if(!INFO_GetIMEI(buffer))
                Assert(false,"NO IMEI");
            Trace(1,"device name:%s",buffer);
            snprintf(requestPath,sizeof(buffer2),"/?id=%d&lat=%f&lon=%f&speed=%f&bearing=%.1f&altitude=%f&accuracy=%.1f&batt=%.1f&gm=%d&adcVal:%d&adcmV:%d",
                                                    1234,latitude,longitude,isFixed*1.0,0.0,gpsInfo->gga.altitude,0.0,percent*1.0,statusNow,value,mV);
			if(fabs(oldLat-latitude)>0.0001)
			{
				oldLat = latitude;
				latFlag = true;
			}else{latFlag = false;}
			if(fabs(oldLon-longitude)>0.0001)
			{
				oldLon = longitude;
				lonFlag = true;
			}else{lonFlag = false;}
			if(time1-time2>=176)
			{
				timeFlag = true;
			}else{timeFlag = false;}
			if(latFlag||lonFlag||timeFlag)
			{
				time2 = time(NULL);
				if(Http_Post(SERVER_IP,SERVER_PORT,requestPath,NULL,0,buffer,sizeof(buffer)) <0 )
                Trace(1,"send location to server fail");
				else
				{
					Trace(1,"send location to server success");
					Trace(1,"response:%s",buffer);
				}
				if(Http_Post_Udp(APRS_IP,APRS_PORT,aprsbuf,NULL,0,aprsbuf,sizeof(aprsbuf)) <0 )
					Trace(1,"send location to server fail");
				else
				{
					Trace(1,"send location to server success");
					Trace(1,"response:%s",buffer);
				}
			}
            
        }

        OS_Sleep(10000);
    }
}


void gps_MainTask(void *pData)
{
    API_Event_t* event=NULL;
    
    TIME_SetIsAutoUpdateRtcTime(true);
    
    //open UART1 to print NMEA infomation
    UART_Config_t config = {
        .baudRate = UART_BAUD_RATE_115200,
        .dataBits = UART_DATA_BITS_8,
        .stopBits = UART_STOP_BITS_1,
        .parity   = UART_PARITY_NONE,
        .rxCallback = NULL,
        .useEvent   = true
    };
    UART_Init(UART1,config);

    //Create UART1 send task and location print task
    OS_CreateTask(gps_testTask,
            NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);

    //Wait event
    while(1)
    {
        if(OS_WaitEvent(gpsTaskHandle, (void**)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}


void gps_tracker_Main(void)
{
    gpsTaskHandle = OS_CreateTask(gps_MainTask,
        NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&gpsTaskHandle);
}

