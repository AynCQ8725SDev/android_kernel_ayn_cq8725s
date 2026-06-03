/*
 * Copyright (c) 2024, check mcu data. All rights reserved.
 */

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/ipc_logging.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/suspend.h>/*包含#include <linux/freezer.h>，freezer.h又包含"set_freezable"*/
#include <linux/ioctl.h>
#include <linux/pinctrl/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/kernel.h>/*printk()*/
#include <linux/sched.h>
#include <mydebug.h>


#define FRAME_POS_HEAD_1				0
#define FRAME_POS_HEAD_2				1
#define FRAME_POS_HEAD_3				2
#define FRAME_POS_HEAD_4				3
#define FRAME_POS_SEQ					4
#define FRAME_POS_CMD					5
#define FRAME_POS_LEN_LOW				6
#define FRAME_POS_LEN_HIGHT				7

#define FRAME_POS_DATA_0				8
#define FRAME_POS_DATA_1				9
#define FRAME_POS_DATA_2				10

#define FRAME_DATA_MIN					9

#define FRAME_HEAD_1					0xA5
#define FRAME_HEAD_2					0xD3
#define FRAME_HEAD_3					0x5A
#define FRAME_HEAD_4					0x3D

#define CMD_COMMOD						0x01
#define CMD_STATUS						0x02
#define CMD_IAP							0x03

#define USART_RX_BUFF_MAX				2048//20480
#define PRINT_DATA_BUFF_MAX				1024//4096



int dataLen(char* buf)
{
	return (char)( (buf[FRAME_POS_LEN_LOW]&0xff) | ((buf[FRAME_POS_LEN_HIGHT] >> 8)&0xFF) );
}

int checkSumDataLen(char* buf)
{
	return dataLen(buf) + 4; // data + seq length1 length2 cmd
}

int calcChecksum(char* buf)
{
	int index = 0;
	int sum = 0;// 定义为byte时，最高位变成符号，导致最高位为1时检验和为负数，变成检验不通过
	int len = checkSumDataLen(buf) + FRAME_POS_SEQ;
	for(index=FRAME_POS_SEQ; index<len; index++){
		sum ^= buf[index];
		//KERNEL_ERROR("sum=0x%x, buf[%d]=%x", (sum&0xFF), index, buf[index]);
	}
	//KERNEL_ERROR("sum=0x%x", (sum&0xFF));
	return (sum&0xFF);
}

int headFrontErr = 0;
int headErr = 0;
int checksumErr = 0;
int dataLenErr = 0;
EXPORT_SYMBOL(headFrontErr);
EXPORT_SYMBOL(headErr);
EXPORT_SYMBOL(checksumErr);
EXPORT_SYMBOL(dataLenErr);

void printfData(char errno, char *buffer, int len)
{
	static char data[PRINT_DATA_BUFF_MAX] = {0};
	int index = 0, pos = 0;
	for(index=0; index<len; index++){
		sprintf(data+pos, "%02x ", buffer[index]);
		pos += 3;
		if(pos >= PRINT_DATA_BUFF_MAX - 3){
			break;
		}
	}
	if(errno > 0){
		KERNEL_ERROR("errno=%d, length=%d, data=%s", errno, len, data);
	}
}

static char WORK_INTERVAL = 30;
static char buffer_ring[USART_RX_BUFF_MAX] = {0};
static int recvDataLen = 0;
static int procDataLen = 0;
void copyMcuData(char *buf, int length)
{
	// 接收数据
	if(length >= USART_RX_BUFF_MAX){
		KERNEL_ERROR("length=%d, too much data, bigger %d.", length, USART_RX_BUFF_MAX);
		recvDataLen = 0;
	}
	if(length + recvDataLen >= USART_RX_BUFF_MAX){
		KERNEL_ERROR("recvDataLen=%d, length=%d, Complete verification, bigger %d.", recvDataLen, length, USART_RX_BUFF_MAX);
		recvDataLen = 0;
	}
	memcpy(buffer_ring+recvDataLen, buf, length);

	recvDataLen += length;
	WORK_INTERVAL = 30;
	//printfData(1, buffer_ring, recvDataLen);
}
EXPORT_SYMBOL(copyMcuData);

static void processMcuData(void)
{
	char *p = buffer_ring;
	int len = recvDataLen;
	int checksum = 0;
	int pkt_len = 0;
	char printfEn = 0;

	if(recvDataLen == procDataLen){
		WORK_INTERVAL = 100;
		return;
	}

	// 需要处理的数据
	while(len >= FRAME_DATA_MIN){

		// find a packet HEAD
		while ( len >= FRAME_POS_HEAD_4 && \
			((p[FRAME_POS_HEAD_1] != FRAME_HEAD_1 || \
			  p[FRAME_POS_HEAD_2] != FRAME_HEAD_2 || \
			  p[FRAME_POS_HEAD_3] != FRAME_HEAD_3 || \
			  p[FRAME_POS_HEAD_4] != FRAME_HEAD_4))) {
				++p;
				--len;
				++headFrontErr;
				++procDataLen;
		}
		if(len <= 0){ // 数据查找完成，但没有找到帧头
			headErr++;
			printfEn = 1;
			KERNEL_ERROR("can`t find a packet HEAD\n");
			break;
		}

		if(len < FRAME_DATA_MIN){ // 数据查找完成，但数据不完整
			dataLenErr++;
			printfEn = 2;
			KERNEL_ERROR("The data is not complete frame. len=%d\n", len);
			break;
		}
		pkt_len = p[FRAME_POS_LEN_LOW] + (p[FRAME_POS_LEN_HIGHT] << 8);
		if(len < pkt_len + FRAME_DATA_MIN){
			dataLenErr++;
			printfEn = 3;
			KERNEL_ERROR("The data is not complete frame. len=%d, pkt_len=%d\n", len, pkt_len);
			break;
		}

		checksum = calcChecksum(p);
		if(p[pkt_len + FRAME_POS_DATA_0] == checksum){
			//KERNEL_ERROR("packet checksum Successful.");
			printfEn = 5;
			p += pkt_len + FRAME_DATA_MIN;
			len -= pkt_len + FRAME_DATA_MIN; // 数据减少一帧
			procDataLen += pkt_len + FRAME_DATA_MIN;
		} else {
			printfEn = 4;
			checksumErr++;
			p += FRAME_POS_HEAD_4;
			len -= FRAME_POS_HEAD_4; // 数据减少帧头
			procDataLen += FRAME_POS_HEAD_4;
			KERNEL_ERROR("packet checksum=%x(%x) failed, skip it, len=%d", checksum, p[pkt_len + FRAME_POS_DATA_0], pkt_len);
		}
		if(printfEn != 5){
			printfData(printfEn, buffer_ring, len);
			printfEn = 0;
		}
		if(procDataLen >= USART_RX_BUFF_MAX){
			procDataLen = 0;
		}
	}
}

static struct task_struct *detect_task = NULL;
static int readstatus_work(void *arg)
{
	KERNEL_ERROR("readstatus_work start!\n");
	set_freezable();

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(WORK_INTERVAL));

		if (kthread_freezable_should_stop(NULL))
			break;

		processMcuData();
	}

	KERNEL_ERROR("readstatus_work exit!\n");
	return 0;
}

void checkMcuRemove(void)
{
	KERNEL_ERROR("remove task");
	if(detect_task != NULL){
		kthread_stop(detect_task);
		detect_task = NULL;
	}
}
EXPORT_SYMBOL(checkMcuRemove);

void checkMcuInit(void)
{
	KERNEL_ERROR("init task");
	if(detect_task){
		KERNEL_ERROR("mcu detect: kthread_create init");
		return;
	}
	detect_task = kthread_run(readstatus_work, NULL, "mcu detect");
	if (IS_ERR(detect_task)) {
		KERNEL_ERROR("mcu detect: kthread_create failed");
		return;
	}
}
EXPORT_SYMBOL(checkMcuInit);