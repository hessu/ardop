// TCP KISS server for ardopcf
//
// Provides a TCP server that speaks the KISS protocol
// (https://www.ax25.net/kiss.aspx) so that external AX.25/APRS software can
// use ardopcf as a simple FEC modem.  KISS data frames received from a
// connected client are decapsulated and the raw AX.25 frame they contain is
// transmitted using the ARDOP FEC modulator.  AX.25 frames decoded from
// received ARDOP FEC frames are KISS encapsulated and sent to all connected
// clients.
//
// Each KISS data frame is carried in a single ARDOP FEC frame ("stuff the
// AX.25 frame in the ARDOP frame").  The configured FEC mode (FECMODE) must
// therefore be large enough to hold the largest packet to be sent.  If a
// packet does not fit in a single FEC frame, the FEC layer will split it over
// multiple frames, but the receiving side has no way to recombine them, so
// such packets would be delivered to KISS clients as separate fragments.

#include <stdbool.h>

#ifdef WIN32
#define _CRT_SECURE_NO_DEPRECATE
#include <windows.h>
#pragma comment(lib, "WS2_32.Lib")
#define ioctl ioctlsocket
#else
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#define SOCKET int
#define INVALID_SOCKET (SOCKET)(~0)
#define SOCKET_ERROR (-1)
#define WSAGetLastError() errno
#define closesocket close
#endif

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#include "os_util.h"
#include "ARDOPC.h"
#include "ardopcommon.h"

// KISS special characters
#define FEND  0xC0  // Frame End
#define FESC  0xDB  // Frame Escape
#define TFEND 0xDC  // Transposed Frame End
#define TFESC 0xDD  // Transposed Frame Escape

#define KISS_MAX_CLIENTS 16
// Generous buffer.  Larger than any single FEC frame payload, with margin for
// KISS escaping.
#define KISS_BUFSIZE 2048

// Configuration (set by KISSConfig() from the --kiss command line option)
int KISSPort = 0;  // 0 means KISS server disabled
char KISSAddr[64] = "";  // empty means listen on loopback (127.0.0.1) only

extern char strFECMode[];
extern int FECRepeats;
extern const short FrameSize[256];
unsigned char FrameCode(char * bytFrameType);
bool StartFEC(UCHAR * bytData, int Len, char * strDataMode, int intRepeats, bool blnSendID);

static SOCKET KISSListenSock = INVALID_SOCKET;
static SOCKET KISSClients[KISS_MAX_CLIENTS];

// Per-client KISS decoder state
static UCHAR KISSRxFrame[KISS_MAX_CLIENTS][KISS_BUFSIZE];
static int KISSRxLen[KISS_MAX_CLIENTS];
static bool KISSInEsc[KISS_MAX_CLIENTS];

// Parse the --kiss argument, which is "[addr:]port".  If addr is omitted, the
// server listens on loopback only.  Returns true on success.
bool KISSConfig(const char *arg)
{
	if (arg == NULL || arg[0] == 0x00)
		return false;

	const char *colon = strrchr(arg, ':');
	if (colon != NULL)
	{
		int n = (int)(colon - arg);
		if (n >= (int)sizeof(KISSAddr))
			n = sizeof(KISSAddr) - 1;
		memcpy(KISSAddr, arg, n);
		KISSAddr[n] = 0x00;
		KISSPort = atoi(colon + 1);
	}
	else
	{
		KISSAddr[0] = 0x00;
		KISSPort = atoi(arg);
	}

	if (KISSPort <= 0 || KISSPort > 65535)
	{
		KISSPort = 0;
		return false;
	}
	return true;
}

// Open the listening socket.  Returns true on success.
bool KISSInit()
{
	struct sockaddr_in local_sin;
	u_long param = 1;

	for (int i = 0; i < KISS_MAX_CLIENTS; i++)
	{
		KISSClients[i] = INVALID_SOCKET;
		KISSRxLen[i] = 0;
		KISSInEsc[i] = false;
	}

	if (KISSPort == 0)
		return false;

	// The KISS bridge maps one AX.25 frame to one ARDOP FEC frame.  If the
	// configured FEC mode cannot carry at least 256 bytes in a single frame,
	// AX.25 packets would be fragmented across multiple FEC frames, which the
	// receiver cannot reassemble.  Treat this as a hard failure.
	char FullType[20];
	snprintf(FullType, sizeof(FullType), "%s.E", strFECMode);
	int fc = FrameCode(FullType);
	if (fc <= 0 || FrameSize[fc] < 256)
	{
		ZF_LOGE("KISS: FEC mode %s carries only %d bytes per frame, but at"
			" least 256 are required for KISS operation.  KISS server not"
			" started.  Set a larger FECMODE (e.g. 16QAM.500.100 = 256 bytes,"
			" 4FSK.2000.600 = 600 bytes, or 16QAM.2000.100 = 1024 bytes).",
			strFECMode, fc > 0 ? FrameSize[fc] : 0);
		KISSPort = 0;
		return false;
	}

#ifdef WIN32
	{
		WSADATA WsaData;
		WSAStartup(MAKEWORD(2, 0), &WsaData);
	}
#endif

	memset(&local_sin, 0, sizeof(local_sin));
	local_sin.sin_family = AF_INET;
	local_sin.sin_port = htons(KISSPort);
	if (KISSAddr[0] == 0x00)
		local_sin.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	else
	{
		local_sin.sin_addr.s_addr = inet_addr(KISSAddr);
		if (local_sin.sin_addr.s_addr == INADDR_NONE)
		{
			ZF_LOGE("KISS: invalid listen address \"%s\".  KISS server not"
				" started.", KISSAddr);
			KISSPort = 0;
			return false;
		}
	}

	KISSListenSock = socket(AF_INET, SOCK_STREAM, 0);
	if (KISSListenSock == INVALID_SOCKET)
	{
		ZF_LOGE("KISS: socket() failed error %d", WSAGetLastError());
		KISSPort = 0;
		return false;
	}

	setsockopt(KISSListenSock, SOL_SOCKET, SO_REUSEADDR, (char *)&param, sizeof(param));

	if (bind(KISSListenSock, (struct sockaddr *)&local_sin, sizeof(local_sin)) == SOCKET_ERROR)
	{
		ZF_LOGE("KISS: bind() failed for %s:%d error %d.  KISS server not"
			" started.",
			KISSAddr[0] ? KISSAddr : "127.0.0.1", KISSPort, WSAGetLastError());
		closesocket(KISSListenSock);
		KISSListenSock = INVALID_SOCKET;
		KISSPort = 0;
		return false;
	}

	if (listen(KISSListenSock, 4) == SOCKET_ERROR)
	{
		ZF_LOGE("KISS: listen() failed error %d.  KISS server not started.",
			WSAGetLastError());
		closesocket(KISSListenSock);
		KISSListenSock = INVALID_SOCKET;
		KISSPort = 0;
		return false;
	}

	ioctl(KISSListenSock, FIONBIO, &param);

	ZF_LOGI("KISS: listening for TCP KISS connections on %s:%d (FEC mode %s,"
		" %d bytes/frame)",
		KISSAddr[0] ? KISSAddr : "127.0.0.1", KISSPort, strFECMode, FrameSize[fc]);

	return true;
}

// Transmit one decapsulated AX.25 frame using ARDOP FEC.
static void KISSTransmit(UCHAR *axdata, int len)
{
	if (len <= 0)
		return;

	// Guard against a runtime FECMODE change to a frame too small to hold this
	// packet in a single FEC frame.  Dropping it is preferable to transmitting
	// a fragmented frame the receiver cannot reassemble.
	char FullType[20];
	snprintf(FullType, sizeof(FullType), "%s.E", strFECMode);
	int fc = FrameCode(FullType);
	if (fc <= 0 || FrameSize[fc] < len)
	{
		ZF_LOGE("KISS: dropping %d byte AX.25 frame; current FEC mode %s holds"
			" only %d bytes per frame and the frame would be fragmented.",
			len, strFECMode, fc > 0 ? FrameSize[fc] : 0);
		return;
	}

	ZF_LOGI("KISS: transmitting %d byte AX.25 frame via FEC (%s)", len, strFECMode);

	// Send without an ARDOP IDFrame.  The source callsign is carried within the
	// AX.25 frame, and an ARDOP IDFrame would be meaningless to AX.25 clients.
	if (!StartFEC(axdata, len, strFECMode, FECRepeats, false))
		ZF_LOGW("KISS: StartFEC() failed for received KISS frame.");
}

// Process one complete (already de-escaped) KISS frame.
static void KISSProcessFrame(UCHAR *frame, int len)
{
	if (len < 1)
		return;  // Need at least the type/command byte

	UCHAR cmd = frame[0] & 0x0F;  // low nibble is the command, high nibble port

	if (cmd == 0x00)
	{
		// Data frame.  The remainder is the raw AX.25 frame.
		KISSTransmit(frame + 1, len - 1);
	}
	else
	{
		// TXDELAY, PERSIST, SLOTTIME, TXTAIL, FULLDUPLEX, SETHARDWARE, RETURN.
		// These TNC parameters do not apply to ardopcf, so they are ignored.
		ZF_LOGD("KISS: ignoring non-data KISS command 0x%02X", cmd);
	}
}

// Feed received bytes from a client through the KISS decoder.
static void KISSDecode(int idx, UCHAR *data, int len)
{
	for (int i = 0; i < len; i++)
	{
		UCHAR b = data[i];

		if (b == FEND)
		{
			if (KISSRxLen[idx] > 0)
				KISSProcessFrame(KISSRxFrame[idx], KISSRxLen[idx]);
			KISSRxLen[idx] = 0;
			KISSInEsc[idx] = false;
			continue;
		}

		if (KISSInEsc[idx])
		{
			KISSInEsc[idx] = false;
			if (b == TFEND)
				b = FEND;
			else if (b == TFESC)
				b = FESC;
			// else protocol violation; store the byte as-is and continue
		}
		else if (b == FESC)
		{
			KISSInEsc[idx] = true;
			continue;
		}

		if (KISSRxLen[idx] < KISS_BUFSIZE)
			KISSRxFrame[idx][KISSRxLen[idx]++] = b;
		else
		{
			// Frame too long; discard it and resynchronise on the next FEND.
			ZF_LOGW("KISS: oversize frame from client, discarding.");
			KISSRxLen[idx] = 0;
			KISSInEsc[idx] = false;
		}
	}
}

static void KISSCloseClient(int idx)
{
	if (KISSClients[idx] != INVALID_SOCKET)
	{
		closesocket(KISSClients[idx]);
		KISSClients[idx] = INVALID_SOCKET;
	}
	KISSRxLen[idx] = 0;
	KISSInEsc[idx] = false;
}

// Accept new connections and read data from connected clients.  Non-blocking.
void KISSPoll()
{
	u_long param = 1;
	UCHAR buf[KISS_BUFSIZE];

	if (KISSListenSock == INVALID_SOCKET)
		return;

	// Accept any pending connections.
	while (true)
	{
		struct sockaddr_in sin;
		int addrlen = sizeof(sin);
		SOCKET newsock = accept(KISSListenSock, (struct sockaddr *)&sin, &addrlen);

		if (newsock == INVALID_SOCKET)
			break;  // EWOULDBLOCK or no pending connection

		int slot = -1;
		for (int i = 0; i < KISS_MAX_CLIENTS; i++)
		{
			if (KISSClients[i] == INVALID_SOCKET)
			{
				slot = i;
				break;
			}
		}
		if (slot == -1)
		{
			ZF_LOGW("KISS: too many clients, rejecting new connection.");
			closesocket(newsock);
			continue;
		}

		ioctl(newsock, FIONBIO, &param);
		KISSClients[slot] = newsock;
		KISSRxLen[slot] = 0;
		KISSInEsc[slot] = false;
		ZF_LOGI("KISS: client connected from %s:%d (slot %d)",
			inet_ntoa(sin.sin_addr), ntohs(sin.sin_port), slot);
	}

	// Read from connected clients.
	for (int i = 0; i < KISS_MAX_CLIENTS; i++)
	{
		if (KISSClients[i] == INVALID_SOCKET)
			continue;

		int len = recv(KISSClients[i], (char *)buf, sizeof(buf), 0);
		if (len > 0)
		{
			KISSDecode(i, buf, len);
		}
		else if (len == 0)
		{
			ZF_LOGI("KISS: client (slot %d) disconnected.", i);
			KISSCloseClient(i);
		}
		else
		{
			// len < 0: error or no data available
#ifdef WIN32
			if (WSAGetLastError() != WSAEWOULDBLOCK)
#else
			if (errno != EWOULDBLOCK && errno != EAGAIN)
#endif
			{
				ZF_LOGI("KISS: client (slot %d) read error, closing.", i);
				KISSCloseClient(i);
			}
		}
	}
}

// KISS encapsulate an AX.25 frame and send it to all connected clients.
// Called from the FEC receive path with a successfully decoded frame.
void KISSSendToClients(UCHAR *axdata, int len)
{
	if (KISSListenSock == INVALID_SOCKET || len <= 0)
		return;

	// Build the KISS frame: FEND, type byte (0x00 = data, port 0), escaped
	// payload, FEND.  Worst case each payload byte expands to two bytes.
	UCHAR out[2 * KISS_BUFSIZE + 3];
	int o = 0;

	out[o++] = FEND;
	out[o++] = 0x00;  // data frame, port 0
	for (int i = 0; i < len && o < (int)sizeof(out) - 2; i++)
	{
		UCHAR b = axdata[i];
		if (b == FEND)
		{
			out[o++] = FESC;
			out[o++] = TFEND;
		}
		else if (b == FESC)
		{
			out[o++] = FESC;
			out[o++] = TFESC;
		}
		else
			out[o++] = b;
	}
	out[o++] = FEND;

	for (int i = 0; i < KISS_MAX_CLIENTS; i++)
	{
		if (KISSClients[i] == INVALID_SOCKET)
			continue;
		if (send(KISSClients[i], (char *)out, o, MSG_NOSIGNAL) == SOCKET_ERROR)
		{
#ifdef WIN32
			if (WSAGetLastError() != WSAEWOULDBLOCK)
#else
			if (errno != EWOULDBLOCK && errno != EAGAIN)
#endif
			{
				ZF_LOGI("KISS: send to client (slot %d) failed, closing.", i);
				KISSCloseClient(i);
			}
		}
	}
	ZF_LOGI("KISS: delivered %d byte AX.25 frame to clients.", len);
}
