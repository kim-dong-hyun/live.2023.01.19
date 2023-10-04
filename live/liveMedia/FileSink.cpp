/**********
This library is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the
Free Software Foundation; either version 3 of the License, or (at your
option) any later version. (See <http://www.gnu.org/copyleft/lesser.html>.)

This library is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License
along with this library; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
**********/
// "liveMedia"
// Copyright (c) 1996-2023 Live Networks, Inc.  All rights reserved.
// File sinks
// Implementation

#if (defined(__WIN32__) || defined(_WIN32)) && !defined(_WIN32_WCE)
#include <io.h>
#include <fcntl.h>
#endif
#include "FileSink.hh"
#include "GroupsockHelper.hh"
#include "OutputFile.hh"

#if 1
#define TS_PACKET_SIZE 188
#define MAX_PIDS 8192

unsigned char stream_types[MAX_PIDS] = { 0 };

void parse_pat(unsigned char* packet) {
	int section_length = ((packet[6] & 0x0F) << 8) | packet[7];
	int current_byte = 8; // PAT starts after the pointer_field

	while (current_byte < (section_length + 3 - 4)) {
		unsigned short program_number = (packet[current_byte] << 8) | packet[current_byte + 1];
		unsigned short pmt_pid = ((packet[current_byte + 2] & 0x1F) << 8) | packet[current_byte + 3];

		if (program_number == 0) { // This is the PID for the NIT, not PMT.
			current_byte += 4;
			continue;
		}

		stream_types[pmt_pid] = 0x00;  // Use 0x00 to indicate PMT
		current_byte += 4;
	}
}

void parse_pmt(const unsigned char* packet) {
	int section_length = ((packet[6] & 0x0F) << 8) | packet[7];
	int program_info_length = ((packet[10] & 0x0F) << 8) | packet[11];
	int current_byte = 12 + program_info_length;

	while (current_byte < (section_length + 3 - 4)) {  // -4 for the CRC32 at the end
		unsigned char stream_type = packet[current_byte];
		unsigned short elementary_pid = ((packet[current_byte + 1] & 0x1F) << 8) | packet[current_byte + 2];
		stream_types[elementary_pid] = stream_type;
		int es_info_length = ((packet[current_byte + 3] & 0x0F) << 8) | packet[current_byte + 4];
		current_byte += 5 + es_info_length;
	}
}

void parse_pes(unsigned char* packet) {
	if (packet[0] == 0x00 && packet[1] == 0x00 && packet[2] == 0x01) {
		unsigned char stream_id = packet[3];
		printf("PES with stream ID: 0x%02x\n", stream_id);
	}
}

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

const int ts_packet_size = 188;

// MPEG-2 TS 패킷 헤더 구조체
struct mpeg2ts_packet_header {
	uint8_t sync_byte;
	uint8_t transport_error_indicator;
	uint8_t payload_unit_start_indicator;
	uint8_t transport_priority;
	uint16_t pid;
	uint8_t transport_scrambling_control;
	uint8_t adaptation_field_control;
	uint8_t continuity_counter;
	// adaptation_field_control 비트가 1인 경우
	struct adaptation_field {
		uint8_t adaptation_field_length;
		// 0x01: PES header
		uint8_t adaptation_field_data[1];
	} adaptation_field;
	// payload
	uint8_t payload[ts_packet_size - 24];
};

int analyze_ts_packet(struct mpeg2ts_packet_header* p) {
	// PID가 비디오 또는 오디오인 경우
	if (p->pid >= 0x0000 && p->pid <= 0x001F) {
		printf("video\n");
		return 0;
	}
	else if (p->pid >= 0x0080 && p->pid <= 0x00BF) {
		printf("audio\n");
		return 1;
	}
	else {
		printf("other\n");
		return 2;
	}
}

unsigned char buffer_ts[1024 * 10];
int buffer_ts_index = 0;
#endif

////////// FileSink //////////

FileSink::FileSink(UsageEnvironment& env, FILE* fid, unsigned bufferSize,
	char const* perFrameFileNamePrefix)
	: MediaSink(env), fOutFid(fid), fBufferSize(bufferSize), fSamePresentationTimeCounter(0) {
	fBuffer = new unsigned char[bufferSize];
	if (perFrameFileNamePrefix != NULL) {
		fPerFrameFileNamePrefix = strDup(perFrameFileNamePrefix);
		fPerFrameFileNameBuffer = new char[strlen(perFrameFileNamePrefix) + 100];
	}
	else {
		fPerFrameFileNamePrefix = NULL;
		fPerFrameFileNameBuffer = NULL;
	}
	fPrevPresentationTime.tv_sec = ~0; fPrevPresentationTime.tv_usec = 0;
}

FileSink::~FileSink() {
	delete[] fPerFrameFileNameBuffer;
	delete[] fPerFrameFileNamePrefix;
	delete[] fBuffer;
	if (fOutFid != NULL) fclose(fOutFid);
}

FileSink* FileSink::createNew(UsageEnvironment& env, char const* fileName,
	unsigned bufferSize, Boolean oneFilePerFrame) {
	do {
		FILE* fid;
		char const* perFrameFileNamePrefix;
		if (oneFilePerFrame) {
			// Create the fid for each frame
			fid = NULL;
			perFrameFileNamePrefix = fileName;
		}
		else {
			// Normal case: create the fid once
			fid = OpenOutputFile(env, fileName);
			if (fid == NULL) break;
			perFrameFileNamePrefix = NULL;
		}

		return new FileSink(env, fid, bufferSize, perFrameFileNamePrefix);
	} while (0);

	return NULL;
}

Boolean FileSink::continuePlaying() {
	if (fSource == NULL) return False;

	fSource->getNextFrame(fBuffer, fBufferSize,
		afterGettingFrame, this,
		onSourceClosure, this);

	return True;
}

void FileSink::afterGettingFrame(void* clientData, unsigned frameSize,
	unsigned numTruncatedBytes,
	struct timeval presentationTime,
	unsigned /*durationInMicroseconds*/) {
	FileSink* sink = (FileSink*)clientData;
	sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime);
}

void FileSink::addData(unsigned char const* data, unsigned dataSize,
	struct timeval presentationTime) {
	if (fPerFrameFileNameBuffer != NULL && fOutFid == NULL) {
		// Special case: Open a new file on-the-fly for this frame
		if (presentationTime.tv_usec == fPrevPresentationTime.tv_usec &&
			presentationTime.tv_sec == fPrevPresentationTime.tv_sec) {
			// The presentation time is unchanged from the previous frame, so we add a 'counter'
			// suffix to the file name, to distinguish them:
			sprintf(fPerFrameFileNameBuffer, "%s-%lu.%06lu-%u", fPerFrameFileNamePrefix,
				presentationTime.tv_sec, presentationTime.tv_usec, ++fSamePresentationTimeCounter);
		}
		else {
			sprintf(fPerFrameFileNameBuffer, "%s-%lu.%06lu", fPerFrameFileNamePrefix,
				presentationTime.tv_sec, presentationTime.tv_usec);
			fPrevPresentationTime = presentationTime; // for next time
			fSamePresentationTimeCounter = 0; // for next time
		}
		fOutFid = OpenOutputFile(envir(), fPerFrameFileNameBuffer);
	}

	// Write to our file:
#ifdef TEST_LOSS
	static unsigned const framesPerPacket = 10;
	static unsigned const frameCount = 0;
	static Boolean const packetIsLost;
	if ((frameCount++) % framesPerPacket == 0) {
		packetIsLost = (our_random() % 10 == 0); // simulate 10% packet loss #####
	}

	if (!packetIsLost)
#endif
		if (fOutFid != NULL && data != NULL) {
#if 1
			fwrite(data, 1, dataSize, fOutFid);
#else
			memcpy(&buffer_ts[buffer_ts_index], data, dataSize);
			buffer_ts_index += dataSize;

			while (buffer_ts_index >= TS_PACKET_SIZE) {
				//printf("start : %02x\n", buffer_ts[0]);
				//for (int i = 0; i < 20; i++) printf("%02x ", buffer_ts[i]);
				//printf("\n");
#if 0
				struct mpeg2ts_packet_header* p = (struct mpeg2ts_packet_header*)buffer_ts;
				int ret = analyze_ts_packet(p);
				//if (ret != 1) 
					fwrite(buffer_ts, 1, TS_PACKET_SIZE, fOutFid);
#else
				fwrite(buffer_ts, 1, TS_PACKET_SIZE, fOutFid);

				//unsigned short pid = ((buffer_ts[1] & 0x1F) << 8) | buffer_ts[2];
				unsigned short pid = ((buffer_ts[4] & 0x1F) << 8) | buffer_ts[5];
				if (pid == 0x0000) {  // PAT, we just find PMT PID here
					//unsigned short pmt_pid = ((buffer_ts[13] & 0x1F) << 8) | buffer_ts[14];
					//stream_types[pmt_pid] = 0x00;  // PAT
					parse_pat(buffer_ts);
				}
				else if (stream_types[pid] == 0x00) {  // PMT
					parse_pmt(buffer_ts);
				}
				else {
					//parse_pes(buffer_ts + 4);  // Typically, PES starts after 4 bytes into the TS payload

					switch (stream_types[pid]) {
					case 0x02:
					case 0x1B:
						printf("Video packet with PID: %d, stream type: %02x\n", pid, stream_types[pid]);
						break;
					case 0x03:
					case 0x04:
					case 0x0F:
						printf("Audio packet with PID: %d, stream_type: %02x\n", pid, stream_types[pid]);
						break;
					default:
						break;
					}
					//printf("PID : %d, stream_type : %d(%02x)\n", pid, stream_types[pid], stream_types[pid]);
				}
#endif
				memcpy(buffer_ts, &buffer_ts[TS_PACKET_SIZE], buffer_ts_index - TS_PACKET_SIZE);
				buffer_ts_index -= TS_PACKET_SIZE;
			}
#endif
		}
}

void FileSink::afterGettingFrame(unsigned frameSize,
	unsigned numTruncatedBytes,
	struct timeval presentationTime) {
	if (numTruncatedBytes > 0) {
		envir() << "FileSink::afterGettingFrame(): The input frame data was too large for our buffer size ("
			<< fBufferSize << ").  "
			<< numTruncatedBytes << " bytes of trailing data was dropped!  Correct this by increasing the \"bufferSize\" parameter in the \"createNew()\" call to at least "
			<< fBufferSize + numTruncatedBytes << "\n";
	}
	addData(fBuffer, frameSize, presentationTime);

	if (fOutFid == NULL || fflush(fOutFid) == EOF) {
		// The output file has closed.  Handle this the same way as if the input source had closed:
		if (fSource != NULL) fSource->stopGettingFrames();
		onSourceClosure();
		return;
	}

	if (fPerFrameFileNameBuffer != NULL) {
		if (fOutFid != NULL) { fclose(fOutFid); fOutFid = NULL; }
	}

	// Then try getting the next frame:
	continuePlaying();
}
