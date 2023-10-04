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
// Copyright (c) 1996-2023, Live Networks, Inc.  All rights reserved
// A test program that receives a RTP/RTCP multicast MPEG-2 Transport Stream,
// and outputs the resulting Transport Stream data to 'stdout'
// main program

#include "liveMedia.hh"
#include "GroupsockHelper.hh"

#include "BasicUsageEnvironment.hh"

#include <thread>
#include <chrono>

#ifdef WIN32
#include <conio.h>
#ifdef _DEBUG
#include <crtdbg.h>
#endif

#define mygetch	getch

#elif defined(LINUX)
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

int mygetch(void)
{
	struct termios oldt,
		newt;
	int ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
}
#endif

// To receive a "source-specific multicast" (SSM) stream, uncomment this:
//#define USE_SSM 1

void afterPlaying(void* clientData); // forward

// A structure to hold the state of the current session.
// It is used in the "afterPlaying()" function to clean up the session.
struct sessionState_t {
	RTPSource* source;
	MediaSink* sink;
	RTCPInstance* rtcpInstance;
} sessionState;

UsageEnvironment* env;

class DummySink : public MediaSink {
public:
	static DummySink* createNew(UsageEnvironment& env, unsigned bufferSize = 20000);

	virtual void addData(unsigned char const* data, unsigned dataSize,
		struct timeval presentationTime);
protected:
	DummySink(UsageEnvironment& env, unsigned bufferSize);
	// called only by createNew()
	virtual ~DummySink();

protected: // redefined virtual functions:
	virtual Boolean continuePlaying();

protected:
	static void afterGettingFrame(void* clientData, unsigned frameSize,
		unsigned numTruncatedBytes,
		struct timeval presentationTime,
		unsigned durationInMicroseconds);
	virtual void afterGettingFrame(unsigned frameSize,
		unsigned numTruncatedBytes,
		struct timeval presentationTime);

	unsigned char* fBuffer;
	unsigned fBufferSize;
	struct timeval fPrevPresentationTime;
	unsigned fSamePresentationTimeCounter;

	FILE* fFile;
};

DummySink::DummySink(UsageEnvironment& env, unsigned bufferSize)
	: MediaSink(env), fBufferSize(bufferSize), fSamePresentationTimeCounter(0), fFile(NULL) {
	fBuffer = new unsigned char[bufferSize];
	fPrevPresentationTime.tv_sec = ~0; fPrevPresentationTime.tv_usec = 0;
	//fFile = fopen("test.264", "wb");
}

DummySink::~DummySink() {
	if (fFile) fclose(fFile);
	delete[] fBuffer;
}

DummySink* DummySink::createNew(UsageEnvironment& env, unsigned bufferSize) {
	return new DummySink(env, bufferSize);
}

Boolean DummySink::continuePlaying() {
	if (fSource == NULL) return False;

	fSource->getNextFrame(fBuffer, fBufferSize,
		afterGettingFrame, this,
		onSourceClosure, this);

	return True;
}

void DummySink::afterGettingFrame(void* clientData, unsigned frameSize,
	unsigned numTruncatedBytes,
	struct timeval presentationTime,
	unsigned /*durationInMicroseconds*/) {
	DummySink* sink = (DummySink*)clientData;
	sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime);
}

void DummySink::afterGettingFrame(unsigned frameSize,
	unsigned numTruncatedBytes,
	struct timeval presentationTime) {
	if (numTruncatedBytes > 0) {
		envir() << "DummySink::afterGettingFrame(): The input frame data was too large for our buffer size ("
			<< fBufferSize << ").  "
			<< numTruncatedBytes << " bytes of trailing data was dropped!  Correct this by increasing the \"bufferSize\" parameter in the \"createNew()\" call to at least "
			<< fBufferSize + numTruncatedBytes << "\n";
	}
	addData(fBuffer, frameSize, presentationTime);
#if 1
	char uSecsStr[6 + 1]; // used to output the 'microseconds' part of the presentation time
	sprintf(uSecsStr, "%06u", (unsigned)presentationTime.tv_usec);

	envir() << "DummySink received " << frameSize << " bytes" 
		<< "\tPresentation time: " << (int)presentationTime.tv_sec << "." << uSecsStr << "\n";
#endif
	if (fFile) fwrite(fBuffer, 1, frameSize, fFile);
	// Then try getting the next frame:
	continuePlaying();
}

void DummySink::addData(unsigned char const* data, unsigned dataSize,
	struct timeval presentationTime) {
	// Write to our file:
#ifdef TEST_LOSS
	static unsigned const framesPerPacket = 10;
	static unsigned const frameCount = 0;
	static Boolean const packetIsLost;
	if ((frameCount++) % framesPerPacket == 0) {
		packetIsLost = (our_random() % 10 == 0); // simulate 10% packet loss #####
	}

	if (!packetIsLost) {
	}
#endif
}

char schedulerRun = 0;
void thread_scheduler()
{
	env->taskScheduler().doEventLoop(&schedulerRun);
}

int main(int argc, char** argv) {
	// Begin by setting up our usage environment:
	TaskScheduler* scheduler = BasicTaskScheduler::createNew();
	env = BasicUsageEnvironment::createNew(*scheduler);

	// Create the data sink for 'stdout':
	//sessionState.sink = FileSink::createNew(*env, "stdout");
	//sessionState.sink = FileSink::createNew(*env, "mp2ts.264");
	sessionState.sink = DummySink::createNew(*env);
	// Note: The string "stdout" is handled as a special case.
	// A real file name could have been used instead.

	// Create 'groupsocks' for RTP and RTCP:
	char const* sessionAddressStr
#ifdef USE_SSM
		= "232.255.42.42";
#else
		//= "239.255.42.42";
		= "230.0.5.1";
	// Note: If the session is unicast rather than multicast,
	// then replace this string with "0.0.0.0"
#endif
  //const unsigned short rtpPortNum = 1234;
	const unsigned short rtpPortNum = 50010;
	const unsigned short rtcpPortNum = rtpPortNum + 1;
#ifndef USE_SSM
	const unsigned char ttl = 1; // low, in case routers don't admin scope
#endif

	NetAddressList sessionAddresses(sessionAddressStr);
	struct sockaddr_storage sessionAddress;
	copyAddress(sessionAddress, sessionAddresses.firstAddress());

	const Port rtpPort(rtpPortNum);
	const Port rtcpPort(rtcpPortNum);

#ifdef USE_SSM
	char const* sourceAddressStr = "aaa.bbb.ccc.ddd";
	// replace this with the real source address
	NetAddressList sourceFilterAddresses(sourceAddressStr);
	struct sockaddr_storage sourceFilterAddress;
	copyAddress(sourceFilterAddress, sourceFilterAddresses.firstAddress());

	Groupsock rtpGroupsock(*env, sessionAddress, sourceFilterAddress, rtpPort);
	Groupsock rtcpGroupsock(*env, sessionAddress, sourceFilterAddress, rtcpPort);
	rtcpGroupsock.changeDestinationParameters(sourceFilterAddress, 0, ~0);
	// our RTCP "RR"s are sent back using unicast
#else
	Groupsock rtpGroupsock(*env, sessionAddress, rtpPort, ttl);
	Groupsock rtcpGroupsock(*env, sessionAddress, rtcpPort, ttl);
#endif

	// Create the data source: a "MPEG-2 TransportStream RTP source" (which uses a 'simple' RTP payload format):
	sessionState.source = SimpleRTPSource::createNew(*env, &rtpGroupsock, 33, 90000, "video/MP2T", 0, False /*no 'M' bit*/);	

	// Create (and start) a 'RTCP instance' for the RTP source:
	const unsigned estimatedSessionBandwidth = 5000; // in kbps; for RTCP b/w share
	const unsigned maxCNAMElen = 100;
	unsigned char CNAME[maxCNAMElen + 1];
	gethostname((char*)CNAME, maxCNAMElen);
	CNAME[maxCNAMElen] = '\0'; // just in case
	sessionState.rtcpInstance
		= RTCPInstance::createNew(*env, &rtcpGroupsock,
			estimatedSessionBandwidth, CNAME,
			NULL /* we're a client */, sessionState.source);
	// Note: This starts RTCP running automatically

	// Finally, start receiving the multicast stream:
	*env << "Beginning receiving multicast stream...\n";
	sessionState.sink->startPlaying(*sessionState.source, afterPlaying, NULL);
#if 0
	env->taskScheduler().doEventLoop(); // does not return
#else
	std::thread thread(thread_scheduler);

	char c = mygetch();
	while (c != 'q') {
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		c = mygetch();
	}

	//sessionState.source->stopGettingFrames();
	//sessionState.sink->stopPlaying();

	schedulerRun = 1;
	thread.join();
	
	sessionState.source->stopGettingFrames();
	sessionState.sink->stopPlaying();
#endif
	return 0; // only to prevent compiler warning
}

void afterPlaying(void* /*clientData*/) {
	*env << "...done receiving\n";

	// End by closing the media:
	Medium::close(sessionState.rtcpInstance); // Note: Sends a RTCP BYE
	Medium::close(sessionState.sink);
	Medium::close(sessionState.source);
}
