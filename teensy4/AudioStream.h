/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef AudioStream_h
#define AudioStream_h

#ifndef __ASSEMBLER__
#include <stdio.h>  // for NULL
#include <string.h> // for memcpy

#endif

// AUDIO_BLOCK_SAMPLES determines how many samples the audio library processes
// per update.  It may be reduced to achieve lower latency response to events,
// at the expense of higher interrupt and DMA setup overhead.
//
// Less than 32 may not work with some input & output objects.  Multiples of 16
// should be used, since some synthesis objects generate 16 samples per loop.
//
// Some parts of the audio library may have hard-coded dependency on 128 samples.
// Please report these on the forum with reproducible test cases.  The following
// audio classes are known to have problems with smaller block sizes:
//   AudioInputUSB, AudioOutputUSB, AudioPlaySdWav, AudioAnalyzeFFT256,
//   AudioAnalyzeFFT1024

#ifndef AUDIO_BLOCK_SAMPLES
#define AUDIO_BLOCK_SAMPLES  128
#endif

#ifndef AUDIO_SAMPLE_RATE_EXACT
#define AUDIO_SAMPLE_RATE_EXACT 44100.0f
#endif

#define AUDIO_SAMPLE_RATE AUDIO_SAMPLE_RATE_EXACT

#define noAUDIO_DEBUG_CLASS // disable this class by default

#ifndef __ASSEMBLER__
class AudioStream;
class AudioConnection;
#if defined(AUDIO_DEBUG_CLASS)
class AudioDebug;  // for testing only, never for public release
#endif // defined(AUDIO_DEBUG_CLASS)

typedef struct audio_block_struct {
	uint8_t  ref_count;
	uint8_t  reserved1;
	uint16_t memory_pool_index;
	int16_t  data[AUDIO_BLOCK_SAMPLES];
} audio_block_t;



class AudioConnection
{
public:
	AudioConnection();
	AudioConnection(AudioStream &source, AudioStream &destination)
		: AudioConnection() { connect(source,destination); }
	AudioConnection(AudioStream &source, unsigned char sourceOutput,
		AudioStream &destination, unsigned char destinationInput)
		: AudioConnection() { connect(source,sourceOutput, destination,destinationInput); }
	friend class AudioStream;
	~AudioConnection(); 
	int disconnect(void);
	int connect(void);
	int connect(AudioStream &source, AudioStream &destination) {return connect(source,0,destination,0);};
	int connect(AudioStream &source, unsigned char sourceOutput,
		AudioStream &destination, unsigned char destinationInput);
protected:
	AudioStream* src;	// can't use references as... 
	AudioStream* dst;	// ...they can't be re-assigned!
	unsigned char src_index;
	unsigned char dest_index;
	AudioConnection *next_dest; // linked list of connections from one source
	bool isConnected;
#if defined(AUDIO_DEBUG_CLASS)
	friend class AudioDebug;
#endif // defined(AUDIO_DEBUG_CLASS)
};


#define AudioMemory(num) ({ \
	static DMAMEM __attribute__((aligned(4))) audio_block_t data[num]; \
	AudioStream::initialize_memory(data, num); \
})

#define CYCLE_COUNTER_APPROX_PERCENT(n) (((float)((uint32_t)(n) * 6400u) * (float)(AUDIO_SAMPLE_RATE_EXACT / AUDIO_BLOCK_SAMPLES)) / (float)(F_CPU_ACTUAL))

#define AudioProcessorUsage() (CYCLE_COUNTER_APPROX_PERCENT(AudioStream::cpu_cycles_total))
#define AudioProcessorUsageMax() (CYCLE_COUNTER_APPROX_PERCENT(AudioStream::cpu_cycles_total_max))
#define AudioProcessorUsageMaxReset() (AudioStream::cpu_cycles_total_max = AudioStream::cpu_cycles_total)
#define AudioMemoryUsage() (AudioStream::memory_used)
#define AudioMemoryUsageMax() (AudioStream::memory_used_max)
#define AudioMemoryUsageMaxReset() (AudioStream::memory_used_max = AudioStream::memory_used)

class AudioStream
{
public:
	AudioStream(unsigned char ninput, audio_block_t **iqueue) :
		num_inputs(ninput), inputQueue(iqueue) {
			active = false;
			destination_list = NULL;
			for (int i=0; i < num_inputs; i++) {
				inputQueue[i] = NULL;
			}
			// add to a simple list, for update_all
			// TODO: replace with a proper data flow analysis in update_all
			if (first_update == NULL) {
				first_update = this;
			} else {
				AudioStream *p;
				for (p=first_update; p->next_update; p = p->next_update) ;
				p->next_update = this;
			}
			next_update = NULL;
			cpu_cycles = 0;
			cpu_cycles_max = 0;
			numConnections = 0;
		}
	static void initialize_memory(audio_block_t *data, unsigned int num);
	float processorUsage(void) { return CYCLE_COUNTER_APPROX_PERCENT(cpu_cycles); }
	float processorUsageMax(void) { return CYCLE_COUNTER_APPROX_PERCENT(cpu_cycles_max); }
	void processorUsageMaxReset(void) { cpu_cycles_max = cpu_cycles; }
	bool isActive(void) { return active; }
	uint16_t cpu_cycles;
	uint16_t cpu_cycles_max;
	static uint16_t cpu_cycles_total;
	static uint16_t cpu_cycles_total_max;
	static uint16_t memory_used;
	static uint16_t memory_used_max;
protected:
	bool active;
	unsigned char num_inputs;
	static audio_block_t * allocate(void);
	static void release(audio_block_t * block);
	void transmit(audio_block_t *block, unsigned char index = 0);
	audio_block_t * receiveReadOnly(unsigned int index = 0);
	audio_block_t * receiveWritable(unsigned int index = 0);
	static bool update_setup(void);
	static void update_stop(void);
	static void update_all(void) { NVIC_SET_PENDING(IRQ_SOFTWARE); }
	friend void software_isr(void);
	friend class AudioConnection;
#if defined(AUDIO_DEBUG_CLASS)
	friend class AudioDebug;
#endif // defined(AUDIO_DEBUG_CLASS)
	uint8_t numConnections;
private:
	static AudioConnection* unused; // linked list of unused but not destructed connections
	AudioConnection *destination_list;
	audio_block_t **inputQueue;
	static bool update_scheduled;
	virtual void update(void) = 0;
	static AudioStream *first_update; // for update_all
	AudioStream *next_update; // for update_all
	static audio_block_t *memory_pool;
	static uint32_t memory_pool_available_mask[];
	static uint16_t memory_pool_first_mask;
};

#if defined(AUDIO_DEBUG_CLASS)
// This class aids debugging of the internal functionality of the
// AudioStream and AudioConnection classes, but is NOT intended
// for general users of the Audio library.
class AudioDebug
{
	public:
		// info on connections
		AudioStream* getSrc(AudioConnection& c) { return c.src;};
		AudioStream* getDst(AudioConnection& c) { return c.dst;};
		unsigned char getSrcN(AudioConnection& c) { return c.src_index;};
		unsigned char getDstN(AudioConnection& c) { return c.dest_index;};
		AudioConnection* getNext(AudioConnection& c) { return c.next_dest;};
		bool isConnected(AudioConnection& c) { return c.isConnected;};
		AudioConnection* unusedList() { return AudioStream::unused;};
		
		// info on streams
		AudioConnection* dstList(AudioStream& s) { return s.destination_list;};
		audio_block_t ** inqList(AudioStream& s) { return s.inputQueue;};
		uint8_t 	 	 getNumInputs(AudioStream& s) { return s.num_inputs;};
		AudioStream*     firstUpdate(AudioStream& s) { return s.first_update;};
		AudioStream* 	 nextUpdate(AudioStream& s) { return s.next_update;};
		uint8_t 	 	 getNumConnections(AudioStream& s) { return s.numConnections;};
		bool 	 	 	 isActive(AudioStream& s) { return s.active;};
		 
		
};
#endif // defined(AUDIO_DEBUG_CLASS)

#endif // __ASSEMBLER__
#endif // AudioStream_h
