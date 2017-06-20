#include "devices/playback.h"

#include "devices/davis.h"
#include <stdatomic.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#if defined(HAVE_PTHREADS)
  #include "c11threads_posix.h"
#endif

#include "ringbuffer/ringbuffer.h"
#include "davis_common.h"

struct playback_state {
  FILE * file;
  // Data Acquisition Thread -> Mainloop Exchange
  RingBuffer dataExchangeBuffer;
  atomic_uint_fast32_t dataExchangeBufferSize; // Only takes effect on DataStart() calls!
  atomic_bool dataExchangeBlocking;
  atomic_bool dataExchangeStartProducers;

  // USB Device State
  char deviceThreadName[15 + 1]; // +1 for terminating NUL character.

  // Data Acquisition Thread
  void (*playbackFinishedCallback) (void*);
  void* callbackParam;

  thrd_t dataAcquisitionThread;
  atomic_bool dataAcquisitionThreadRun;
  atomic_uint_fast32_t dataAcquisitionThreadConfigUpdate;
  // Timestamp fields
  int32_t wrapOverflow;
  int32_t wrapAdd;
  int32_t lastTimestamp;
  int32_t currentTimestamp;
  int32_t startTimestamp;
  int64_t hostStartTime;

  float playbackSpeed;

  bool lastAPSEventWasReset;
  uint32_t pixelReceived;
  // DVS specific fields
  uint16_t dvsLastY;
  bool dvsGotY;
  bool dvsFlipX;
  bool dvsFlipY;
  int16_t dvsSizeX;
  int16_t dvsSizeY;

  // APS specific fields
  int16_t apsSizeX;
  int16_t apsSizeY;

  bool apsFlipX;
  bool apsFlipY;
  bool apsIgnoreEvents;
  bool apsGlobalShutter;
  bool apsResetRead;
  bool apsRGBPixelOffsetDirection; // 0 is increasing, 1 is decreasing.
  int16_t apsRGBPixelOffset;
  uint16_t apsCurrentReadoutType;
  uint16_t apsCountX[APS_READOUT_TYPES_NUM];
  uint16_t apsCountY[APS_READOUT_TYPES_NUM];
  uint16_t *apsCurrentResetFrame;
  uint16_t apsROIUpdate;
  uint16_t apsROITmpData;
  uint16_t apsROISizeX[APS_ROI_REGIONS_MAX];
  uint16_t apsROISizeY[APS_ROI_REGIONS_MAX];
  uint16_t apsROIPositionX[APS_ROI_REGIONS_MAX];
  uint16_t apsROIPositionY[APS_ROI_REGIONS_MAX];

  // IMU specific fields
  bool imuIgnoreEvents;
  bool imuFlipX;
  bool imuFlipY;
  bool imuFlipZ;
  uint8_t imuCount;
  uint8_t imuTmpData;
  float imuAccelScale;
  float imuGyroScale;
  // Microphone specific fields
  bool micRight;
  uint8_t micCount;
  uint16_t micTmpData;
  // Packet Container state
  caerEventPacketContainer currentPacketContainer;
  atomic_uint_fast32_t maxPacketContainerPacketSize;
  atomic_uint_fast32_t maxPacketContainerInterval;
  int64_t currentPacketContainerCommitTimestamp;
  // Polarity Packet state
  caerPolarityEventPacket currentPolarityPacket;
  int32_t currentPolarityPacketPosition;
  // Frame Packet state
  caerFrameEventPacket currentFramePacket;
  int32_t currentFramePacketPosition;
  // IMU6 Packet state
  caerIMU6EventPacket currentIMU6Packet;
  int32_t currentIMU6PacketPosition;
  // Special Packet state
  caerSpecialEventPacket currentSpecialPacket;
  int32_t currentSpecialPacketPosition;
  // Microphone Sample Packet state
  caerSampleEventPacket currentSamplePacket;
  int32_t currentSamplePacketPosition;
  // Current composite events, for later copy, to not loose them on commits.
  caerFrameEvent currentFrameEvent[APS_ROI_REGIONS_MAX];
  struct caer_imu6_event currentIMU6Event;

};

struct playback_handle {
  uint16_t deviceType;
  // Information fields
  struct caer_davis_info info;
  // State for data management, common to all DAVIS.
  struct playback_state state;

  struct playback_info playbackInfo;
};


static void playbackDavisEventTranslator(void *vhd, uint8_t *buffer, size_t bytesSent);
static int playbackDataAcquisitionThread(void *inPtr);

static inline void checkStrictMonotonicTimestamp(playbackHandle handle) {
  if (handle->state.currentTimestamp < handle->state.lastTimestamp) {
    caerLog(CAER_LOG_ALERT, handle->info.deviceString,
      "Timestamps: non strictly-monotonic timestamp detected: lastTimestamp=%" PRIi32 ", currentTimestamp=%" PRIi32 ", difference=%" PRIi32 ".",
      handle->state.lastTimestamp, handle->state.currentTimestamp,
      (handle->state.lastTimestamp - handle->state.currentTimestamp));
  }
}

static inline uint64_t getCurrTime(){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return (uint64_t)1000000 * tv.tv_sec + tv.tv_usec;
}

static inline void updateROISizes(playbackState state) {
  // Calculate APS ROI sizes for each region.
  for (size_t i = 0; i < APS_ROI_REGIONS_MAX; i++) {
    uint16_t startColumn = state->apsROIPositionX[i];
    uint16_t startRow = state->apsROIPositionY[i];
    uint16_t endColumn = state->apsROISizeX[i];
    uint16_t endRow = state->apsROISizeY[i];

    // Position is already set to startCol/Row, so we don't have to reset
    // it here. We only have to calculate size from start and end Col/Row.
    if (startColumn < state->apsSizeX && endColumn < state->apsSizeX && startRow < state->apsSizeY
      && endRow < state->apsSizeY) {
      state->apsROISizeX[i] = U16T(endColumn + 1 - startColumn);
      state->apsROISizeY[i] = U16T(endRow + 1 - startRow);

      if (state->apsFlipX && state->apsFlipY) {
        // Inverted, so X[StartColumn] becomes endColumn. Y[endRow] becomes startRow.
        // Same accounting for origin in upper left corner, but on the other axis here.
        state->apsROIPositionX[i] = U16T(state->apsSizeX - 1 - endColumn);
        state->apsROIPositionY[i] = startRow;
      }
      else {
        // Y position needs to be inverted with endRow to account for the
        // origin (0, 0) being in the upper left corner. X is fine as startColumn.
        state->apsROIPositionY[i] = U16T(state->apsSizeY - 1 - endRow);
      }
    }
    else {
      // Turn off this ROI region.
      state->apsROISizeX[i] = state->apsROIPositionX[i] = U16T(state->apsSizeX);
      state->apsROISizeY[i] = state->apsROIPositionY[i] = U16T(state->apsSizeY);
    }
  }
}

static inline void initFrame(playbackHandle handle) {
  playbackState state = &handle->state;

  state->apsCurrentReadoutType = APS_READOUT_RESET;
  for (size_t i = 0; i < APS_READOUT_TYPES_NUM; i++) {
    state->apsCountX[i] = 0;
    state->apsCountY[i] = 0;
  }

  // for (size_t i = 0; i < APS_ROI_REGIONS_MAX; i++) {
  memset(state->currentFrameEvent[0], 0, (sizeof(struct caer_frame_event) - sizeof(uint16_t)));
  // }

  if (state->apsROIUpdate != 0) {
    //updateROISizes(state);
  }

  // Skip frame if ROI region is disabled.
  if (state->apsROIPositionX[0] >= state->apsSizeX || state->apsROIPositionY[0] >= state->apsSizeY) {
    return;
  }

  // Write out start of frame timestamp.
  caerFrameEventSetTSStartOfFrame(state->currentFrameEvent[0], state->currentTimestamp);

  // Send APS info event out (as special event).
  caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(state->currentSpecialPacket,
    state->currentSpecialPacketPosition);
  caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
  caerSpecialEventSetType(currentSpecialEvent, APS_FRAME_START);
  caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
  state->currentSpecialPacketPosition++;

  // Setup frame. Only ROI region 0 is supported currently.
  caerFrameEventSetColorFilter(state->currentFrameEvent[0], handle->info.apsColorFilter);
  caerFrameEventSetROIIdentifier(state->currentFrameEvent[0], 0);
  caerFrameEventSetLengthXLengthYChannelNumber(state->currentFrameEvent[0], state->apsROISizeX[0],
    state->apsROISizeY[0], APS_ADC_CHANNELS, state->currentFramePacket);
  caerFrameEventSetPositionX(state->currentFrameEvent[0], state->apsROIPositionX[0]);
  caerFrameEventSetPositionY(state->currentFrameEvent[0], state->apsROIPositionY[0]);
}

static inline float calculateIMUAccelScale(uint8_t imuAccelScale) {
  // Accelerometer scale is:
  // 0 - +-2 g - 16384 LSB/g
  // 1 - +-4 g - 8192 LSB/g
  // 2 - +-8 g - 4096 LSB/g
  // 3 - +-16 g - 2048 LSB/g
  float accelScale = 65536.0f / (float) U32T(4 * (1 << imuAccelScale));

  return (accelScale);
}

static inline float calculateIMUGyroScale(uint8_t imuGyroScale) {
  // Gyroscope scale is:
  // 0 - +-250 °/s - 131 LSB/°/s
  // 1 - +-500 °/s - 65.5 LSB/°/s
  // 2 - +-1000 °/s - 32.8 LSB/°/s
  // 3 - +-2000 °/s - 16.4 LSB/°/s
  float gyroScale = 65536.0f / (float) U32T(500 * (1 << imuGyroScale));

  return (gyroScale);
}

static inline void freeAllDataMemory(playbackState state) {
  if (state->dataExchangeBuffer != NULL) {
    ringBufferFree(state->dataExchangeBuffer);
    state->dataExchangeBuffer = NULL;
  }

  // Since the current event packets aren't necessarily
  // already assigned to the current packet container, we
  // free them separately from it.
  if (state->currentPolarityPacket != NULL) {
    free(&state->currentPolarityPacket->packetHeader);
    state->currentPolarityPacket = NULL;

    if (state->currentPacketContainer != NULL) {
      caerEventPacketContainerSetEventPacket(state->currentPacketContainer, POLARITY_EVENT, NULL);
    }
  }

  if (state->currentSpecialPacket != NULL) {
    free(&state->currentSpecialPacket->packetHeader);
    state->currentSpecialPacket = NULL;

    if (state->currentPacketContainer != NULL) {
      caerEventPacketContainerSetEventPacket(state->currentPacketContainer, SPECIAL_EVENT, NULL);
    }
  }

  if (state->currentFramePacket != NULL) {
    free(&state->currentFramePacket->packetHeader);
    state->currentFramePacket = NULL;

    if (state->currentPacketContainer != NULL) {
      caerEventPacketContainerSetEventPacket(state->currentPacketContainer, FRAME_EVENT, NULL);
    }
  }

  if (state->currentIMU6Packet != NULL) {
    free(&state->currentIMU6Packet->packetHeader);
    state->currentIMU6Packet = NULL;

    if (state->currentPacketContainer != NULL) {
      caerEventPacketContainerSetEventPacket(state->currentPacketContainer, IMU6_EVENT, NULL);
    }
  }

  if (state->currentSamplePacket != NULL) {
    free(&state->currentSamplePacket->packetHeader);
    state->currentSamplePacket = NULL;

    if (state->currentPacketContainer != NULL) {
      caerEventPacketContainerSetEventPacket(state->currentPacketContainer, DAVIS_SAMPLE_POSITION, NULL);
    }
  }

  if (state->currentPacketContainer != NULL) {
    caerEventPacketContainerFree(state->currentPacketContainer);
    state->currentPacketContainer = NULL;
  }

  if (state->apsCurrentResetFrame != NULL) {
    free(state->apsCurrentResetFrame);
    state->apsCurrentResetFrame = NULL;
  }

  // Also free current ROI frame events.
  free(state->currentFrameEvent[0]); // Other regions are contained within contiguous memory block.

  for (size_t i = 0; i < APS_ROI_REGIONS_MAX; i++) {
    // Reset pointers to NULL.
    state->currentFrameEvent[i] = NULL;
  }
}

playbackInfo caerPlaybackInfoGet(playbackHandle handle)
{
    if (handle == NULL) {
      return (NULL);
    } else {
        return &handle->playbackInfo;
    }
}

playbackHandle playbackOpen(const char *fileName, void (*playbackFinishedCallback) (void*), void *param) {

  playbackHandle handle = calloc(1, sizeof(*handle));
  if (handle == NULL) {
    // Failed to allocate memory for device handle!
    caerLog(CAER_LOG_CRITICAL, __func__, "Failed to allocate memory for device handle.");
    return (NULL);
  }

  playbackState state = &handle->state;
  handle->info.deviceString = calloc(sizeof(char),strlen(fileName)+1);
  memcpy(handle->info.deviceString,fileName,strlen(fileName));
  state->playbackFinishedCallback = playbackFinishedCallback;
  state->callbackParam = param;

  // Initialize state variables to default values (if not zero, taken care of by calloc above).
  atomic_store_explicit(&state->dataExchangeBufferSize, 64, memory_order_relaxed);
  atomic_store_explicit(&state->dataExchangeBlocking, false, memory_order_relaxed);

  // Packet settings (size (in events) and time interval (in µs)).
  atomic_store_explicit(&state->maxPacketContainerPacketSize, 8192, memory_order_relaxed);
  atomic_store_explicit(&state->maxPacketContainerInterval, 10000, memory_order_relaxed);

  atomic_thread_fence(memory_order_release);

  // Set device thread name. Maximum length of 15 chars due to Linux limitations.
  snprintf(state->deviceThreadName, 15 + 1, "%s ID-", fileName);
  state->deviceThreadName[15] = '\0';
  thrd_set_name(state->deviceThreadName);

  // Try to open a DAVIS device on a specific USB port.
  state->file = fopen(fileName,"r");
  if (state->file == NULL) {
    caerLog(CAER_LOG_CRITICAL, __func__, "Failed to open file %s .", fileName);
    return (false);
  }

  char * line = NULL;
  size_t len = 0;
  ssize_t read;

  bool sizeSet = false;
  while ((read = getline(&line, &len, state->file)) != -1) {
    if(strstr(line,"#End Of ASCII Header")){
      printf("End of header found!\n");
      break;
    }

    if(strstr(line,"AEChip: eu.seebetter.ini.chips.davis.DAVIS240C")){
      state->dvsSizeX = 240;
      state->dvsSizeY = 180;
      state->apsSizeX = state->dvsSizeX;
      state->apsSizeY = state->dvsSizeY;
      handle->playbackInfo.sx = state->dvsSizeX;
      handle->playbackInfo.sy = state->dvsSizeY;

      // TODO HARDCODED!
      state->dvsFlipX = true;
      state->dvsFlipY = true;
      state->apsFlipY = true;

      sizeSet = true;
    }
  }
  if(!sizeSet){
    printf("Unsupported camera!\n");
    return false;
  }
  // TODO
  // See DAVIS.h to read this from file header

  // Populate info variables based on data from device.
  uint32_t param32 = 0;

  handle->info.deviceID = 0;
  handle->info.deviceSerialNumber[0] = '\0';

  handle->info.logicVersion = -1;
  handle->info.deviceIsMaster = -1;
  handle->info.logicClock = -1;
  handle->info.adcClock = -1;
  handle->info.chipID = -1;
  handle->info.dvsHasPixelFilter = -1;
  handle->info.dvsHasBackgroundActivityFilter = -1;
  handle->info.dvsHasTestEventGenerator = -1;
  handle->info.apsColorFilter = -1;
  handle->info.apsHasGlobalShutter = -1;
  handle->info.apsHasQuadROI = -1;
  handle->info.apsHasExternalADC = -1;
  handle->info.apsHasInternalADC = -1;
  handle->info.extInputHasGenerator = -1;
  handle->info.extInputHasExtraDetectors = -1;

  // Default IMU settings (for event parsing).
  //spiConfigReceive(state->usbState.deviceHandle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, &param32);
  state->imuAccelScale = 1;//calculateIMUAccelScale(U8T(param32));
  //spiConfigReceive(state->usbState.deviceHandle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, &param32);
  state->imuGyroScale = 1;//calculateIMUGyroScale(U8T(param32));

  // Disable all ROI regions by setting them to -1.
  for (size_t i = 0; i < APS_ROI_REGIONS_MAX; i++) {
    state->apsROISizeX[i] = state->apsROIPositionX[i] = U16T(state->apsSizeX);
    state->apsROISizeY[i] = state->apsROIPositionY[i] = U16T(state->apsSizeY);
  }
  state->apsROIPositionX[0] = 0;
  state->apsROIPositionY[0] = 0;

  // Ignore multi-part events (APS and IMU) at startup, so that any initial
  // incomplete event is ignored. The START events reset this as soon as
  // the first one is observed.
  state->apsIgnoreEvents = true;
  state->imuIgnoreEvents = true;

  //spiConfigReceive(state->usbState.deviceHandle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER, &param32);
  state->apsGlobalShutter = -1;//param32;
  //spiConfigReceive(state->usbState.deviceHandle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RESET_READ, &param32);
  state->apsResetRead = -1;//param32;
  state->startTimestamp = -1;
  state->hostStartTime = -1;
  state->playbackSpeed = 1;
  printf("Parsed file header of %s with success.\n", fileName);

return handle;
}

int playbackClose(playbackHandle handle) {
  playbackState state = &handle->state;

  if(fclose(state->file) == EOF){
    printf("Failed to close file.\n");
  }else{
    printf("Shutdown successful.\n");
  }

  // Free memory.
  free(handle->info.deviceString);
  free(handle);

  return (true);
}

int playbackDataStart(playbackHandle handle) {
  playbackState state = &handle->state;

  // Set wanted time interval to uninitialized. Getting the first TS or TS_RESET
  // will then set this correctly.
  state->currentPacketContainerCommitTimestamp = -1;

  // Initialize RingBuffer.
  state->dataExchangeBuffer = ringBufferInit(atomic_load(&state->dataExchangeBufferSize));
  if (state->dataExchangeBuffer == NULL) {
    caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to initialize data exchange buffer.");
    return (false);
  }

  // Allocate packets.
  state->currentPacketContainer = caerEventPacketContainerAllocate(DAVIS_EVENT_TYPES);
  if (state->currentPacketContainer == NULL) {
    freeAllDataMemory(state);

    caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate event packet container.");
    return (false);
  }

  state->currentPolarityPacket = caerPolarityEventPacketAllocate(DAVIS_POLARITY_DEFAULT_SIZE,
    I16T(handle->info.deviceID), 0);
  if (state->currentPolarityPacket == NULL) {
    freeAllDataMemory(state);

    caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate polarity event packet.");
    return (false);
  }

  state->currentSpecialPacket = caerSpecialEventPacketAllocate(DAVIS_SPECIAL_DEFAULT_SIZE,
    I16T(handle->info.deviceID), 0);
  if (state->currentSpecialPacket == NULL) {
    freeAllDataMemory(state);

    caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate special event packet.");
    return (false);
  }

  state->currentFramePacket = caerFrameEventPacketAllocate(DAVIS_FRAME_DEFAULT_SIZE, I16T(handle->info.deviceID), 0,
    state->apsSizeX, state->apsSizeY, 1);
  if (state->currentFramePacket == NULL) {
    freeAllDataMemory(state);

    caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate frame event packet.");
    return (false);
  }

  // Allocate memory for the current FrameEvents. Use contiguous memory for all ROI FrameEvents.
  size_t eventSize = (sizeof(struct caer_frame_event) - sizeof(uint16_t))
    + ((size_t) state->apsSizeX * (size_t) state->apsSizeY * APS_ADC_CHANNELS * sizeof(uint16_t));

  state->currentFrameEvent[0] = calloc(APS_ROI_REGIONS_MAX, eventSize);
  if (state->currentFrameEvent[0] == NULL) {
    freeAllDataMemory(state);

    caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate ROI frame events.");
    return (false);
  }

  for (size_t i = 1; i < APS_ROI_REGIONS_MAX; i++) {
    // Assign the right memory offset to the pointers into the block that
    // contains all the ROI FrameEvents.
    state->currentFrameEvent[i] = (caerFrameEvent) (((uint8_t*) state->currentFrameEvent[0]) + (i * eventSize));
  }

  state->currentIMU6Packet = caerIMU6EventPacketAllocate(DAVIS_IMU_DEFAULT_SIZE, I16T(handle->info.deviceID), 0);
  if (state->currentIMU6Packet == NULL) {
    freeAllDataMemory(state);

    caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate IMU6 event packet.");
    return (false);
  }

  state->currentSamplePacket = caerSampleEventPacketAllocate(DAVIS_SAMPLE_DEFAULT_SIZE, I16T(handle->info.deviceID),
    0);
  if (state->currentSamplePacket == NULL) {
    freeAllDataMemory(state);

    caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate Sample event packet.");
    return (false);
  }

  state->apsCurrentResetFrame = calloc((size_t) (state->apsSizeX * state->apsSizeY * APS_ADC_CHANNELS),
    sizeof(uint16_t));
  if (state->apsCurrentResetFrame == NULL) {
    freeAllDataMemory(state);

    caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate APS reset frame memory.");
    return (false);
  }

  if ((errno = thrd_create(&state->dataAcquisitionThread, &playbackDataAcquisitionThread, handle)) != thrd_success) {
    freeAllDataMemory(state);

    caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to start data acquisition thread. Error: %d.",
    errno);
    return (false);
  }

  // Wait for the data acquisition thread to be ready.
  while (!atomic_load_explicit(&state->dataAcquisitionThreadRun, memory_order_relaxed)) {
    ;
  }

  return (true);
}

int playbackDataStop(playbackHandle cdh) {
  playbackHandle handle = (playbackHandle) cdh;
  playbackState state = &handle->state;

  atomic_store(&state->dataAcquisitionThreadRun, false);

  // Wait for data acquisition thread to terminate...
  if ((errno = thrd_join(state->dataAcquisitionThread, NULL)) != thrd_success) {
    // This should never happen!
    caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to join data acquisition thread. Error: %d.",
    errno);
    return (false);
  }

  // Empty ringbuffer.
  caerEventPacketContainer container;
  while ((container = ringBufferGet(state->dataExchangeBuffer)) != NULL) {
    // Free container, which will free its subordinate packets too.
    caerEventPacketContainerFree(container);
  }

  // Free current, uncommitted packets and ringbuffer.
  freeAllDataMemory(state);

  // Reset packet positions.
  state->currentPolarityPacketPosition = 0;
  state->currentSpecialPacketPosition = 0;
  state->currentFramePacketPosition = 0;
  state->currentIMU6PacketPosition = 0;
  state->currentSamplePacketPosition = 0;

  // Reset private composite events. 'currentFrameEvent' is taken care of in freeAllDataMemory().
  memset(&state->currentIMU6Event, 0, sizeof(struct caer_imu6_event));

  return (true);
}

caerEventPacketContainer playbackDataGet(playbackHandle cdh) {
  playbackHandle handle = (playbackHandle) cdh;
  playbackState state = &handle->state;
  caerEventPacketContainer container = NULL;

  retry: container = ringBufferGet(state->dataExchangeBuffer);

  if (container != NULL) {
    return (container);
  }

  // Didn't find any event container, either report this or retry, depending
  // on blocking setting.
  if (atomic_load_explicit(&state->dataExchangeBlocking, memory_order_relaxed)) {
    // Don't retry right away in a tight loop, back off and wait a little.
    // If no data is available, sleep for a millisecond to avoid wasting resources.
    struct timespec noDataSleep = { .tv_sec = 0, .tv_nsec = 1000000 };
    if (thrd_sleep(&noDataSleep, NULL) == 0) {
      goto retry;
    }
  }

  // Nothing.
  return (NULL);
}

#define TS_WRAP_ADD 0x8000

static inline int64_t generateFullTimestamp(int32_t tsOverflow, int32_t timestamp) {
  return (I64T((U64T(tsOverflow) << TS_OVERFLOW_SHIFT) | U64T(timestamp)));
}

static inline void initContainerCommitTimestamp(playbackState state) {
  if (state->currentPacketContainerCommitTimestamp == -1) {
    state->currentPacketContainerCommitTimestamp = state->currentTimestamp
      + I32T(atomic_load_explicit(&state->maxPacketContainerInterval, memory_order_relaxed)) - 1;
  }
}

void playbackChangeSpeed(playbackHandle handle, float speed)
{
    if(handle != NULL && speed > 0 ){

        handle->state.startTimestamp = -1;
        atomic_store_explicit(&handle->state.playbackSpeed,speed, memory_order_relaxed);

    }
}

static void playbackDavisEventTranslator(void *vhd, uint8_t *buffer, size_t bytesSent) {
  playbackHandle handle = vhd;
  playbackState state = &handle->state;

  // Return right away if not running anymore. This prevents useless work if many
  // buffers are still waiting when shut down, as well as incorrect event sequences
  // if a TS_RESET is stuck on ring-buffer commit further down, and detects shut-down;
  // then any subsequent buffers should also detect shut-down and not be handled.
  if (!atomic_load_explicit(&state->dataAcquisitionThreadRun, memory_order_relaxed)) {
    return;
  }

  for (size_t i = 0; i < bytesSent; i += 8) {
    // Allocate new packets for next iteration as needed.
    if (state->currentPacketContainer == NULL) {
      state->currentPacketContainer = caerEventPacketContainerAllocate(DAVIS_EVENT_TYPES);
      if (state->currentPacketContainer == NULL) {
        caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate event packet container.");
        return;
      }
    }

    if (state->currentPolarityPacket == NULL) {
      state->currentPolarityPacket = caerPolarityEventPacketAllocate(
      DAVIS_POLARITY_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
      if (state->currentPolarityPacket == NULL) {
        caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate polarity event packet.");
        return;
      }
    }
    else if (state->currentPolarityPacketPosition
      >= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentPolarityPacket)) {
      // If not committed, let's check if any of the packets has reached its maximum
      // capacity limit. If yes, we grow them to accomodate new events.
      caerPolarityEventPacket grownPacket = (caerPolarityEventPacket) caerEventPacketGrow(
        (caerEventPacketHeader) state->currentPolarityPacket, state->currentPolarityPacketPosition * 2);
      if (grownPacket == NULL) {
        caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to grow polarity event packet.");
        return;
      }

      state->currentPolarityPacket = grownPacket;
    }

    if (state->currentSpecialPacket == NULL) {
      state->currentSpecialPacket = caerSpecialEventPacketAllocate(
      DAVIS_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
      if (state->currentSpecialPacket == NULL) {
        caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate special event packet.");
        return;
      }
    }
    else if (state->currentSpecialPacketPosition
      >= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentSpecialPacket)) {
      // If not committed, let's check if any of the packets has reached its maximum
      // capacity limit. If yes, we grow them to accomodate new events.
      caerSpecialEventPacket grownPacket = (caerSpecialEventPacket) caerEventPacketGrow(
        (caerEventPacketHeader) state->currentSpecialPacket, state->currentSpecialPacketPosition * 2);
      if (grownPacket == NULL) {
        caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to grow special event packet.");
        return;
      }

      state->currentSpecialPacket = grownPacket;
    }

    if (state->currentFramePacket == NULL) {
      state->currentFramePacket = caerFrameEventPacketAllocate(
      DAVIS_FRAME_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow, state->apsSizeX,
        state->apsSizeY, 1);
      if (state->currentFramePacket == NULL) {
        caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate frame event packet.");
        return;
      }
    }
    else if (state->currentFramePacketPosition
      >= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentFramePacket)) {
      // If not committed, let's check if any of the packets has reached its maximum
      // capacity limit. If yes, we grow them to accomodate new events.
      caerFrameEventPacket grownPacket = (caerFrameEventPacket) caerEventPacketGrow(
        (caerEventPacketHeader) state->currentFramePacket, state->currentFramePacketPosition * 2);
      if (grownPacket == NULL) {
        caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to grow frame event packet.");
        return;
      }

      state->currentFramePacket = grownPacket;
    }

    if (state->currentIMU6Packet == NULL) {
      state->currentIMU6Packet = caerIMU6EventPacketAllocate(
      DAVIS_IMU_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
      if (state->currentIMU6Packet == NULL) {
        caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate IMU6 event packet.");
        return;
      }
    }
    else if (state->currentIMU6PacketPosition
      >= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentIMU6Packet)) {
      // If not committed, let's check if any of the packets has reached its maximum
      // capacity limit. If yes, we grow them to accomodate new events.
      caerIMU6EventPacket grownPacket = (caerIMU6EventPacket) caerEventPacketGrow(
        (caerEventPacketHeader) state->currentIMU6Packet, state->currentIMU6PacketPosition * 2);
      if (grownPacket == NULL) {
        caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to grow IMU6 event packet.");
        return;
      }

      state->currentIMU6Packet = grownPacket;
    }

    if (state->currentSamplePacket == NULL) {
      state->currentSamplePacket = caerSampleEventPacketAllocate(
      DAVIS_SAMPLE_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
      if (state->currentSamplePacket == NULL) {
        caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate Sample event packet.");
        return;
      }
    }
    else if (state->currentSamplePacketPosition
      >= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentSamplePacket)) {
      // If not committed, let's check if any of the packets has reached its maximum
      // capacity limit. If yes, we grow them to accomodate new events.
      caerSampleEventPacket grownPacket = (caerSampleEventPacket) caerEventPacketGrow(
        (caerEventPacketHeader) state->currentSamplePacket, state->currentSamplePacketPosition * 2);
      if (grownPacket == NULL) {
        caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to grow Sample event packet.");
        return;
      }

      state->currentSamplePacket = grownPacket;
    }

    bool tsReset = false;
    bool tsBigWrap = false;

    //uint32_t event = *((uint32_t * ) (buffer+i));
    //uint32_t timestamp = le32toh(*((uint32_t * ) (buffer+i+4)));

    int32_t event = (int32_t)((uint8_t)buffer[i+0] << 0x18);
        event |= (int32_t)((uint8_t)buffer[i+1] << 0x10);
        event |= (int32_t)((uint8_t)buffer[i+2] << 0x08);
        event |= (int32_t)((uint8_t)buffer[i+3] << 0x00);

    int32_t timestamp = (int32_t)((uint8_t)buffer[i+4] << 0x18);
        timestamp |= (int32_t)((uint8_t)buffer[i+5] << 0x10);
        timestamp |= (int32_t)((uint8_t)buffer[i+6] << 0x08);
        timestamp |= (int32_t)((uint8_t)buffer[i+7] << 0x00);

    //printf("%d %d %d %d\n",buffer[i+0],buffer[i+1],buffer[i+2],buffer[i+3]);
    //printf("%d %d %d %d\n",buffer[i+4],buffer[i+5],buffer[i+6],buffer[i+7]);
    //printf("%d\n",timestamp);

    // APS event
    if(((uint32_t)event >> 31)){
        uint8_t imuSample = (event >> 11) & 0x01;

        if(imuSample)
            continue;

        uint8_t signalRead = (event >> 10) & 0x01;
        uint16_t intensity = (event >> 0) & 0x1FF;

        uint16_t x,y;
        if(state->apsFlipX){
            x = state->dvsSizeX -1 -(event & 0x003FF000) >> 12;
        }
        else{
            x = (event & 0x003FF000) >> 12;
        }
        if(state->apsFlipY){
            y = state->dvsSizeY -1 - ((event & 0x7FC00000) >> 22);
        }
        else{
            y = (event & 0x7FC00000) >> 22;
        }


        size_t pixelPosition = (size_t) (y * state->dvsSizeX) + x;

        // Signal read
        if(signalRead){

            if(state->lastAPSEventWasReset){
                initFrame(handle);
                state->pixelReceived = 0;
            }
            uint16_t resetValue = 0;
            uint16_t signalValue = 0;

            resetValue = state->apsCurrentResetFrame[pixelPosition];
            signalValue = intensity;

            int32_t pixelValue = 0;
            //if (resetValue < 512 || signalValue == 0) {
              // If the signal value is 0, that is only possible if the camera
              // has seen tons of light. In that case, the photo-diode current
              // may be greater than the reset current, and the reset value
              // never goes back up fully, which results in black spots where
              // there is too much light. This confuses algorithms, so we filter
              // this out here by setting the pixel to white in that case.
              // Another effect of the same thing is the reset value not going
              // back up to a decent value, so we also filter that out here.
              //pixelValue = 1023;
              //printf("Reset");
            //}
            //else {
              // Do CDS.
              pixelValue = resetValue - signalValue;

              // Check for underflow.
              pixelValue = (pixelValue < 0) ? (0) : (pixelValue);

              // Check for overflow.
              pixelValue = (pixelValue > 1023) ? (1023) : (pixelValue);
            //}

            // Normalize the ADC value to 16bit generic depth. This depends on ADC used.
            pixelValue = pixelValue << (16 - APS_ADC_DEPTH);

            caerFrameEventGetPixelArrayUnsafe(state->currentFrameEvent[0])[pixelPosition] = htole16(
              (uint16_t)(pixelValue));

            //printf("Pixel %zu: %d\n",pixelPosition,pixelValue);

            state->lastAPSEventWasReset = false;
            state->pixelReceived++;
        }
        // Reset read
        else{
            if(!state->lastAPSEventWasReset){
                if(state->pixelReceived == state->apsSizeX*state->apsSizeY){
                    caerFrameEventValidate(state->currentFrameEvent[0], state->currentFramePacket);
                    // IMU6 and APS operate on an internal event and copy that to the actual output
                    // packet here, in the END state, for a reason: if a packetContainer, with all its
                    // packets, is committed due to hitting any of the triggers that are not TS reset
                    // or TS wrap-around related, like number of polarity events, the event in the packet
                    // would be left incomplete, and the event in the new packet would be corrupted.
                    // We could avoid this like for the TS reset/TS wrap-around case (see forceCommit) by
                    // just deleting that event, but these kinds of commits happen much more often and the
                    // possible data loss would be too significant. So instead we keep a private event,
                    // fill it, and then only copy it into the packet here in the END state, at which point
                    // the whole event is ready and cannot be broken/corrupted in any way anymore.
                    caerFrameEvent currentFrameEvent = caerFrameEventPacketGetEvent(
                      state->currentFramePacket, state->currentFramePacketPosition);
                    memcpy(currentFrameEvent, state->currentFrameEvent[0],
                      (sizeof(struct caer_frame_event) - sizeof(uint16_t))
                        + caerFrameEventGetPixelsSize(state->currentFrameEvent[0]));
                    state->currentFramePacketPosition++;
                }else{
                        //printf("Partial frame: %d of %d\n",state->pixelReceived,state->apsSizeX*state->apsSizeY);
                    }
                state->pixelReceived = 0;
            }
            //printf("Reset Pixel %zu: %d\n",pixelPosition,intensity);
            state->apsCurrentResetFrame[pixelPosition] = intensity;
            state->lastAPSEventWasReset = true;
            state->pixelReceived++;
        }
        /*
            bool validFrame = true;

            for (size_t j = 0; j < APS_READOUT_TYPES_NUM; j++) {
              int32_t checkValue = caerFrameEventGetLengthX(state->currentFrameEvent[0]);

              // Check main reset read against zero if disabled.
              if (j == APS_READOUT_RESET && !state->apsResetRead) {
                checkValue = 0;
              }

              caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "APS Frame End: CountX[%zu] is %d.",
                j, state->apsCountX[j]);

              if (state->apsCountX[j] != checkValue) {
                caerLog(CAER_LOG_ERROR, handle->info.deviceString,
                  "APS Frame End - %zu: wrong column count %d detected, expected %d.", j,
                  state->apsCountX[j], checkValue);
                validFrame = false;
              }
            }

            // Write out end of frame timestamp.
            caerFrameEventSetTSEndOfFrame(state->currentFrameEvent[0], state->currentTimestamp);

            // Send APS info event out (as special event).
            caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
              state->currentSpecialPacket, state->currentSpecialPacketPosition);
            caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
            caerSpecialEventSetType(currentSpecialEvent, APS_FRAME_END);
            caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
            state->currentSpecialPacketPosition++;

            // Validate event and advance frame packet position.
            if (validFrame) {
              caerFrameEventValidate(state->currentFrameEvent[0], state->currentFramePacket);

              // Invert X and Y axes if image from chip is inverted.
              if (state->apsInvertXY) {
                SWAP_VAR(int32_t, state->currentFrameEvent[0]->lengthX,
                  state->currentFrameEvent[0]->lengthY);
                SWAP_VAR(int32_t, state->currentFrameEvent[0]->positionX,
                  state->currentFrameEvent[0]->positionY);
              }

              // IMU6 and APS operate on an internal event and copy that to the actual output
              // packet here, in the END state, for a reason: if a packetContainer, with all its
              // packets, is committed due to hitting any of the triggers that are not TS reset
              // or TS wrap-around related, like number of polarity events, the event in the packet
              // would be left incomplete, and the event in the new packet would be corrupted.
              // We could avoid this like for the TS reset/TS wrap-around case (see forceCommit) by
              // just deleting that event, but these kinds of commits happen much more often and the
              // possible data loss would be too significant. So instead we keep a private event,
              // fill it, and then only copy it into the packet here in the END state, at which point
              // the whole event is ready and cannot be broken/corrupted in any way anymore.
              caerFrameEvent currentFrameEvent = caerFrameEventPacketGetEvent(
                state->currentFramePacket, state->currentFramePacketPosition);
              memcpy(currentFrameEvent, state->currentFrameEvent[0],
                (sizeof(struct caer_frame_event) - sizeof(uint16_t))
                  + caerFrameEventGetPixelsSize(state->currentFrameEvent[0]));
              state->currentFramePacketPosition++;

            }

            state->apsResetRead = true;
            initFrame(handle);
        }*/
    }
    // DVS event
    else{
        // External event
        if((event >> 10 ) & 0x01){
            continue;
        }

        if(state->startTimestamp == -1){
            state->startTimestamp = timestamp;
            state->hostStartTime = getCurrTime();
        }else{
            float speed = atomic_load_explicit(&state->playbackSpeed,memory_order_relaxed);
            uint64_t currTime = getCurrTime();
            int64_t hostTimeDiff = (currTime-state->hostStartTime)*speed;
            int64_t camTimeDiff = timestamp-state->startTimestamp;
            int64_t diff = camTimeDiff-hostTimeDiff;

            if(diff > 0)
            {
                usleep(diff);
            }
        }

        // Is a timestamp! Expand to 32 bits. (Tick is 1µs already.)
        state->lastTimestamp = state->currentTimestamp;
        state->currentTimestamp = state->wrapAdd + timestamp;
        initContainerCommitTimestamp(state);

        // Check monotonicity of timestamps.
        checkStrictMonotonicTimestamp(handle);

        // Look at the code, to determine event and data type.
    //    uint8_t code = U8T((event & 0x70000000) >> 12);
    //    uint16_t data = (event & 0x0FFF);

        caerPolarityEvent currentPolarityEvent = caerPolarityEventPacketGetEvent(
          state->currentPolarityPacket, state->currentPolarityPacketPosition);

        // Timestamp at event-stream insertion point.
        caerPolarityEventSetTimestamp(currentPolarityEvent, state->currentTimestamp);
        uint8_t polarity = event >> 11 & 0x01;//((IS_DAVIS208(handle->info.chipID)) && (data < 192)) ? U8T(~code) : (code);
        caerPolarityEventSetPolarity(currentPolarityEvent, polarity );

        if(state->dvsFlipX){
            caerPolarityEventSetX(currentPolarityEvent, state->dvsSizeX -1 - ((event & 0x003FF000) >> 12));
        }
        else{
            caerPolarityEventSetX(currentPolarityEvent, (event & 0x003FF000) >> 12);
        }
        if(state->dvsFlipY){
            caerPolarityEventSetY(currentPolarityEvent, state->dvsSizeY -1 - ((event & 0x7FC00000) >> 22));
        }
        else{
            caerPolarityEventSetY(currentPolarityEvent, (event & 0x7FC00000) >> 22);
        }

        caerPolarityEventValidate(currentPolarityEvent, state->currentPolarityPacket);
        state->currentPolarityPacketPosition++;
    }
/*
    switch (code) {
      case 0: // Special event
        switch (data) {


          case 8: { // APS Global Shutter Frame Start
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "APS GS Frame Start event received.");
            state->apsIgnoreEvents = false;
            state->apsGlobalShutter = true;
            state->apsResetRead = true;

            initFrame(handle);

            break;
          }

          case 9: { // APS Rolling Shutter Frame Start
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "APS RS Frame Start event received.");
            state->apsIgnoreEvents = false;
            state->apsGlobalShutter = false;
            state->apsResetRead = true;

            initFrame(handle);

            break;
          }

          case 10: { // APS Frame End
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "APS Frame End event received.");
            if (state->apsIgnoreEvents) {
              break;
            }

            bool validFrame = true;

            for (size_t j = 0; j < APS_READOUT_TYPES_NUM; j++) {
              int32_t checkValue = caerFrameEventGetLengthX(state->currentFrameEvent[0]);

              // Check main reset read against zero if disabled.
              if (j == APS_READOUT_RESET && !state->apsResetRead) {
                checkValue = 0;
              }

              caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "APS Frame End: CountX[%zu] is %d.",
                j, state->apsCountX[j]);

              if (state->apsCountX[j] != checkValue) {
                caerLog(CAER_LOG_ERROR, handle->info.deviceString,
                  "APS Frame End - %zu: wrong column count %d detected, expected %d.", j,
                  state->apsCountX[j], checkValue);
                validFrame = false;
              }
            }

            // Write out end of frame timestamp.
            caerFrameEventSetTSEndOfFrame(state->currentFrameEvent[0], state->currentTimestamp);

            // Send APS info event out (as special event).
            caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
              state->currentSpecialPacket, state->currentSpecialPacketPosition);
            caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
            caerSpecialEventSetType(currentSpecialEvent, APS_FRAME_END);
            caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
            state->currentSpecialPacketPosition++;

            // Validate event and advance frame packet position.
            if (validFrame) {
              caerFrameEventValidate(state->currentFrameEvent[0], state->currentFramePacket);

              // Invert X and Y axes if image from chip is inverted.
              if (state->apsInvertXY) {
                SWAP_VAR(int32_t, state->currentFrameEvent[0]->lengthX,
                  state->currentFrameEvent[0]->lengthY);
                SWAP_VAR(int32_t, state->currentFrameEvent[0]->positionX,
                  state->currentFrameEvent[0]->positionY);
              }

              // IMU6 and APS operate on an internal event and copy that to the actual output
              // packet here, in the END state, for a reason: if a packetContainer, with all its
              // packets, is committed due to hitting any of the triggers that are not TS reset
              // or TS wrap-around related, like number of polarity events, the event in the packet
              // would be left incomplete, and the event in the new packet would be corrupted.
              // We could avoid this like for the TS reset/TS wrap-around case (see forceCommit) by
              // just deleting that event, but these kinds of commits happen much more often and the
              // possible data loss would be too significant. So instead we keep a private event,
              // fill it, and then only copy it into the packet here in the END state, at which point
              // the whole event is ready and cannot be broken/corrupted in any way anymore.
              caerFrameEvent currentFrameEvent = caerFrameEventPacketGetEvent(
                state->currentFramePacket, state->currentFramePacketPosition);
              memcpy(currentFrameEvent, state->currentFrameEvent[0],
                (sizeof(struct caer_frame_event) - sizeof(uint16_t))
                  + caerFrameEventGetPixelsSize(state->currentFrameEvent[0]));
              state->currentFramePacketPosition++;

            }

            break;
          }

          case 11: { // APS Reset Column Start
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
              "APS Reset Column Start event received.");
            if (state->apsIgnoreEvents) {
              break;
            }

            state->apsCurrentReadoutType = APS_READOUT_RESET;
            state->apsCountY[state->apsCurrentReadoutType] = 0;

            state->apsRGBPixelOffsetDirection = 0;
            state->apsRGBPixelOffset = 1; // RGB support, first pixel of row always even.

            // The first Reset Column Read Start is also the start
            // of the exposure for the RS.
            if (!state->apsGlobalShutter && state->apsCountX[APS_READOUT_RESET] == 0) {
              caerFrameEventSetTSStartOfExposure(state->currentFrameEvent[0],
                state->currentTimestamp);

              // Send APS info event out (as special event).
              caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
                state->currentSpecialPacket, state->currentSpecialPacketPosition);
              caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
              caerSpecialEventSetType(currentSpecialEvent, APS_EXPOSURE_START);
              caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
              state->currentSpecialPacketPosition++;
            }

            break;
          }

          case 12: { // APS Signal Column Start
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
              "APS Signal Column Start event received.");
            if (state->apsIgnoreEvents) {
              break;
            }

            state->apsCurrentReadoutType = APS_READOUT_SIGNAL;
            state->apsCountY[state->apsCurrentReadoutType] = 0;

            state->apsRGBPixelOffsetDirection = 0;
            state->apsRGBPixelOffset = 1; // RGB support, first pixel of row always even.

            // The first Signal Column Read Start is also always the end
            // of the exposure time, for both RS and GS.
            if (state->apsCountX[APS_READOUT_SIGNAL] == 0) {
              caerFrameEventSetTSEndOfExposure(state->currentFrameEvent[0], state->currentTimestamp);

              // Send APS info event out (as special event).
              caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
                state->currentSpecialPacket, state->currentSpecialPacketPosition);
              caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
              caerSpecialEventSetType(currentSpecialEvent, APS_EXPOSURE_END);
              caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
              state->currentSpecialPacketPosition++;
            }

            break;
          }

          case 13: { // APS Column End
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "APS Column End event received.");
            if (state->apsIgnoreEvents) {
              break;
            }

            caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "APS Column End: CountX[%d] is %d.",
              state->apsCurrentReadoutType, state->apsCountX[state->apsCurrentReadoutType]);
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "APS Column End: CountY[%d] is %d.",
              state->apsCurrentReadoutType, state->apsCountY[state->apsCurrentReadoutType]);

            if (state->apsCountY[state->apsCurrentReadoutType]
              != caerFrameEventGetLengthY(state->currentFrameEvent[0])) {
              caerLog(CAER_LOG_ERROR, handle->info.deviceString,
                "APS Column End - %d: wrong row count %d detected, expected %d.",
                state->apsCurrentReadoutType, state->apsCountY[state->apsCurrentReadoutType],
                caerFrameEventGetLengthY(state->currentFrameEvent[0]));
            }

            state->apsCountX[state->apsCurrentReadoutType]++;

            // The last Reset Column Read End is also the start
            // of the exposure for the GS.
            if (state->apsGlobalShutter && state->apsCurrentReadoutType == APS_READOUT_RESET
              && state->apsCountX[APS_READOUT_RESET]
                == caerFrameEventGetLengthX(state->currentFrameEvent[0])) {
              caerFrameEventSetTSStartOfExposure(state->currentFrameEvent[0],
                state->currentTimestamp);

              // Send APS info event out (as special event).
              caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
                state->currentSpecialPacket, state->currentSpecialPacketPosition);
              caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
              caerSpecialEventSetType(currentSpecialEvent, APS_EXPOSURE_START);
              caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
              state->currentSpecialPacketPosition++;
            }

            break;
          }

          case 14: { // APS Global Shutter Frame Start with no Reset Read
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
              "APS GS NORST Frame Start event received.");
            state->apsIgnoreEvents = false;
            state->apsGlobalShutter = true;
            state->apsResetRead = false;

            initFrame(handle);

            // If reset reads are disabled, the start of exposure is closest to
            // the start of frame.
            caerFrameEventSetTSStartOfExposure(state->currentFrameEvent[0], state->currentTimestamp);

            // No APS info event is sent out (as special event). Only one event
            // per type can be sent out per cycle, and initFrame() already does
            // that and sets APS_FRAME_START.

            break;
          }

          case 15: { // APS Rolling Shutter Frame Start with no Reset Read
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
              "APS RS NORST Frame Start event received.");
            state->apsIgnoreEvents = false;
            state->apsGlobalShutter = false;
            state->apsResetRead = false;

            initFrame(handle);

            // If reset reads are disabled, the start of exposure is closest to
            // the start of frame.
            caerFrameEventSetTSStartOfExposure(state->currentFrameEvent[0], state->currentTimestamp);

            // No APS info event is sent out (as special event). Only one event
            // per type can be sent out per cycle, and initFrame() already does
            // that and sets APS_FRAME_START.

            break;
          }

          case 16:
          case 17:
          case 18:
          case 19:
          case 20:
          case 21:
          case 22:
          case 23:
          case 24:
          case 25:
          case 26:
          case 27:
          case 28:
          case 29:
          case 30:
          case 31: {
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
              "IMU Scale Config event (%" PRIu16 ") received.", data);
            if (state->imuIgnoreEvents) {
              break;
            }

            // Set correct IMU accel and gyro scales, used to interpret subsequent
            // IMU samples from the device.
            state->imuAccelScale = calculateIMUAccelScale((data >> 2) & 0x03);
            state->imuGyroScale = calculateIMUGyroScale(data & 0x03);

            // At this point the IMU event count should be zero (reset by start).
            if (state->imuCount != 0) {
              caerLog(CAER_LOG_INFO, handle->info.deviceString,
                "IMU Scale Config: previous IMU start event missed, attempting recovery.");
            }

            // Increase IMU count by one, to a total of one (0+1=1).
            // This way we can recover from the above error of missing start, and we can
            // later discover if the IMU Scale Config event actually arrived itself.
            state->imuCount = 1;

            break;
          }

          case 32: {
            // Next Misc8 APS ROI Size events will refer to ROI region 0.
            // 0/1 used to distinguish between X and Y sizes.
            state->apsROIUpdate = (0 << 2);
            state->apsROISizeX[0] = state->apsROIPositionX[0] = U16T(state->apsSizeX);
            state->apsROISizeY[0] = state->apsROIPositionY[0] = U16T(state->apsSizeY);
            break;
          }

          case 33: {
            // Next Misc8 APS ROI Size events will refer to ROI region 1.
            // 2/3 used to distinguish between X and Y sizes.
            state->apsROIUpdate = (1 << 2);
            state->apsROISizeX[1] = state->apsROIPositionX[1] = U16T(state->apsSizeX);
            state->apsROISizeY[1] = state->apsROIPositionY[1] = U16T(state->apsSizeY);
            break;
          }

          case 34: {
            // Next Misc8 APS ROI Size events will refer to ROI region 2.
            // 4/5 used to distinguish between X and Y sizes.
            state->apsROIUpdate = (2 << 2);
            state->apsROISizeX[2] = state->apsROIPositionX[2] = U16T(state->apsSizeX);
            state->apsROISizeY[2] = state->apsROIPositionY[2] = U16T(state->apsSizeY);
            break;
          }

          case 35: {
            // Next Misc8 APS ROI Size events will refer to ROI region 3.
            // 6/7 used to distinguish between X and Y sizes.
            state->apsROIUpdate = (3 << 2);
            state->apsROISizeX[3] = state->apsROIPositionX[3] = U16T(state->apsSizeX);
            state->apsROISizeY[3] = state->apsROIPositionY[3] = U16T(state->apsSizeY);
            break;
          }

          case 36: { // External input 1 (falling edge)
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
              "External input 1 (falling edge) event received.");

            caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
              state->currentSpecialPacket, state->currentSpecialPacketPosition);
            caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
            caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT1_FALLING_EDGE);
            caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
            state->currentSpecialPacketPosition++;
            break;
          }

          case 37: { // External input 1 (rising edge)
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
              "External input 1 (rising edge) event received.");

            caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
              state->currentSpecialPacket, state->currentSpecialPacketPosition);
            caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
            caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT1_RISING_EDGE);
            caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
            state->currentSpecialPacketPosition++;
            break;
          }

          case 38: { // External input 1 (pulse)
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
              "External input 1 (pulse) event received.");

            caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
              state->currentSpecialPacket, state->currentSpecialPacketPosition);
            caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
            caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT1_PULSE);
            caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
            state->currentSpecialPacketPosition++;
            break;
          }

          case 39: { // External input 2 (falling edge)
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
              "External input 2 (falling edge) event received.");

            caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
              state->currentSpecialPacket, state->currentSpecialPacketPosition);
            caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
            caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT2_FALLING_EDGE);
            caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
            state->currentSpecialPacketPosition++;
            break;
          }

          case 40: { // External input 2 (rising edge)
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
              "External input 2 (rising edge) event received.");

            caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
              state->currentSpecialPacket, state->currentSpecialPacketPosition);
            caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
            caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT2_RISING_EDGE);
            caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
            state->currentSpecialPacketPosition++;
            break;
          }

          case 41: { // External input 2 (pulse)
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
              "External input 2 (pulse) event received.");

            caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
              state->currentSpecialPacket, state->currentSpecialPacketPosition);
            caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
            caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT2_PULSE);
            caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
            state->currentSpecialPacketPosition++;
            break;
          }

          case 42: { // External generator (falling edge)
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
              "External generator (falling edge) event received.");

            caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
              state->currentSpecialPacket, state->currentSpecialPacketPosition);
            caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
            caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_GENERATOR_FALLING_EDGE);
            caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
            state->currentSpecialPacketPosition++;
            break;
          }

          case 43: { // External generator (rising edge)
            caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
              "External generator (rising edge) event received.");

            caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
              state->currentSpecialPacket, state->currentSpecialPacketPosition);
            caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
            caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_GENERATOR_RISING_EDGE);
            caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
            state->currentSpecialPacketPosition++;
            break;
          }

          default:
            caerLog(CAER_LOG_ERROR, handle->info.deviceString,
              "Caught special event that can't be handled: %d.", data);
            break;
        }
        break;

      case 1: // Y address
        // Check range conformity.
        if (data >= state->dvsSizeY) {
          caerLog(CAER_LOG_ALERT, handle->info.deviceString,
            "DVS: Y address out of range (0-%d): %" PRIu16 ".", state->dvsSizeY - 1, data);
          break; // Skip invalid Y address (don't update lastY).
        }

        if (state->dvsGotY) {
          caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
            state->currentSpecialPacket, state->currentSpecialPacketPosition);

          // Timestamp at event-stream insertion point.
          caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
          caerSpecialEventSetType(currentSpecialEvent, DVS_ROW_ONLY);
          caerSpecialEventSetData(currentSpecialEvent, state->dvsLastY);
          caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
          state->currentSpecialPacketPosition++;

          caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
            "DVS: row-only event received for address Y=%" PRIu16 ".", state->dvsLastY);
        }

        state->dvsLastY = data;
        state->dvsGotY = true;

        break;

      case 2: // X address, Polarity OFF
      case 3: { // X address, Polarity ON
        // Check range conformity.
        if (data >= state->dvsSizeX) {
          caerLog(CAER_LOG_ALERT, handle->info.deviceString,
            "DVS: X address out of range (0-%d): %" PRIu16 ".", state->dvsSizeX - 1, data);
          break; // Skip invalid event.
        }

        // Invert polarity for PixelParade high gain pixels (DavisSense), because of
        // negative gain from pre-amplifier.
        uint8_t polarity = ((IS_DAVIS208(handle->info.chipID)) && (data < 192)) ? U8T(~code) : (code);

        caerPolarityEvent currentPolarityEvent = caerPolarityEventPacketGetEvent(
          state->currentPolarityPacket, state->currentPolarityPacketPosition);

        // Timestamp at event-stream insertion point.
        caerPolarityEventSetTimestamp(currentPolarityEvent, state->currentTimestamp);
        caerPolarityEventSetPolarity(currentPolarityEvent, (polarity & 0x01));
        if (state->dvsInvertXY) {
          // Flip Y address to conform to CG format.
          caerPolarityEventSetY(currentPolarityEvent, U16T((state->dvsSizeX - 1) - data));
          caerPolarityEventSetX(currentPolarityEvent, state->dvsLastY);
        }
        else {
          // Flip Y address to conform to CG format.
          caerPolarityEventSetY(currentPolarityEvent, U16T((state->dvsSizeY - 1) - state->dvsLastY));
          caerPolarityEventSetX(currentPolarityEvent, data);
        }
        caerPolarityEventValidate(currentPolarityEvent, state->currentPolarityPacket);
        state->currentPolarityPacketPosition++;

        state->dvsGotY = false;

        break;
      }

      case 4: {
        if (state->apsIgnoreEvents) {
          break;
        }

        // Let's check that apsCountX is not above the maximum. This could happen
        // if the maximum is a smaller number that comes from ROI, while we're still
        // reading out a frame with a bigger, old size.
        if (state->apsCountX[state->apsCurrentReadoutType]
          >= caerFrameEventGetLengthX(state->currentFrameEvent[0])) {
          caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
            "APS ADC sample: column count is at maximum, discarding further samples.");
          break;
        }

        // Let's check that apsCountY is not above the maximum. This could happen
        // if start/end of column events are discarded (no wait on transfer stall).
        if (state->apsCountY[state->apsCurrentReadoutType]
          >= caerFrameEventGetLengthY(state->currentFrameEvent[0])) {
          caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
            "APS ADC sample: row count is at maximum, discarding further samples.");
          break;
        }

        // If reset read, we store the values in a local array. If signal read, we
        // store the final pixel value directly in the output frame event. We already
        // do the subtraction between reset and signal here, to avoid carrying that
        // around all the time and consuming memory. This way we can also only take
        // infrequent reset reads and re-use them for multiple frames, which can heavily
        // reduce traffic, and should not impact image quality heavily, at least in GS.
        uint16_t xPos =
          (state->apsFlipX) ?
            (U16T(
              caerFrameEventGetLengthX(state->currentFrameEvent[0]) - 1
                - state->apsCountX[state->apsCurrentReadoutType])) :
            (U16T(state->apsCountX[state->apsCurrentReadoutType]));
        uint16_t yPos =
          (state->apsFlipY) ?
            (U16T(
              caerFrameEventGetLengthY(state->currentFrameEvent[0]) - 1
                - state->apsCountY[state->apsCurrentReadoutType])) :
            (U16T(state->apsCountY[state->apsCurrentReadoutType]));

        if (IS_DAVISRGB(handle->info.chipID)) {
          yPos = U16T(yPos + state->apsRGBPixelOffset);
        }

        int32_t stride = 0;

        if (state->apsInvertXY) {
          SWAP_VAR(uint16_t, xPos, yPos);

          stride = caerFrameEventGetLengthY(state->currentFrameEvent[0]);

          // Flip Y address to conform to CG format.
          yPos = U16T(caerFrameEventGetLengthX(state->currentFrameEvent[0]) - 1 - yPos);
        }
        else {
          stride = caerFrameEventGetLengthX(state->currentFrameEvent[0]);

          // Flip Y address to conform to CG format.
          yPos = U16T(caerFrameEventGetLengthY(state->currentFrameEvent[0]) - 1 - yPos);
        }

        size_t pixelPosition = (size_t) (yPos * stride) + xPos;

        // DAVIS240 has a reduced dynamic range due to external
        // ADC high/low ref resistors not having optimal values.
        // To fix this multiply by 1.95 to 2.15, so we choose to
        // just shift by one (multiply by 2.00) for efficiency.
        if (IS_DAVIS240(handle->info.chipID)) {
          data = U16T(data << 1);
        }

        if ((state->apsCurrentReadoutType == APS_READOUT_RESET
          && !(IS_DAVISRGB(handle->info.chipID) && state->apsGlobalShutter))
          || (state->apsCurrentReadoutType == APS_READOUT_SIGNAL
            && (IS_DAVISRGB(handle->info.chipID) && state->apsGlobalShutter))) {
          state->apsCurrentResetFrame[pixelPosition] = data;
        }
        else {
          uint16_t resetValue = 0;
          uint16_t signalValue = 0;

          if (IS_DAVISRGB(handle->info.chipID) && state->apsGlobalShutter) {
            // DAVIS RGB GS has inverted samples, signal read comes first
            // and was stored above inside state->apsCurrentResetFrame.
            resetValue = data;
            signalValue = state->apsCurrentResetFrame[pixelPosition];
          }
          else {
            resetValue = state->apsCurrentResetFrame[pixelPosition];
            signalValue = data;
          }

          int32_t pixelValue = 0;

          if (resetValue < 512 || signalValue == 0) {
            // If the signal value is 0, that is only possible if the camera
            // has seen tons of light. In that case, the photo-diode current
            // may be greater than the reset current, and the reset value
            // never goes back up fully, which results in black spots where
            // there is too much light. This confuses algorithms, so we filter
            // this out here by setting the pixel to white in that case.
            // Another effect of the same thing is the reset value not going
            // back up to a decent value, so we also filter that out here.
            pixelValue = 1023;
          }
          else {
            // Do CDS.
            pixelValue = resetValue - signalValue;

            // Check for underflow.
            pixelValue = (pixelValue < 0) ? (0) : (pixelValue);

            // Check for overflow.
            pixelValue = (pixelValue > 1023) ? (1023) : (pixelValue);
          }

          // Normalize the ADC value to 16bit generic depth. This depends on ADC used.
          pixelValue = pixelValue << (16 - APS_ADC_DEPTH);

          caerFrameEventGetPixelArrayUnsafe(state->currentFrameEvent[0])[pixelPosition] = htole16(
            U16T(pixelValue));
        }

        caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
          "APS ADC Sample: column=%" PRIu16 ", row=%" PRIu16 ", xPos=%" PRIu16 ", yPos=%" PRIu16 ", data=%" PRIu16 ".",
          state->apsCountX[state->apsCurrentReadoutType], state->apsCountY[state->apsCurrentReadoutType],
          xPos, yPos, data);

        state->apsCountY[state->apsCurrentReadoutType]++;

        // RGB support: first 320 pixels are even, then odd.
        if (IS_DAVISRGB(handle->info.chipID)) {
          if (state->apsRGBPixelOffsetDirection == 0) { // Increasing
            state->apsRGBPixelOffset++;

            if (state->apsRGBPixelOffset == 321) {
              // Switch to decreasing after last even pixel.
              state->apsRGBPixelOffsetDirection = 1;
              state->apsRGBPixelOffset = 318;
            }
          }
          else { // Decreasing
            state->apsRGBPixelOffset = I16T(state->apsRGBPixelOffset - 3);
          }
        }

        break;
      }

      case 5: {
        // Misc 8bit data, used currently only
        // for IMU events in DAVIS FX3 boards.
        uint8_t misc8Code = U8T((data & 0x0F00) >> 8);
        uint8_t misc8Data = U8T(data & 0x00FF);

        switch (misc8Code) {
          case 0:
            if (state->imuIgnoreEvents) {
              break;
            }

            // Detect missing IMU end events.
            if (state->imuCount >= IMU6_COUNT) {
              caerLog(CAER_LOG_INFO, handle->info.deviceString,
                "IMU data: IMU samples count is at maximum, discarding further samples.");
              break;
            }

            // IMU data event.
            switch (state->imuCount) {
              case 0:
                caerLog(CAER_LOG_ERROR, handle->info.deviceString,
                  "IMU data: missing IMU Scale Config event. Parsing of IMU events will still be attempted, but be aware that Accel/Gyro scale conversions may be inaccurate.");
                state->imuCount = 1;
                // Fall through to next case, as if imuCount was equal to 1.

              case 1:
              case 3:
              case 5:
              case 7:
              case 9:
              case 11:
              case 13:
                state->imuTmpData = misc8Data;
                break;

              case 2: {
                int16_t accelX = I16T((state->imuTmpData << 8) | misc8Data);
                if (state->imuFlipX) {
                  accelX = I16T(-accelX);
                }
                caerIMU6EventSetAccelX(&state->currentIMU6Event, accelX / state->imuAccelScale);
                break;
              }

              case 4: {
                int16_t accelY = I16T((state->imuTmpData << 8) | misc8Data);
                if (state->imuFlipY) {
                  accelY = I16T(-accelY);
                }
                caerIMU6EventSetAccelY(&state->currentIMU6Event, accelY / state->imuAccelScale);
                break;
              }

              case 6: {
                int16_t accelZ = I16T((state->imuTmpData << 8) | misc8Data);
                if (state->imuFlipZ) {
                  accelZ = I16T(-accelZ);
                }
                caerIMU6EventSetAccelZ(&state->currentIMU6Event, accelZ / state->imuAccelScale);
                break;
              }

                // Temperature is signed. Formula for converting to °C:
                // (SIGNED_VAL / 340) + 36.53
              case 8: {
                int16_t temp = I16T((state->imuTmpData << 8) | misc8Data);
                caerIMU6EventSetTemp(&state->currentIMU6Event, (temp / 340.0f) + 36.53f);
                break;
              }

              case 10: {
                int16_t gyroX = I16T((state->imuTmpData << 8) | misc8Data);
                if (state->imuFlipX) {
                  gyroX = I16T(-gyroX);
                }
                caerIMU6EventSetGyroX(&state->currentIMU6Event, gyroX / state->imuGyroScale);
                break;
              }

              case 12: {
                int16_t gyroY = I16T((state->imuTmpData << 8) | misc8Data);
                if (state->imuFlipY) {
                  gyroY = I16T(-gyroY);
                }
                caerIMU6EventSetGyroY(&state->currentIMU6Event, gyroY / state->imuGyroScale);
                break;
              }

              case 14: {
                int16_t gyroZ = I16T((state->imuTmpData << 8) | misc8Data);
                if (state->imuFlipZ) {
                  gyroZ = I16T(-gyroZ);
                }
                caerIMU6EventSetGyroZ(&state->currentIMU6Event, gyroZ / state->imuGyroScale);
                break;
              }
            }

            state->imuCount++;

            break;

          case 1:
            // APS ROI Size Part 1 (bits 15-8).
            // Here we just store the temporary value, and use it again
            // in the next case statement.
            state->apsROITmpData = U16T(misc8Data << 8);

            break;

          case 2: {
            // APS ROI Size Part 2 (bits 7-0).
            // Here we just store the values and re-use the four fields
            // sizeX/Y and positionX/Y to store endCol/Row and startCol/Row.
            // We then recalculate all the right values and set everything
            // up in START_FRAME.
            size_t apsROIRegion = state->apsROIUpdate >> 2;

            switch (state->apsROIUpdate & 0x03) {
              case 0:
                // START COLUMN
                state->apsROIPositionX[apsROIRegion] = U16T(state->apsROITmpData | misc8Data);
                break;

              case 1:
                // START ROW
                state->apsROIPositionY[apsROIRegion] = U16T(state->apsROITmpData | misc8Data);
                break;

              case 2:
                // END COLUMN
                state->apsROISizeX[apsROIRegion] = U16T(state->apsROITmpData | misc8Data);
                break;

              case 3:
                // END ROW
                state->apsROISizeY[apsROIRegion] = U16T(state->apsROITmpData | misc8Data);
                break;

              default:
                break;
            }

            // Jump to next type of APS info (col->row, start->end).
            state->apsROIUpdate++;

            break;
          }

          case 3: {
            // APS ADC depth info, use directly as ADC depth.
            // 16 bits is the maximum supported depth for APS.
            // Currently not being used by anything!
            break;
          }

          case 4: {
            // Microphone FIRST RIGHT.
            state->micRight = true;
            state->micCount = 1;
            state->micTmpData = misc8Data;
            break;
          }

          case 5: {
            // Microphone FIRST LEFT.
            state->micRight = false;
            state->micCount = 1;
            state->micTmpData = misc8Data;
            break;
          }

          case 6: {
            // Microphone SECOND.
            if (state->micCount != 1) {
              // Ignore incomplete samples.
              break;
            }

            state->micCount = 2;
            state->micTmpData = U16T(U32T(state->micTmpData << 8) | misc8Data);
            break;
          }

          case 7: {
            // Microphone THIRD.
            if (state->micCount != 2) {
              // Ignore incomplete samples.
              break;
            }

            state->micCount = 0;
            uint32_t micData = U32T(U32T(state->micTmpData << 8) | misc8Data);

            caerSampleEvent micSample = caerSampleEventPacketGetEvent(state->currentSamplePacket,
              state->currentSamplePacketPosition);
            caerSampleEventSetType(micSample, state->micRight);
            caerSampleEventSetSample(micSample, micData);
            caerSampleEventSetTimestamp(micSample, state->currentTimestamp);
            caerSampleEventValidate(micSample, state->currentSamplePacket);
            state->currentSamplePacketPosition++;
            break;
          }

          default:
            caerLog(CAER_LOG_ERROR, handle->info.deviceString,
              "Caught Misc8 event that can't be handled.");
            break;
        }

        break;
      }

      case 7: { // Timestamp wrap
        // Detect big timestamp wrap-around.
        int64_t wrapJump = (TS_WRAP_ADD * data);
        int64_t wrapSum = I64T(state->wrapAdd) + wrapJump;

        if (wrapSum > I64T(INT32_MAX)) {
          // Reset wrapAdd at this point, so we can again
          // start detecting overruns of the 32bit value.
          // We reset not to zero, but to the remaining value after
          // multiple wrap-jumps are taken into account.
          int64_t wrapRemainder = wrapSum - I64T(INT32_MAX) - 1LL;
          state->wrapAdd = I32T(wrapRemainder);

          state->lastTimestamp = 0;
          state->currentTimestamp = state->wrapAdd;

          // Increment TSOverflow counter.
          state->wrapOverflow++;

          caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
            state->currentSpecialPacket, state->currentSpecialPacketPosition);
          caerSpecialEventSetTimestamp(currentSpecialEvent, INT32_MAX);
          caerSpecialEventSetType(currentSpecialEvent, TIMESTAMP_WRAP);
          caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
          state->currentSpecialPacketPosition++;

          // Commit packets to separate before wrap from after cleanly.
          tsBigWrap = true;
        }
        else {
          // Each wrap is 2^15 µs (~32ms), and we have
          // to multiply it with the wrap counter,
          // which is located in the data part of this
          // event.
          state->wrapAdd = I32T(wrapSum);

          state->lastTimestamp = state->currentTimestamp;
          state->currentTimestamp = state->wrapAdd;
          initContainerCommitTimestamp(state);

          // Check monotonicity of timestamps.
          checkStrictMonotonicTimestamp(handle);

          caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
            "Timestamp wrap event received with multiplier of %" PRIu16 ".", data);
        }

        break;
      }

      default:
        caerLog(CAER_LOG_ERROR, handle->info.deviceString, "Caught event that can't be handled.");
        break;
    }
*/

    // Thresholds on which to trigger packet container commit.
    // forceCommit is already defined above.
    // Trigger if any of the global container-wide thresholds are met.
    int32_t currentPacketContainerCommitSize = I32T(
      atomic_load_explicit(&state->maxPacketContainerPacketSize, memory_order_relaxed));
    bool containerSizeCommit = (currentPacketContainerCommitSize > 0)
      && ((state->currentPolarityPacketPosition >= currentPacketContainerCommitSize)
        || (state->currentSpecialPacketPosition >= currentPacketContainerCommitSize)
        || (state->currentFramePacketPosition >= currentPacketContainerCommitSize)
        || (state->currentIMU6PacketPosition >= currentPacketContainerCommitSize)
        || (state->currentSamplePacketPosition >= currentPacketContainerCommitSize));

    bool containerTimeCommit = generateFullTimestamp(state->wrapOverflow, state->currentTimestamp)
      > state->currentPacketContainerCommitTimestamp;

    // Commit packet containers to the ring-buffer, so they can be processed by the
    // main-loop, when any of the required conditions are met.
    if (tsReset || tsBigWrap || containerSizeCommit || containerTimeCommit) {
      // One or more of the commit triggers are hit. Set the packet container up to contain
      // any non-empty packets. Empty packets are not forwarded to save memory.
      bool emptyContainerCommit = true;

      if (state->currentPolarityPacketPosition > 0) {
        caerEventPacketContainerSetEventPacket(state->currentPacketContainer, POLARITY_EVENT,
          (caerEventPacketHeader) state->currentPolarityPacket);

        state->currentPolarityPacket = NULL;
        state->currentPolarityPacketPosition = 0;
        emptyContainerCommit = false;
      }

      if (state->currentSpecialPacketPosition > 0) {
        caerEventPacketContainerSetEventPacket(state->currentPacketContainer, SPECIAL_EVENT,
          (caerEventPacketHeader) state->currentSpecialPacket);

        state->currentSpecialPacket = NULL;
        state->currentSpecialPacketPosition = 0;
        emptyContainerCommit = false;
      }

      if (state->currentFramePacketPosition > 0) {
        caerEventPacketContainerSetEventPacket(state->currentPacketContainer, FRAME_EVENT,
          (caerEventPacketHeader) state->currentFramePacket);

        state->currentFramePacket = NULL;
        state->currentFramePacketPosition = 0;
        emptyContainerCommit = false;
      }

      if (state->currentIMU6PacketPosition > 0) {
        caerEventPacketContainerSetEventPacket(state->currentPacketContainer, IMU6_EVENT,
          (caerEventPacketHeader) state->currentIMU6Packet);

        state->currentIMU6Packet = NULL;
        state->currentIMU6PacketPosition = 0;
        emptyContainerCommit = false;
      }

      if (state->currentSamplePacketPosition > 0) {
        caerEventPacketContainerSetEventPacket(state->currentPacketContainer, DAVIS_SAMPLE_POSITION,
          (caerEventPacketHeader) state->currentSamplePacket);

        state->currentSamplePacket = NULL;
        state->currentSamplePacketPosition = 0;
        emptyContainerCommit = false;
      }

      if (tsReset || tsBigWrap) {
        // Ignore all APS and IMU6 (composite) events, until a new APS or IMU6
        // Start event comes in, for the next packet.
        // This is to correctly support the forced packet commits that a TS reset,
        // or a TS big wrap, impose. Continuing to parse events would result
        // in a corrupted state of the first event in the new packet, as it would
        // be incomplete, incorrect and miss vital initialization data.
        // See APS and IMU6 END states for more details on a related issue.
        state->apsIgnoreEvents = true;
        state->imuIgnoreEvents = true;
      }

      // If the commit was triggered by a packet container limit being reached, we always
      // update the time related limit. The size related one is updated implicitly by size
      // being reset to zero after commit (new packets are empty).
      if (containerTimeCommit) {
        while (generateFullTimestamp(state->wrapOverflow, state->currentTimestamp)
          > state->currentPacketContainerCommitTimestamp) {
          state->currentPacketContainerCommitTimestamp += I32T(
            atomic_load_explicit( &state->maxPacketContainerInterval, memory_order_relaxed));
        }
      }

      // Filter out completely empty commits. This can happen when data is turned off,
      // but the timestamps are still going forward.
      if (emptyContainerCommit) {
        caerEventPacketContainerFree(state->currentPacketContainer);
        state->currentPacketContainer = NULL;
      }
      else {
        if (!ringBufferPut(state->dataExchangeBuffer, state->currentPacketContainer)) {
          // Failed to forward packet container, just drop it, it doesn't contain
          // any critical information anyway.
          caerLog(CAER_LOG_INFO, handle->info.deviceString,
            "Dropped EventPacket Container because ring-buffer full!");

          caerEventPacketContainerFree(state->currentPacketContainer);
          state->currentPacketContainer = NULL;
        }
        else {

          state->currentPacketContainer = NULL;
        }
      }

      // The only critical timestamp information to forward is the timestamp reset event.
      // The timestamp big-wrap can also (and should!) be detected by observing a packet's
      // tsOverflow value, not the special packet TIMESTAMP_WRAP event, which is only informative.
      // For the timestamp reset event (TIMESTAMP_RESET), we thus ensure that it is always
      // committed, and we send it alone, in its own packet container, to ensure it will always
      // be ordered after any other event packets in any processing or output stream.
      if (tsReset) {
        // Allocate packet container just for this event.
        caerEventPacketContainer tsResetContainer = caerEventPacketContainerAllocate(DAVIS_EVENT_TYPES);
        if (tsResetContainer == NULL) {
          caerLog(CAER_LOG_CRITICAL, handle->info.deviceString,
            "Failed to allocate tsReset event packet container.");
          return;
        }

        // Allocate special packet just for this event.
        caerSpecialEventPacket tsResetPacket = caerSpecialEventPacketAllocate(1, I16T(handle->info.deviceID),
          state->wrapOverflow);
        if (tsResetPacket == NULL) {
          caerLog(CAER_LOG_CRITICAL, handle->info.deviceString,
            "Failed to allocate tsReset special event packet.");
          return;
        }

        // Create timestamp reset event.
        caerSpecialEvent tsResetEvent = caerSpecialEventPacketGetEvent(tsResetPacket, 0);
        caerSpecialEventSetTimestamp(tsResetEvent, INT32_MAX);
        caerSpecialEventSetType(tsResetEvent, TIMESTAMP_RESET);
        caerSpecialEventValidate(tsResetEvent, tsResetPacket);

        // Assign special packet to packet container.
        caerEventPacketContainerSetEventPacket(tsResetContainer, SPECIAL_EVENT,
          (caerEventPacketHeader) tsResetPacket);

        // Reset MUST be committed, always, else downstream data processing and
        // outputs get confused if they have no notification of timestamps
        // jumping back go zero.
        while (!ringBufferPut(state->dataExchangeBuffer, tsResetContainer)) {
          // Prevent dead-lock if shutdown is requested and nothing is consuming
          // data anymore, but the ring-buffer is full (and would thus never empty),
          // thus blocking the USB handling thread in this loop.
          if (!atomic_load_explicit(&state->dataAcquisitionThreadRun, memory_order_relaxed)) {
            return;
          }
        }
      }
    }
  }
}

static int playbackDataAcquisitionThread(void *inPtr) {
  // inPtr is a pointer to device handle.
  playbackHandle handle = inPtr;
  playbackState state = &handle->state;

  caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "Initializing data acquisition thread ...");

  // Set thread name.
  thrd_set_name(state->deviceThreadName);

  // Reset configuration update, so as to not re-do work afterwards.
  atomic_store(&state->dataAcquisitionThreadConfigUpdate, 0);

  // Signal data thread ready back to start function.
  atomic_store(&state->dataAcquisitionThreadRun, true);

  caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "data acquisition thread ready to process events.");

  // Handle USB events (1 second timeout).
  struct timeval te = { .tv_sec = 1, .tv_usec = 0 };

  size_t blockSize = 8*50;
  char inBuffer[blockSize];
  size_t readBytes = 0;
  while (atomic_load_explicit(&state->dataAcquisitionThreadRun, memory_order_relaxed)) {

    //libusb_handle_events_timeout(state->usbState.deviceContext, &te);
    readBytes = fread(inBuffer, sizeof(char), blockSize, state->file);
    playbackDavisEventTranslator(handle,inBuffer, readBytes);

    if(readBytes < blockSize)
      break;
  }

  caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "shutting down data acquisition thread ...");

  // Cancel all transfers and handle them.
  //usbDeallocateTransfers(&state->usbState);

  if(state->playbackFinishedCallback != NULL){
    state->playbackFinishedCallback(state->callbackParam);
  }
  // Ensure shutdown is stored and notified, could be because of all data transfers going away!
  atomic_store(&state->dataAcquisitionThreadRun, false);

  caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "data acquisition thread shut down.");

  return (EXIT_SUCCESS);
}
