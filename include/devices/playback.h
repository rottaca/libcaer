#ifndef LIBCAER_HDR_PLAYBACK_H_
#define LIBCAER_HDR_PLAYBACK_H_

#ifdef __cplusplus
extern "C" {
#endif
    #include "davis.h"

    typedef struct playback_state *playbackState;
    typedef struct playback_handle *playbackHandle;
    struct playback_info {
        int sx;
        int sy;
    };
    typedef  struct playback_info* playbackInfo;

    playbackHandle playbackOpen( const char *fileName, void (*playbackFinishedCallback) (void*), void* param);
    void playbackChangeSpeed(playbackHandle handle, float speed);
    int playbackClose(playbackHandle handle);

    int playbackDataStart(playbackHandle handle);
    int playbackDataStop(playbackHandle handle);
    caerEventPacketContainer playbackDataGet(playbackHandle handle);

    playbackInfo caerPlaybackInfoGet(playbackHandle handle);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_HDR_PLAYBACK_H_ */
