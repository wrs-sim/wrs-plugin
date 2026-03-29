/**
    @author Kenta Suzuki
*/

#ifndef CNOID_LIVE_PLAYBACK_PLUGIN_LIVE_PLAYBACK_BAR_H
#define CNOID_LIVE_PLAYBACK_PLUGIN_LIVE_PLAYBACK_BAR_H

#include <cnoid/ToolBar>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;

class CNOID_EXPORT LivePlaybackBar : public ToolBar
{
public:
    static void initialize(ExtensionManager* ext);
    static LivePlaybackBar* instance();

private:
    class Impl;
    Impl* impl;

    LivePlaybackBar();
    ~LivePlaybackBar();
};

} // namespace cnoid

#endif