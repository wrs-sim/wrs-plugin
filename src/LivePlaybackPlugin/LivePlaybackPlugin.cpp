/**
    @author Kenta Suzuki
*/

#include <cnoid/Format>
#include <cnoid/Plugin>
#include "LivePlaybackBar.h"

using namespace cnoid;

class LivePlaybackPlugin : public Plugin
{
public:
    LivePlaybackPlugin()
        : Plugin("LivePlayback")
    {
        require("Body");
    }

    virtual bool initialize() override
    {
        LivePlaybackBar::initialize(this);
        return true;
    }

    virtual const char* description() const override
    {
        static std::string text
            = formatC("WRS2025 Plugin Version {}\n", CNOID_FULL_VERSION_STRING)
              + "\n"
              + "Copyright (c) 2025 WRS Simulation Disaster Challenge Competition Committee.\n"
                "\n"
              + MITLicenseText();
        return text.c_str();
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(LivePlaybackPlugin)