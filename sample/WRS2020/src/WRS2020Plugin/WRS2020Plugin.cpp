/**
   @author Kenta Suzuki
*/

#include <cnoid/Format>
#include <cnoid/Plugin>
#include <cnoid/WRSUtilBar>

using namespace cnoid;

class WRS2020Plugin : public Plugin
{
public:
    WRS2020Plugin()
        : Plugin("WRS2020")
    {
        require("Body");
        require("WRSUtil");
    }

    virtual bool initialize() override
    {
        WRSUtilBar::instance()->addFormat({ "WRS2020", 20.0 });
        return true;
    }

    virtual const char* description() const override
    {
        static std::string text =
            formatC("WRS Plugin Version {}\n", CNOID_FULL_VERSION_STRING) +
            "\n" +
            "Copyright (c) 2025 WRS Simulation Disaster Challenge Competition Committee.\n"
            "\n" +
            MITLicenseText();
        return text.c_str();
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(WRS2020Plugin)