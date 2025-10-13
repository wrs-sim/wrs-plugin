/**
    @author Kenta Suzuki
*/

#include "LivePlaybackBar.h"
#include <cnoid/ExtensionManager>
#include <cnoid/ItemList>
#include <cnoid/MainWindow>
#include <cnoid/OptionManager>
#include <cnoid/RootItem>
#include <cnoid/TimeBar>
#include <cnoid/ToolButton>
#include <cnoid/WorldLogFileItem>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class LivePlaybackBar::Impl
{
public:
    LivePlaybackBar* self;
    ToolButton* playbackButton;
    ToolButton* saveButton;

    Impl(LivePlaybackBar* self);
    ~Impl();
    void onPlaybackButtonClicked();
    void onSaveButtonClicked();
    void onSelectedWorldLogFileItemsChanged(ItemList<WorldLogFileItem> selectedItems);
    void onOptionParsed(OptionManager* optionManager);
};

} // namespace cnoid

void LivePlaybackBar::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized) {
        ext->addToolBar(instance());
        initialized = true;
    }
}

LivePlaybackBar* LivePlaybackBar::instance()
{
    static LivePlaybackBar* livePlaybackBar = new LivePlaybackBar;
    return livePlaybackBar;
}

LivePlaybackBar::LivePlaybackBar()
    : ToolBar(N_("LivePlaybackBar"))
{
    impl = new Impl(this);
}

LivePlaybackBar::Impl::Impl(LivePlaybackBar* self)
    : self(self)
{
    self->setVisibleByDefault(false);

    auto om = OptionManager::instance();
    om->add_flag("--live-playback", "start live playback automatically");
    om->sigOptionsParsed(1).connect([&](OptionManager* optionManager) { onOptionParsed(optionManager); });

    const QIcon playbackIcon = QIcon::fromTheme("media-playback-start");
    playbackButton = self->addButton(playbackIcon);
    playbackButton->setToolTip(_("Start Live Playback"));
    playbackButton->sigClicked().connect([&]() { onPlaybackButtonClicked(); });

    self->addSeparator();

    const QIcon saveIcon = QIcon::fromTheme("document-save-as");
    saveButton = self->addButton(saveIcon);
    saveButton->setToolTip(_("Save project as log playback archive"));
    saveButton->sigClicked().connect([&]() { onSaveButtonClicked(); });

    RootItem::instance()->sigSelectedItemsChanged().connect(
        [&](const ItemList<>& selectedItems) { onSelectedWorldLogFileItemsChanged(selectedItems); });
}

LivePlaybackBar::~LivePlaybackBar()
{
    delete impl;
}

LivePlaybackBar::Impl::~Impl()
{
}

void LivePlaybackBar::Impl::onPlaybackButtonClicked()
{
    auto timeBar = TimeBar::instance();
    const QIcon playbackIcon = QIcon::fromTheme("media-playback-start");
    const QIcon resumeIcon = QIcon::fromTheme("media-playback-pause");

    WorldLogFileItem* fileItem = nullptr;
    auto fileItems = RootItem::instance()->selectedItems<WorldLogFileItem>();
    for(auto& item : fileItems) {
        fileItem = item;
        break;
    }

    if(!fileItem) {
        auto items = RootItem::instance()->descendantItems<WorldLogFileItem>();
        for(auto& item : items) {
            fileItem = item;
            item->setSelected(true);
            break;
        }
    }

    if(fileItem) {
        if(timeBar->isDoingPlayback()) {
            playbackButton->setIcon(playbackIcon);
            fileItem->stopLivePlayback();
        } else {
            playbackButton->setIcon(resumeIcon);
            fileItem->setLivePlaybackReadInterval(0);
            fileItem->setLivePlaybackReadTimeout(5000.0);
            fileItem->stopLivePlayback();
            fileItem->startLivePlayback();
        }
    }
}

void LivePlaybackBar::Impl::onSaveButtonClicked()
{
    auto fileItems = RootItem::instance()->selectedItems<WorldLogFileItem>();
    for(auto& fileItem : fileItems) {
        fileItem->showPlaybackArchiveSaveDialog();
        break;
    }
}

void LivePlaybackBar::Impl::onSelectedWorldLogFileItemsChanged(ItemList<WorldLogFileItem> selectedItems)
{
    const QIcon playbackIcon = QIcon::fromTheme("media-playback-start");
    playbackButton->setIcon(playbackIcon);
}

void LivePlaybackBar::Impl::onOptionParsed(OptionManager* optionManager)
{
    if(optionManager->count("--live-playback")) {
        onPlaybackButtonClicked();
    }
}