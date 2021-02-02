#include "gpio_presence.hpp"

#include "xyz/openbmc_project/Common/error.hpp"

#include <fcntl.h>
#include <libevdev/libevdev.h>

#include <fstream>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>

namespace phosphor
{
namespace gpio
{
namespace presence
{

using namespace phosphor::logging;
using namespace sdbusplus::xyz::openbmc_project::Common::Error;

constexpr auto INVENTORY_PATH = "/xyz/openbmc_project/inventory";
constexpr auto INVENTORY_INTF = "xyz.openbmc_project.Inventory.Manager";

constexpr auto MAPPER_BUSNAME = "xyz.openbmc_project.ObjectMapper";
constexpr auto MAPPER_PATH = "/xyz/openbmc_project/object_mapper";
constexpr auto MAPPER_INTERFACE = "xyz.openbmc_project.ObjectMapper";

constexpr char SYSTEMD_BUSNAME[] = "org.freedesktop.systemd1";
constexpr char SYSTEMD_PATH[] = "/org/freedesktop/systemd1";
constexpr char SYSTEMD_INTERFACE[] = "org.freedesktop.systemd1.Manager";
constexpr char OBMCREADEEPROM_UNITNAME[] = "obmc-read-eeprom";

std::string getService(const std::string& path, const std::string& interface,
                       sdbusplus::bus::bus& bus)
{
    auto mapperCall = bus.new_method_call(MAPPER_BUSNAME, MAPPER_PATH,
                                          MAPPER_INTERFACE, "GetObject");

    mapperCall.append(path);
    mapperCall.append(std::vector<std::string>({interface}));

    auto mapperResponseMsg = bus.call(mapperCall);
    if (mapperResponseMsg.is_method_error())
    {
        log<level::ERR>("Error in mapper call to get service name",
                        entry("PATH=%s", path.c_str()),
                        entry("INTERFACE=%s", interface.c_str()));
        elog<InternalFailure>();
    }

    std::map<std::string, std::vector<std::string>> mapperResponse;
    mapperResponseMsg.read(mapperResponse);

    if (mapperResponse.empty())
    {
        log<level::ERR>("Error in mapper response for getting service name",
                        entry("PATH=%s", path.c_str()),
                        entry("INTERFACE=%s", interface.c_str()));
        elog<InternalFailure>();
    }

    return mapperResponse.begin()->first;
}

void Presence::determinePresence()
{
    auto present = false;
    auto value = static_cast<int>(0);
    auto fetch_rc =
        libevdev_fetch_event_value(devicePtr.get(), EV_KEY, key, &value);
    if (0 == fetch_rc)
    {
        log<level::ERR>("Device does not support event type",
                        entry("KEYCODE=%d", key));
        elog<InternalFailure>();
        return;
    }
    if (value > 0)
    {
        present = true;
    }
    iPresence = present;
    updateInventory(present);
}

// Callback handler when there is an activity on the FD
int Presence::processEvents(sd_event_source*, int, uint32_t, void* userData)
{
    auto presence = static_cast<Presence*>(userData);

    presence->analyzeEvent();
    return 0;
}

// Analyzes the GPIO event
void Presence::analyzeEvent()
{

    // Data returned
    struct input_event ev
    {
    };
    int rc = 0;

    // While testing, observed that not having a loop here was leading
    // into events being missed.
    while (rc >= 0)
    {
        // Wait until no more events are available on the device.
        rc = libevdev_next_event(devicePtr.get(), LIBEVDEV_READ_FLAG_NORMAL,
                                 &ev);
        if (rc < 0)
        {
            // There was an error waiting for events, mostly that there are no
            // events to be read.. So continue waiting...
            return;
        }

        if (rc == LIBEVDEV_READ_STATUS_SUCCESS)
        {
            if (ev.type == EV_SYN && ev.code == SYN_REPORT)
            {
                continue;
            }
            else if (ev.code == key)
            {
                auto present = false;
                if (ev.value > 0)
                {
                    present = true;
                }
                if(iPresence != present){
                    updateInventory(present);
                    bindOrUnbindDrivers(present);
                    iPresence = present;
                }
            }
        }
    }

    return;
}

Presence::ObjectMap Presence::getObjectMap(bool present)
{
    ObjectMap invObj;
    InterfaceMap invIntf;
    PropertyMap invProp;

    invProp.emplace("Present", present);
    invProp.emplace("PrettyName", name);
    invIntf.emplace("xyz.openbmc_project.Inventory.Item", std::move(invProp));
    // Add any extra interfaces we want to associate with the inventory item
    for (auto& iface : ifaces)
    {
        invIntf.emplace(iface, PropertyMap());
    }
    invObj.emplace(std::move(inventory), std::move(invIntf));

    return invObj;
}

void Presence::readEEPROM(const std::string& unit)
{
    std::string unit_name;// = unit.substr(1,unit.length()-1);
    unit_name = OBMCREADEEPROM_UNITNAME;
    unit_name += "@";
    unit_name += unit.substr(1,unit.length()-1);
    std::replace( unit_name.begin(), unit_name.end(), '/', '-');
    unit_name += ".service";
    try
    {
        auto method = bus.new_method_call(SYSTEMD_BUSNAME, SYSTEMD_PATH,
                                          SYSTEMD_INTERFACE, "RestartUnit");
        method.append(unit_name.c_str(), "replace");
        bus.call_noreply(method);
    }
    catch (const sdbusplus::exception::SdBusError& ex)
    {
        log<level::ERR>("Failed to restart service", entry("ERR=%s", ex.what()),
                        entry("UNIT=%s", unit_name.c_str()));
        elog<InternalFailure>();
    }
}

void Presence::updateInventory(bool present)
{
    ObjectMap invObj = getObjectMap(present);

    log<level::DEBUG>("Updating inventory present property",
                     entry("PRESENT=%d", present),
                     entry("PATH=%s", inventory.c_str()));

    auto invService = getService(INVENTORY_PATH, INVENTORY_INTF, bus);

    // Update inventory
    auto invMsg = bus.new_method_call(invService.c_str(), INVENTORY_PATH,
                                      INVENTORY_INTF, "Notify");
    invMsg.append(std::move(invObj));
    auto invMgrResponseMsg = bus.call(invMsg);
    if (invMgrResponseMsg.is_method_error())
    {
        log<level::ERR>("Error in inventory manager call to update inventory");
        elog<InternalFailure>();
    }
    readEEPROM(inventory.c_str());
}

void Presence::bindOrUnbindDrivers(bool present)
{
    auto action = (present) ? "bind" : "unbind";

    for (auto& driver : drivers)
    {
        auto path = std::get<pathField>(driver) / action;
        auto device = std::get<deviceField>(driver);

        if (present)
        {
            log<level::INFO>("Binding a device driver",
                             entry("PATH=%s", path.c_str()),
                             entry("DEVICE=%s", device.c_str()));
        }
        else
        {
            log<level::INFO>("Unbinding a device driver",
                             entry("PATH=%s", path.c_str()),
                             entry("DEVICE=%s", device.c_str()));
        }

        std::ofstream file;

        file.exceptions(std::ofstream::failbit | std::ofstream::badbit |
                        std::ofstream::eofbit);
        int i,iMax=3;
        for(i=0;i<iMax;i++){
            try
            {
                file.open(path);
                file << device;
                file.close();
                i=iMax;
            }
            catch (std::exception& e)
            {
                auto err = errno;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                log<level::ERR>("Failed binding or unbinding a device "
                                "after a card was removed or added",
                                entry("PATH=%s", path.c_str()),
                                entry("DEVICE=%s", device.c_str()),
                                entry("Retry=%d", i),
                                entry("ERRNO=%d", err));
            }
        }
    }
}

} // namespace presence
} // namespace gpio
} // namespace phosphor
