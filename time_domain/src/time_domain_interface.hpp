#ifndef TIMEDOMAININTERFACE_H
#define TIMEDOMAININTERFACE_H

#include "basic_functions.hpp"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/package.h>

#include "lib/rcm.h"
#include "lib/rcmIf.h"
#include "lib/rn.h"
#include "lib/helper.h"

#define StatusSuccess 0
#define StatusTimeoutFailure 1
#define StatusVirtualCarrierSense 3

namespace uavos{

struct UWB_Options{
    int node_id;
    std::string operation_mode;
    std::string connection_interface;
    std::string ip_address;

    int integration_index;
    int antenna_mode;
    int code_channel;
    bool echo_last_range;
    int tx_gain;



    std::string network_type;
    std::string autosend_range_type;
    std::string autosend_NDB_type;
    int autosend_NDB_update_interval_ms;
    int max_neighbor_age_ms;
    bool do_not_range_me;

    int max_request_data_size;
    int max_response_data_size;


    bool ALOHA_ACC;
    int ALOHA_min_tx_ms;
    int ALOHA_max_tx_ms;
    bool ALOHA_is_beacon;


    std::string TDMA_slotmap_file;
};

inline bool operator==(const rnConfiguration& src, const rnConfiguration& dst){
    bool is_equal=true;

    if(src.maxNeighborAgeMs!=dst.maxNeighborAgeMs ||
            src.autosendNeighborDbUpdateIntervalMs!=dst.autosendNeighborDbUpdateIntervalMs ||
            src.rnFlags!= dst.rnFlags ||
            src.networkSyncMode!=dst.networkSyncMode ||
            src.autosendType!=dst.autosendType ||
            //src.reserved!=dst.reserved ||
            src.defaultIf!=dst.defaultIf ||
            src.defaultIfAddr1!=dst.defaultIfAddr1 ||
            src.defaultIfAddr2!=dst.defaultIfAddr2){
        is_equal = false;
    }

    return is_equal;
}
inline bool operator!=(const rnConfiguration& src, const rnConfiguration& dst){
    return !(src==dst);
}

inline bool operator==(const rcmConfiguration& src, const rcmConfiguration& dst){
    bool is_equal=true;

    if(src.nodeId!=dst.nodeId ||
            src.integrationIndex!=dst.integrationIndex ||
            src.antennaMode!=dst.antennaMode ||
            src.codeChannel!=dst.codeChannel ||
            src.electricalDelayPsA!=dst.electricalDelayPsA ||
            src.electricalDelayPsB!=dst.electricalDelayPsB ||
            src.flags!=dst.flags ||
            src.txGain!=dst.txGain ||
            src.persistFlag!=dst.persistFlag){
        is_equal = false;
    }

    return is_equal;
}
inline bool operator!=(const rcmConfiguration& src, const rcmConfiguration& dst){
    return !(src==dst);
}

inline bool operator==(const rnALOHAConfiguration& src, const rnALOHAConfiguration& dst){
    bool is_equal=true;

    if(src.minTimeBetweenTxMs!=dst.minTimeBetweenTxMs ||
            src.maxTimeBetweenTxMs!=dst.maxTimeBetweenTxMs ||
            src.maxRequestDataSize!=dst.maxRequestDataSize ||
            src.maxResponseDataSize!=dst.maxResponseDataSize ||
            src.flags!=dst.flags ||
            src.accKfactor!=dst.accKfactor
            //src.reserved!=dst.reserved
            ){
        is_equal = false;
    }

    return is_equal;
}
inline bool operator!=(const rnALOHAConfiguration& src, const rnALOHAConfiguration& dst){
    return !(src==dst);
}

inline bool operator==(const rnTDMAConfiguration& src, const rnTDMAConfiguration& dst){
    bool is_equal=true;

    if(src.maxRequestDataSize!=dst.maxRequestDataSize ||
            src.maxResponseDataSize!=dst.maxResponseDataSize
            //src.reserved1!= dst.reserved1 ||
            //src.reserved2!=dst.reserved2
            ){
        is_equal = false;
    }

    return is_equal;
}
inline bool operator!=(const rnTDMAConfiguration& src, const rnTDMAConfiguration& dst){
    return !(src==dst);
}

inline bool operator==(const rnTDMASlotDefinition& src, const rnTDMASlotDefinition& dst){
    bool is_equal = true;

    if(src.slotType!=dst.slotType ||
            src.slotNumber!=dst.slotNumber ||
            src.flags!=dst.flags ||
            src.integrationIndex!=dst.integrationIndex ||
            src.antennaMode != dst.antennaMode ||
            src.codeChannel != dst.codeChannel ||
            //src.reserved1 != dst.reserved1 ||
            src.requesterId != dst.requesterId ||
            src.responderId != dst.responderId ||
            src.requestedDurationMicroseconds != dst.requestedDurationMicroseconds){
        is_equal = false;
    }

    return is_equal;
}
inline bool operator!=(const rnTDMASlotDefinition& src, const rnTDMASlotDefinition& dst){
    return !(src==dst);
}

inline bool operator==(const rnMsg_GetTDMASlotmapConfirm& confirm, const rnMsg_SetTDMASlotmapRequest& request){
    bool is_equal = true;

    if(confirm.numSlots!=request.numSlots){
        is_equal = false;
    } else {
        for(int i=0;i<confirm.numSlots;++i){
            if(confirm.slots[i].slot!=request.slots[i]){
                is_equal = false;
                break;
            }
        }
    }

    return is_equal;
}
inline bool operator!=(const rnMsg_GetTDMASlotmapConfirm& confirm, const rnMsg_SetTDMASlotmapRequest& request){
    return !(confirm==request);
}

rnMsg_SetTDMASlotmapRequest readSlotmap(const std::string path);




// ---------------------------------------------------
class TimeDomainInterface
{
public:
    explicit TimeDomainInterface(ros::NodeHandle nh);
    ~TimeDomainInterface(){
        ROS_WARN("Deconstructing TimeDomainInterface instance.");
//        while(!restoreConfig()){
//            ROS_ERROR("Retry reset config.");
//        }
        // perform cleanup
        rcmIfClose();
    }


private:
    ros::NodeHandle m_nh;
    uavos::UWB_Options m_options;
    bool m_is_rewrite_RN_config;

    ros::Rate m_retry_rate;

    // Configuration structures
    rcmConfiguration m_rcmInitConfig, m_rcmConfig;
    rnConfiguration m_rnInitConfig, m_rnConfig;

    // ALOHA
    rnALOHAConfiguration m_rnInitAlohaConfig, m_rnAlohaConfig;

    // TDMA
    rnTDMAConfiguration m_rnInitTdmaConfig, m_rnTdmaConfig;
    rnMsg_GetTDMASlotmapConfirm m_rnInitTdmaSlotMap;
    rnMsg_SetTDMASlotmapRequest m_rnTdmaSlotMap;

    // ros topic
    ros::Publisher m_full_range_info_pub;
    ros::Publisher m_error_range_info_pub;
    ros::Publisher m_NDB_pub;
    ros::Publisher m_data_info_pub;
    ros::Publisher m_echoed_range_pub;

    ros::Subscriber m_send_data_sub;

    // recover from failure
    ros::Time m_last_read_ok_time;

    // mutex and sem_t to ensure data and range doesn't conflict, under multi thread spinner
    boost::mutex m_read_msg_mutex;
    int m_semaphore_counter;

public:
    bool m_is_uwb_ready;

private:
    void loadParameters();
    int preInit();
    int RCMInit();
    int RNInit();
    int TDMAInit();
    int TDMASlotmapInit();

    int ALOHAInit();

    common_msgs::UWB_FullRangeInfo rcmRange2ros(int node_id, rcmMsg_FullRangeInfo rangeInfo);
    common_msgs::UWB_FullNeighborDatabaseEntry rnNDBEntry2ros(rnFullNeighborDatabaseEntry_t entry);
    common_msgs::UWB_FullNeighborDatabase rnNDB2ros(int node_id, rnMsg_GetFullNeighborDatabaseConfirm ndb);
    common_msgs::UWB_DataInfo rcmData2ros(int node_id, rcmMsg_DataInfo dataInfo);
    common_msgs::UWB_EchoedRangeInfo rcmEcho2ros(rcmMsg_EchoedRangeInfo echoedInfo);

public:
    template<typename T>
    int loopWrapper(T(TimeDomainInterface::*f)(), TimeDomainInterface* object){
        while(1!=(object->*f)() && ros::ok()){
            ros::Duration(0.3).sleep();
        }
        
        // wait, althought I don't know whether this is necessary
        ros::Duration(0.001).sleep();
    }

    int setupUWB();

    bool activateRadio();
    bool activateRadioAndConfigOpmodeRn();
    bool restoreConfig();

    bool readAMessage();
    void readMsgCallback(const ros::TimerEvent &event);

    void sendDataToNetworkCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);


    // recover from failure
    void watchDogCallback(const ros::TimerEvent& event);


};

}

#endif // TIMEDOMAININTERFACE_H
