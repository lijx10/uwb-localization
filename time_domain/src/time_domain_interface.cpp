#include "time_domain_interface.hpp"

rnMsg_SetTDMASlotmapRequest uavos::readSlotmap(const std::string filename){
    std::vector<std::vector<int> > slotmap_matrix;
    int num_of_slot = uavos::readFromTxt<int>(slotmap_matrix, filename, std::string(","));
    ROS_INFO("Number of TDMA slots: %d", num_of_slot);

    rnMsg_SetTDMASlotmapRequest slotmap_request;
    slotmap_request.numSlots = num_of_slot;
    for(int i=0;i<num_of_slot;++i){
        int slotType = slotmap_matrix.at(i).at(0);
        int slotNumber = slotmap_matrix.at(i).at(1);
        int flags = slotmap_matrix.at(i).at(2);
        int integrationIndex = slotmap_matrix.at(i).at(3);
        int antennaMode = slotmap_matrix.at(i).at(4);
        int codeChannel = slotmap_matrix.at(i).at(5);
        int requesterId = slotmap_matrix.at(i).at(7);
        int responderId = slotmap_matrix.at(i).at(8);
        int requestedDurationMicroseconds = slotmap_matrix.at(i).at(9);

        slotmap_request.slots[i].slotType = slotType;
        slotmap_request.slots[i].slotNumber = slotNumber;
        slotmap_request.slots[i].flags = flags;
        slotmap_request.slots[i].integrationIndex = integrationIndex;
        slotmap_request.slots[i].antennaMode = antennaMode;
        slotmap_request.slots[i].codeChannel = codeChannel;
        slotmap_request.slots[i].requesterId = requesterId;
        slotmap_request.slots[i].responderId = responderId;
        slotmap_request.slots[i].requestedDurationMicroseconds = requestedDurationMicroseconds;
    }

    return slotmap_request;
}



// class functions
uavos::TimeDomainInterface::TimeDomainInterface(ros::NodeHandle nh):
    m_nh(nh), m_retry_rate(5)
{

    loadParameters();

    m_is_uwb_ready = false;

    // init ros topic
    m_full_range_info_pub = m_nh.advertise<common_msgs::UWB_FullRangeInfo>("full_range_info", 10);
    m_error_range_info_pub = m_nh.advertise<common_msgs::UWB_FullRangeInfo>("error_range_info", 10);
    m_NDB_pub = m_nh.advertise<common_msgs::UWB_FullNeighborDatabase>("NDB", 10);
    m_data_info_pub = m_nh.advertise<common_msgs::UWB_DataInfo>("data_info", 10);
    m_echoed_range_pub = m_nh.advertise<common_msgs::UWB_EchoedRangeInfo>("echoed_range", 10);

    m_send_data_sub = m_nh.subscribe("send_data_to_network", 10, &uavos::TimeDomainInterface::sendDataToNetworkCallback, this);

    // mutex
    m_semaphore_counter = 0;
}

void uavos::TimeDomainInterface::watchDogCallback(const ros::TimerEvent &event){
    // to check whether the connection break down.
    if(true==m_is_uwb_ready && ros::Time::now().toSec() - m_last_read_ok_time.toSec()>3){
        m_is_uwb_ready = false;
        ROS_ERROR("No message received in 3 seconds. Trying to soft reboot UWB!");

        setupUWB();
    }
}

// -------------------------- rcm/rn msg to ros msg -------------------------

common_msgs::UWB_FullRangeInfo uavos::TimeDomainInterface::rcmRange2ros(int node_id, rcmMsg_FullRangeInfo rangeInfo){
    common_msgs::UWB_FullRangeInfo ros_range_info;

    ros_range_info.header.stamp = ros::Time::now();

    ros_range_info.nodeId = node_id;

    ros_range_info.msgType = rangeInfo.msgType;
    ros_range_info.msgId = rangeInfo.msgId;

    ros_range_info.responderId = rangeInfo.responderId;

    ros_range_info.rangeStatus = rangeInfo.rangeStatus;
    ros_range_info.antennaMode = rangeInfo.antennaMode;
    ros_range_info.stopwatchTime = rangeInfo.stopwatchTime;

    ros_range_info.precisionRangeMm = rangeInfo.precisionRangeMm;
    ros_range_info.coarseRangeMm = rangeInfo.coarseRangeMm;
    ros_range_info.filteredRangeMm = rangeInfo.filteredRangeMm;

    ros_range_info.precisionRangeErrEst = rangeInfo.precisionRangeErrEst;
    ros_range_info.coarseRangeErrEst = rangeInfo.coarseRangeErrEst;
    ros_range_info.filteredRangeErrEst = rangeInfo.filteredRangeErrEst;

    ros_range_info.filteredRangeVel = rangeInfo.filteredRangeVel;
    ros_range_info.filteredRangeVelErrEst = rangeInfo.filteredRangeVelErrEst;

    ros_range_info.rangeMeasurementType = rangeInfo.rangeMeasurementType;
    ros_range_info.reserved = rangeInfo.reserved;

    ros_range_info.reqLEDFlags = rangeInfo.reqLEDFlags;
    ros_range_info.respLEDFlags = rangeInfo.respLEDFlags;

    ros_range_info.noise = rangeInfo.noise;
    ros_range_info.vPeak = rangeInfo.vPeak;

    ros_range_info.timestamp = rangeInfo.timestamp;

    return ros_range_info;
}

common_msgs::UWB_FullNeighborDatabaseEntry uavos::TimeDomainInterface::rnNDBEntry2ros(rnFullNeighborDatabaseEntry_t entry){
    common_msgs::UWB_FullNeighborDatabaseEntry entry_ros;

    entry_ros.header.stamp = ros::Time::now();

    entry_ros.nodeId = entry.nodeId;
    entry_ros.rangeStatus = entry.rangeStatus;
    entry_ros.antennaMode = entry.antennaMode;
    entry_ros.stopwatchTime = entry.stopwatchTime;
    entry_ros.rangeMm = entry.rangeMm;
    entry_ros.rangeErrorEstimate = entry.rangeErrorEstimate;
    entry_ros.rangeVelocity = entry.rangeVelocity;
    entry_ros.rangeMeasurementType = entry.rangeMeasurementType;
    entry_ros.flags = entry.flags;
    entry_ros.ledFlags = entry.ledFlags;
    entry_ros.noise = entry.noise;
    entry_ros.vPeak = entry.vPeak;
    entry_ros.statsNumRangeAttempts = entry.statsNumRangeAttempts;
    entry_ros.statsNumRangeSuccesses = entry.statsNumRangeSuccesses;
    entry_ros.statsAgeMs = entry.statsAgeMs;
    entry_ros.rangeUpdateTimestampMs = entry.rangeUpdateTimestampMs;
    entry_ros.lastHeardTimestampMs = entry.lastHeardTimestampMs;
    entry_ros.addedToNDBTimestampMs = entry.addedToNDBTimestampMs;

    return entry_ros;
}

common_msgs::UWB_FullNeighborDatabase uavos::TimeDomainInterface::rnNDB2ros(int node_id, rnMsg_GetFullNeighborDatabaseConfirm ndb){
    common_msgs::UWB_FullNeighborDatabase ndb_ros;
    ndb_ros.header.stamp = ros::Time::now();

    ndb_ros.nodeId = node_id;

    ndb_ros.msgType = ndb.msgType;
    ndb_ros.msgId = ndb.msgId;
    ndb_ros.numNeighborEntries = ndb.numNeighborEntries;
    ndb_ros.sortType = ndb.sortType;
    ndb_ros.reserved = ndb.reserved;
    ndb_ros.timestamp = ndb.timestamp;
    ndb_ros.status = ndb.status;
    for(int i=0;i<ndb.numNeighborEntries;++i){
        ndb_ros.neighbors.push_back(rnNDBEntry2ros(ndb.neighbors[i]));
    }

    return ndb_ros;
}

common_msgs::UWB_DataInfo uavos::TimeDomainInterface::rcmData2ros(int node_id, rcmMsg_DataInfo dataInfo){
    common_msgs::UWB_DataInfo dataInfo_ros;
    dataInfo_ros.header.stamp = ros::Time::now();

    dataInfo_ros.nodeId = node_id;

    dataInfo_ros.msgType = dataInfo.msgType;
    dataInfo_ros.msgId = dataInfo.msgId;
    dataInfo_ros.sourceId = dataInfo.sourceId;
    dataInfo_ros.noise = dataInfo.noise;
    dataInfo_ros.vPeak = dataInfo.vPeak;
    dataInfo_ros.timestamp = dataInfo.timestamp;
    dataInfo_ros.antennaId = dataInfo.antennaId;
    dataInfo_ros.reserved = dataInfo.reserved;
    dataInfo_ros.dataSize = dataInfo.dataSize;
    for(int i=0;i<dataInfo.dataSize;++i){
        dataInfo_ros.data.push_back(dataInfo.data[i]);
    }

    return dataInfo_ros;
}

common_msgs::UWB_EchoedRangeInfo uavos::TimeDomainInterface::rcmEcho2ros(rcmMsg_EchoedRangeInfo echoedInfo){
    common_msgs::UWB_EchoedRangeInfo echoedInfo_ros;
    echoedInfo_ros.header.stamp = ros::Time::now();

    echoedInfo_ros.msgType = echoedInfo.msgType;
    echoedInfo_ros.msgId = echoedInfo.msgId;

    echoedInfo_ros.requesterId = echoedInfo.requesterId;
    echoedInfo_ros.responderId = echoedInfo.responderId;

    echoedInfo_ros.precisionRangeMm = echoedInfo.precisionRangeMm;
    echoedInfo_ros.precisionRangeErrEst = echoedInfo.precisionRangeErrEst;

    echoedInfo_ros.ledFlags = echoedInfo.ledFlags;
    echoedInfo_ros.timestamp = echoedInfo.timestamp;

    return echoedInfo_ros;
}

// -------------------------- rcm/rn msg to ros msg -------------------------




// -------------------------- general initialization --------------------------
void uavos::TimeDomainInterface::loadParameters(){
    // get node_id / uav_id
    if(m_nh.getParam(std::string("/uav_id"), m_options.node_id)){
        ROS_INFO("Node id (/uav_id): %d", m_options.node_id);
    } else if(m_nh.getParam(std::string("node_id"), m_options.node_id)){
        ROS_INFO("Node id (yaml): %d", m_options.node_id);
    } else {
        ROS_ERROR("Node id not found!");
        m_options.node_id = 0;
        assert(m_options.node_id!=0);
    }

    m_nh.param<std::string>("operation_mode", m_options.operation_mode, "RN");

    m_nh.param<std::string>("connection_interface", m_options.connection_interface, std::string("USB"));
    m_nh.param<std::string>("ip_address", m_options.ip_address, std::string("192.168.1.100"));

    m_nh.param<int>("integration_index", m_options.integration_index, 5);
    std::string antenna_mode_str;
    m_nh.param<std::string>("antenna_mode", antenna_mode_str, "A");
    if(std::string("A")==antenna_mode_str){
        m_options.antenna_mode = 0;
    } else {
        m_options.antenna_mode = 0;
    }
    m_nh.param<int>("code_channel", m_options.code_channel, 0);
    m_nh.param<bool>("echo_last_range", m_options.echo_last_range, true);
    m_nh.param<int>("tx_gain", m_options.tx_gain, 63);


    m_nh.param<std::string>("network_type", m_options.network_type, "ALOHA");
    m_nh.param<std::string>("autosend_range_type", m_options.autosend_range_type, "successful");
    m_nh.param<std::string>("autosend_NDB_type", m_options.autosend_NDB_type, "full");
    m_nh.param<int>("autosend_NDB_update_interval_ms", m_options.autosend_NDB_update_interval_ms, 2000);
    m_nh.param<int>("max_neighbor_age_ms", m_options.max_neighbor_age_ms, 999999);
    m_nh.param<bool>("do_not_range_me", m_options.do_not_range_me, false);

    m_nh.param<int>("max_request_data_size", m_options.max_request_data_size, 100);
    m_nh.param<int>("max_response_data_size", m_options.max_response_data_size, 100);


    m_nh.param<bool>("ALOHA_ACC", m_options.ALOHA_ACC, true);
    m_nh.param<int>("ALOHA_min_tx_ms", m_options.ALOHA_min_tx_ms, 100);
    m_nh.param<int>("ALOHA_max_tx_ms", m_options.ALOHA_max_tx_ms, 100);
    m_nh.param<bool>("ALOHA_is_beacon", m_options.ALOHA_is_beacon, false);

    std::string relative_filename;
    m_nh.param<std::string>("TDMA_slotmap_file_folder", relative_filename, "/scripts/txt");
    relative_filename += std::string("/")+uavos::num2str<int>(m_options.node_id)+std::string(".txt");
    m_options.TDMA_slotmap_file = ros::package::getPath(std::string("time_domain"))+relative_filename;
    ROS_INFO("TDMA slot file: %s", m_options.TDMA_slotmap_file.c_str());
    
    m_nh.param<bool>("is_rewrite_RN_config", m_is_rewrite_RN_config, false);
}

int uavos::TimeDomainInterface::preInit(){
    if(m_options.network_type==std::string("TDMA"))
        printf("\n\nBegin configuring the TimeDomain P440 as TDMA - %d\n", m_options.node_id);
    else
        printf("\n\nBegin configuring the TimeDomain P440 as ALOHA - %d\n", m_options.node_id);

    int initOpMode;
    int status;
    rcmIfType rcmIf;
    if(std::string("IP")==m_options.connection_interface){
        rcmIf = rcmIfIp;
    } else if(std::string("USB")==m_options.connection_interface) {
        rcmIf = rcmIfUsb;
    } else {
        ROS_ERROR("Unknown connection interface!");
        return 0;
    }


    // Info message structures
    rcrmMsg_GetStatusInfoConfirm statusInfo;


    // initialize the interface to the RCM
    if(std::string("IP")==m_options.connection_interface){
        if(rcmIfInit(rcmIf, m_options.ip_address.c_str() ) != OK){
            ROS_ERROR("Initialization failed. IP.\n");
            return 0;
        }
    } else if(std::string("USB")==m_options.connection_interface){
        if (rcmIfInit(rcmIf, uavos::num2str<int>(m_options.node_id).c_str() ) != OK){
            ROS_ERROR("Initialization failed. USB.\n");
            return 0;
        }
    } else {
        ROS_ERROR("Unknown connection interface!");
        return 0;
    }
    
    // need delay for USB due to slow port (USB) handling from LINUX
    // possible for Ethernet?
    ros::Duration(1).sleep();


    // Put radio in RCM mode and set Sleep mode to Idle to keep extraneous
    //      messages from coming in while getting and setting configuration

    // Put radio in Idle
    if (rcmSleepModeSet(RCRM_SLEEP_MODE_IDLE) != 0)
    {
        ROS_ERROR("Time out waiting for sleep mode set IDLE.\n");
        return 0;
    }

    // Make sure opmode is correct
    if (rcmOpModeGet(&initOpMode) != 0)
    {
        ROS_ERROR("Time out waiting for opmode get.\n");
        return 0;
    }

    // specially for mobile, set operation mode to RCM
    if (rcmOpModeSet(RCRM_OPMODE_RCM) != 0)
    {
        ROS_ERROR("Time out waiting for opmode set RCM.\n");
        return 0;
    }

    // execute Built-In Test - verify that radio is healthy
    if (rcmBit(&status) != 0)
    {
        ROS_ERROR("Time out waiting for BIT.\n");
        return 0;
    }

    if (status != OK)
    {
        ROS_ERROR("Built-in test failed - status %d.\n", status);
        return 0;
    }
    else
    {
        ROS_INFO("Radio passes built-in test.\n\n");
    }

    // retrieve status info from RCM
    if (rcmStatusInfoGet(&statusInfo) != 0)
    {
        ROS_ERROR("Time out waiting for status info confirm.\n");
        return 0;
    }

    // print out status info
    printf("\nStatus Info:\n");
    printf("\tPackage ID: %s\n", statusInfo.packageVersionStr);
    printf("\tRCM version: %d.%d build %d\n", statusInfo.appVersionMajor,
            statusInfo.appVersionMinor, statusInfo.appVersionBuild);
    printf("\tUWB Kernel version: %d.%d build %d\n", statusInfo.uwbKernelVersionMajor,
            statusInfo.uwbKernelVersionMinor, statusInfo.uwbKernelVersionBuild);
    printf("\tFirmware version: %x/%x/%x ver %X\n", statusInfo.firmwareMonth,
            statusInfo.firmwareDay, statusInfo.firmwareYear,
            statusInfo.firmwareVersion);
    printf("\tSerial number: %08X\n", statusInfo.serialNum);
    printf("\tBoard revision: %c\n", statusInfo.boardRev);
    printf("\tTemperature: %.2f degC\n\n", statusInfo.temperature/4.0);

    return 1;
}

int uavos::TimeDomainInterface::RCMInit(){
    // retrieve RCM config
    if (rcmConfigGet(&m_rcmConfig) != 0)
    {
        ROS_ERROR("Time out waiting for rcmConfig confirm.\n");
        return 0;
    }

    // print out configuration
    printf("Initial RCM Configuration:\n");
    printRcmConfig(&m_rcmConfig);

    // Start with RCM configuration
    //
    m_rcmInitConfig = m_rcmConfig;	// Save initial configuration to restore when done
    // Ensure that the node id is consistent with the one in yaml
    if(m_rcmInitConfig.nodeId != m_options.node_id){
        ROS_ERROR("The node id of the device is different from that in yaml configuration.");
        ros::Duration(1).sleep();
        return 0;
    }

    // Clear/Set ELR
    if(false==m_options.echo_last_range)
        m_rcmConfig.flags &= ~(1 << 7);
    else
        m_rcmConfig.flags |= (1 << 7);
    // Clear Scans
    m_rcmConfig.flags &= ~(1 << 0);
    // Clear Full Scans
    m_rcmConfig.flags &= ~(1 << 1);
    // Disable CRE Ranges (note: setting the bit disables the sending of CREs)
    m_rcmConfig.flags |= (1 << 4);
    // Pii, antenna, codeChannel
    m_rcmConfig.integrationIndex = m_options.integration_index;
    m_rcmConfig.antennaMode = m_options.antenna_mode;
    m_rcmConfig.codeChannel = m_options.code_channel;
    m_rcmConfig.txGain = m_options.tx_gain;

    if(m_rcmConfig!=m_rcmInitConfig){
        // Set RCM config
        if (rcmConfigSet(&m_rcmConfig) != 0)
        {
            ROS_ERROR("Time out waiting for rcmConfig confirm.\n");
            return 0;
        }

        // Print out modified RCM Config
        printf("\nModified RCM Configuration\n");
        printRcmConfig(&m_rcmConfig);
    }

    return 1;
}

int uavos::TimeDomainInterface::RNInit(){
    // retrieve RN config
    if (rnConfigGet(&m_rnConfig) != 0)
    {
        ROS_ERROR("Time out waiting for rnConfig confirm.\n");
        return 0;
    }

    // print out configuration
    printf("\nInitial RN Configuration:\n");
    printRnConfig(&m_rnConfig);

    // Moving on to RangeNet configuration
    //
    m_rnInitConfig = m_rnConfig;	// Save initial configuration to restore when done

    // set to TDMS/ALOHA
    if(std::string("TDMA")==m_options.network_type){
        m_rnConfig.networkSyncMode = RN_NETWORK_SYNC_MODE_TDMA;
    } else {
        m_rnConfig.networkSyncMode = RN_NETWORK_SYNC_MODE_ALOHA;
    }

    // set auto sent type
    m_rnConfig.autosendType &= ~RN_AUTOSEND_RANGEINFO_FLAGS_MASK;  // no ranges
    m_rnConfig.autosendType &= ~RN_AUTOSEND_NEIGHBOR_DATABASE_FLAGS_MASK;  // no NDBs
    if(std::string("successful")==m_options.autosend_range_type){
        m_rnConfig.autosendType |= RN_AUTOSEND_RANGEINFO_SUCCESSFUL;  // successful ranges
    } else if(std::string("all")==m_options.autosend_range_type){
        m_rnConfig.autosendType |= RN_AUTOSEND_RANGEINFO_ALL;
    } else if(std::string("small")==m_options.autosend_range_type){
        m_rnConfig.autosendType |= RN_AUTOSEND_RANGEINFO_SMALL;
    } else {
        m_rnConfig.autosendType |= RN_AUTOSEND_RANGEINFO_SUCCESSFUL;  // successful ranges
    }
    if(std::string("full")==m_options.autosend_NDB_type){
        m_rnConfig.autosendType |= RN_AUTOSEND_NEIGHBOR_DATABASE_FULL;
    } else if(std::string("small")==m_options.autosend_NDB_type){
        m_rnConfig.autosendType |= RN_AUTOSEND_NEIGHBOR_DATABASE_SMALL;
    } else {
        m_rnConfig.autosendType |= RN_AUTOSEND_NEIGHBOR_DATABASE_FULL;
    }

    // Ensure Do Not Range To Me flag is not set / is set
    if(true==m_options.do_not_range_me)
        m_rnConfig.rnFlags |= (1 << 1);
    else
        m_rnConfig.rnFlags &= ~(1 << 1);

    // Set the NDB Neighbor Age arbitrarily high to keep nodes from dropping out
    m_rnConfig.maxNeighborAgeMs = m_options.max_neighbor_age_ms;
    // Set NDB update rate
    m_rnConfig.autosendNeighborDbUpdateIntervalMs = m_options.autosend_NDB_update_interval_ms;


    if(m_rnConfig!=m_rnInitConfig){
        // Set RangeNet config
        if (rnConfigSet(&m_rnConfig) != 0)
        {
            ROS_ERROR("Time out waiting for rnConfig confirm.\n");
            return 0;
        }

        // Print out modified RN Config
        printf("\nModified RN Configuration\n");
        printRnConfig(&m_rnConfig);
    }


    return 1;
}


int uavos::TimeDomainInterface::TDMAInit(){
    // retrieve TDMA Config
    if(rnTdmaConfigGet(&m_rnTdmaConfig) != 0)
    {
        ROS_ERROR("Time out waiting for rnTdmaConfigGet confirm.\n");
        return 0;
    }

    // print out configuration
    printf("\nInitial RN-TDMA Configuration:\n");
    printRnTdmaConfig(&m_rnTdmaConfig);

    m_rnInitTdmaConfig = m_rnTdmaConfig;

    // Set TDMA Config
    m_rnTdmaConfig.maxRequestDataSize = m_options.max_request_data_size;
    m_rnTdmaConfig.maxResponseDataSize = m_options.max_response_data_size;

    if(m_rnInitTdmaConfig != m_rnTdmaConfig){
        if(rnTdmaConfigSet(&m_rnTdmaConfig) != 0)
        {
            ROS_ERROR("Time out waiting for rnTdmaConfig confirm.\n");
            return 0;
        }

        // Print out modified RN-TDMA Config
        printf("\nModified RN-TDMA Configuration:\n");
        printRnTdmaConfig(&m_rnTdmaConfig);
    }


    return 1;
}

int uavos::TimeDomainInterface::TDMASlotmapInit(){
    // retrieve TDMA slot map
    if (rnTdmaSlotMapGet(&m_rnInitTdmaSlotMap) != 0)
    {
        printf("Time out waiting for rnTdmaSlotMap confirm.\n");
        return 0;
    }

    // Print out slotmap
    printf("\nInitial slotmap: Number of Slots = %d\n", m_rnInitTdmaSlotMap.numSlots);
    printRnSlotmapConfirm(&m_rnInitTdmaSlotMap);

    // read slotmap from txt
    rnMsg_SetTDMASlotmapRequest slotmap_request = uavos::readSlotmap(m_options.TDMA_slotmap_file);

    if(m_rnInitTdmaSlotMap!=slotmap_request){
        // Set TDMA Slotmap config
        if (rnTdmaSlotMapSet(&slotmap_request) != 0)
        {
            printf("Time out waiting for rnTdmaSlotMapSet confirm.\n");
            return 0;
        }

        // Print slot map
        printf("\n-------------------------------------------------------------------------\n");
        printf("Modified slotmap. Number of Slots = %d\n\n", slotmap_request.numSlots);
        printRnSlotmapRequest(&slotmap_request);
    }


    return 1;
}


int uavos::TimeDomainInterface::ALOHAInit(){
    // retrieve ALOHA config
    if (rnAlohaConfigGet(&m_rnAlohaConfig) != 0)
    {
        ROS_ERROR("Time out waiting for rnAlohaConfig confirm.\n");
        return 0;
    }

    // print out configuration
    printf("\nInitial RN ALOHA Configuration:\n");
    printRnAlohaConfig(&m_rnAlohaConfig);

    // Finally set up ALOHA configuration
    //
    m_rnInitAlohaConfig = m_rnAlohaConfig;	// Save initial configuration to restore when done

    // Ensure Beacon mode is not set / is set
    if(false==m_options.ALOHA_is_beacon)
        m_rnAlohaConfig.flags &= ~(1 << 0);
    else
        m_rnAlohaConfig.flags |= (1 << 0);
    // Turn off/on ACC
    if(false==m_options.ALOHA_ACC)
        m_rnAlohaConfig.flags &= ~(1 << 2);
    else
        m_rnAlohaConfig.flags |= (1<<2);
    // With ACC off, set Min and Max TX
    m_rnAlohaConfig.minTimeBetweenTxMs = m_options.ALOHA_min_tx_ms;
    m_rnAlohaConfig.maxTimeBetweenTxMs = m_options.ALOHA_max_tx_ms;
    // Data size
    m_rnAlohaConfig.maxRequestDataSize = m_options.max_request_data_size;
    m_rnAlohaConfig.maxResponseDataSize = m_options.max_response_data_size;

    // Set ALOHA config
    if(m_rnInitAlohaConfig != m_rnAlohaConfig){
        if (rnAlohaConfigSet(&m_rnAlohaConfig) != 0)
        {
            ROS_ERROR("Time out waiting for rnAlohaConfig confirm.\n");
            return 0;
        }

        // Print out modified Aloha Config
        printf("\nModified RN ALOHA Configuration\n");
        printRnAlohaConfig(&m_rnAlohaConfig);
    }


    return 1;
}


int uavos::TimeDomainInterface::setupUWB(){
    loopWrapper<int>(&uavos::TimeDomainInterface::preInit, this);
    loopWrapper<int>(&uavos::TimeDomainInterface::RCMInit, this);
    loopWrapper<int>(&uavos::TimeDomainInterface::RNInit, this);
    if(std::string("TDMA")==m_options.network_type){
        loopWrapper<int>(&uavos::TimeDomainInterface::TDMAInit, this);
        loopWrapper<int>(&uavos::TimeDomainInterface::TDMASlotmapInit, this);
    } else {
        loopWrapper<int>(&uavos::TimeDomainInterface::ALOHAInit, this);
    }

    if(false==m_is_rewrite_RN_config){
        // don't rewrite RN config, i.e., RCM mode after reboot
        loopWrapper<bool>(&uavos::TimeDomainInterface::activateRadio, this);
    } else {
        // rewrite RN config, i.e., RN mode after reboot
        loopWrapper<bool>(&uavos::TimeDomainInterface::activateRadioAndConfigOpmodeRn, this);
    }
    

    m_is_uwb_ready = true;
    m_last_read_ok_time = ros::Time::now();
    return 1;
}

// -------------------------- general initialization --------------------------

bool uavos::TimeDomainInterface::activateRadio(){
    // =============================
    // set operation mode to RN. This is specially for mobile
    // because setting operation mode without rewriting RN config will lead to
    // operation mode at RCM after reboot.
    // but, maybe booting time domain as RCM will be better for mobile?
    
    // Put in RangeNet Mode
    if (rcmOpModeSet(RCRM_OPMODE_RN) != 0)
    {
        ROS_ERROR("Time out waiting for opmode set RN.\n");
        return false;
    }
    // =============================
  
    // Set Sleep Mode to Active
    if (rcmSleepModeSet(RCRM_SLEEP_MODE_ACTIVE) != 0)
    {
        ROS_ERROR("Time out waiting for sleep mode set ACTIVE.\n");
        return false;
    }

    // Flush any pending messages
    rcmIfFlush();

    ROS_INFO("TimeDomain UWB is activated.");
    return true;
}

bool uavos::TimeDomainInterface::activateRadioAndConfigOpmodeRn(){
    // get the RCM up and running

    // Put in RangeNet Mode
    if (rcmOpModeSet(RCRM_OPMODE_RN) != 0)
    {
        ROS_ERROR("Time out waiting for opmode set RN.\n");
        return false;
    }
    // Update configuration
    if (rnConfigSet(&m_rnConfig) != 0)
    {
        ROS_ERROR("Time out waiting for rnConfig confirm.\n");
        return false;
    }
    // Set Sleep Mode to Active
    if (rcmSleepModeSet(RCRM_SLEEP_MODE_ACTIVE) != 0)
    {
        ROS_ERROR("Time out waiting for sleep mode set ACTIVE.\n");
        return false;
    }

    // Flush any pending messages
    rcmIfFlush();

    ROS_INFO("TimeDomain UWB is activated, rnConfig rewritten.");
    return true;
}

bool uavos::TimeDomainInterface::restoreConfig(){
    // All done. Restore original configuration
    ROS_INFO("\nAll done. Restoring original configuration...\n");
    int initOpMode;

    // Set RCM config
    if (rcmConfigSet(&m_rcmInitConfig) != 0)
    {
        ROS_ERROR("Time out waiting for rcmConfig confirm.\n");
        return false;
    }
    // Set RangeNet config
    if (rnConfigSet(&m_rnInitConfig) != 0)
    {
        ROS_ERROR("Time out waiting for rnConfig confirm.\n");
        return false;
    }
    // Set ALOHA config
    if (rnAlohaConfigSet(&m_rnInitAlohaConfig) != 0)
    {
        ROS_ERROR("Time out waiting for rnAlohaConfig confirm.\n");
        return false;
    }
    // Set opMode
    if (initOpMode != RCRM_OPMODE_RCM)
        if (rcmOpModeSet(initOpMode) != 0)
        {
            ROS_ERROR("Time out waiting for opmode set.\n");
            return false;
        }
    // Finally, put radio back into Active mode
    if (rcmSleepModeSet(RCRM_SLEEP_MODE_ACTIVE) != 0)
    {
        ROS_ERROR("Time out waiting for sleep mode set.\n");
        return false;
    }

    return true;

}

bool uavos::TimeDomainInterface::readAMessage(){
    boost::lock_guard<boost::mutex> lck(m_read_msg_mutex);
    m_semaphore_counter = 1;

    // Info message structures
    rcrmMsg_GetStatusInfoConfirm statusInfo;
    rcmMsg_FullRangeInfo rangeInfo;
    rnMsg_GetFullNeighborDatabaseConfirm ndbInfo;
    rcmMsg_DataInfo dataInfo;
    rcmMsg_EchoedRangeInfo echoedInfo;

    switch(rcmInfoGetFull(&rangeInfo, &ndbInfo, &dataInfo, &echoedInfo))
    {
        case RANGEINFO:
        {
//            printf("\nRANGE_INFO: Responder ID %d   Message ID %u\n", rangeInfo.responderId, rangeInfo.msgId);
//            printf("Range Status %d   Stopwatch %d ms\n", rangeInfo.rangeStatus, rangeInfo.stopwatchTime);
//            printf("Noise %d   vPeak %d   Measurement Type %d\n", rangeInfo.noise, rangeInfo.vPeak,
//                    rangeInfo.rangeMeasurementType);

            // The RANGE_INFO can provide multiple types of ranges
            if (rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_PRECISION)
            {
//                printf("Precision Range: %d mm, Error Estimate %d mm\n",
//                        rangeInfo.precisionRangeMm, rangeInfo.precisionRangeErrEst);
            }

            if (rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_COARSE)
            {
//                printf("Coarse Range: %d mm, Error Estimate %d mm\n",
//                        rangeInfo.coarseRangeMm, rangeInfo.coarseRangeErrEst);
            }

            if (rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_FILTERED)
            {
//                printf("Filtered Range: %d mm, Error Estimate %d mm\n",
//                        rangeInfo.filteredRangeMm, rangeInfo.filteredRangeErrEst);
//                printf("Filtered Velocity: %d mm/s, Error Estimate %d mm/s\n",
//                        rangeInfo.filteredRangeVel, rangeInfo.filteredRangeVelErrEst);
            }
            if(StatusSuccess==rangeInfo.rangeStatus){
                m_full_range_info_pub.publish(rcmRange2ros(m_options.node_id, rangeInfo));
            } else {
                m_error_range_info_pub.publish(rcmRange2ros(m_options.node_id, rangeInfo));
            }
            return true;
        }
        case FULLNDB:
        {
            printf("\nFULL_NEIGHBOR_DATABASE: Message ID %u   Number of Neighbors: %u   Sort Type %u\n",
                ndbInfo.msgId, ndbInfo.numNeighborEntries, ndbInfo.sortType);
            for (int i = 0; i < ndbInfo.numNeighborEntries; i++)
            {
                printf("Node %u: Range %d mm   LED Flags 0x%X   vPeak %d   Last Heard %d ms\n",
                    ndbInfo.neighbors[i].nodeId, ndbInfo.neighbors[i].rangeMm,
                    ndbInfo.neighbors[i].ledFlags, ndbInfo.neighbors[i].vPeak,
                    ndbInfo.timestamp - ndbInfo.neighbors[i].lastHeardTimestampMs);
            }

            m_NDB_pub.publish(rnNDB2ros(m_options.node_id, ndbInfo));
            return true;
        }
        case DATAINFO:
        {
            m_data_info_pub.publish(rcmData2ros(m_options.node_id, dataInfo));
            return true;
        }
        case ECHOEDINFO:
        {
            printf("\nEchoed range received.\n");
            m_echoed_range_pub.publish(rcmEcho2ros(echoedInfo));
            return true;
        }
    } // switch

    return false;
}

void uavos::TimeDomainInterface::readMsgCallback(const ros::TimerEvent &event){
    if(this->readAMessage()){
        m_last_read_ok_time = ros::Time::now();
    }
}



void uavos::TimeDomainInterface::sendDataToNetworkCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg){
    boost::lock_guard<boost::mutex> lck(m_read_msg_mutex);
    if(m_semaphore_counter==0){
        ROS_INFO("Send data rejected.");
        return;
    } else {
        m_semaphore_counter = 0;
    }

    // record time
     ros::Time begin_t = ros::Time::now();

    // so far only bytes (unsigned char) is implemented.
    if(msg->data.size()==0){
        return;
    }

    int max_msg_length = 1024;
    static unsigned char* byte_array = new unsigned char[max_msg_length];
    if(msg->data.size()>max_msg_length){
        ROS_ERROR("A very large packet received. Discard.");
        return;
    }

    for(int i=0;i<msg->data.size();++i){
        byte_array[i] = msg->data.at(i);
    }
    rcmDataSend(RCRM_ANTENNAMODE_TXA_RXA, msg->data.size(), byte_array);


    // record time
    ros::Time end_t = ros::Time::now();
    float duration = (end_t-begin_t).toSec();
    ROS_INFO("Send data duration: %f", duration);

}
