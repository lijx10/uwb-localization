import sys
import numpy as np
import math
import csv

# slotType: range-1; data-2
# slot_counter
# flags: 1 for range, 6 for data
#       define RN_TDMA_SLOT_FLAG_SLEEP				1
#       define RN_TDMA_SLOT_FLAG_REQUESTER_DATA	    2
#       define RN_TDMA_SLOT_FLAG_RESPONDER_DATA	    4
# integrationIndex: 5
# antennaMode:
#       define RCRM_ANTENNAMODE_TXA_RXA		0
#       define RCRM_ANTENNAMODE_TXB_RXB		1
#       define RCRM_ANTENNAMODE_TXA_RXB		2
#       define RCRM_ANTENNAMODE_TXB_RXA		3
# codeChannel: default 0
# reserved1: 0
# requesterId
# responderId
# requestedDurationMicroseconds: 0 for auto

global_antenna_mode = 0

synchronization_pii = 5
synchronization_request_us = int(4.5*1000)

data_pii = 5
data_request_us = int(10.5*1000)
# PII=5, 100 byte, min 6.215; 150 byte, min 7.716, 10 byte, min 3.92
# PII=6, 150 byte, min 13.513; 10 byte, min 5.948
# PII=7, 150 byte, min 24.832
# PII=8, 150 byte, min 36.439
# PII=9, 150 byte, min 71.052

range_pii = 5
range_request_us = int(10.5*1000)
# PII=5, min 10.086
# PII=6, min 14.130
# PII=7, min 21.672
# PII=8, min 37.122
# PII=9, min 68.39

# to eliminate possible interference
# here, each anchor use it's own code channel
def get_code_channel(requester_id, responder_id):
    code_channel = round(responder_id-100)
    if code_channel>=0 and code_channel<=6:
        return code_channel
    else:
        return 0

def create_synchronization_slot(slot_counter, requesterId):
    slotType = 2
    # slot_counter
    flags = 6
    integrationIndex = synchronization_pii
    antennaMode = global_antenna_mode
    codeChannel = 0
    reserved1 = 0
    # requesteId
    responderId = requesterId+1
    requestedDurationMicroseconds = synchronization_request_us
    return np.array([slotType, slot_counter, flags, integrationIndex, antennaMode, codeChannel, reserved1, requesterId,
            responderId, requestedDurationMicroseconds])

def create_data_slot(slot_counter, requesterId):
    slotType = 2
    # slot_counter
    flags = 6
    integrationIndex = data_pii
    antennaMode = global_antenna_mode
    codeChannel = 0
    reserved1 = 0
    # requesteId
    responderId = requesterId
    requestedDurationMicroseconds = data_request_us
    return np.array([slotType, slot_counter, flags, integrationIndex, antennaMode, codeChannel, reserved1, requesterId,
            responderId, requestedDurationMicroseconds])

def create_range_slot(slot_counter, requesterId, responderId):
    slotType = 1
    # slot_counter
    flags = 1
    integrationIndex = range_pii
    antennaMode = global_antenna_mode
    codeChannel = get_code_channel(requesterId, responderId)
    reserved1 = 0
    # requesterId
    # responderId
    requestedDurationMicroseconds = range_request_us
    return np.array([slotType, slot_counter, flags, integrationIndex, antennaMode, codeChannel, reserved1, requesterId,
            responderId, requestedDurationMicroseconds])

def create_idle_slot(slot_counter):
    requesterId = 404
    responderId = 404
    slotType = 1
    # slot_counter
    flags = 1
    integrationIndex = range_pii
    antennaMode = global_antenna_mode
    codeChannel = 0
    reserved1 = 0
    # requesterId
    # responderId
    requestedDurationMicroseconds = range_request_us
    return np.array([slotType, slot_counter, flags, integrationIndex, antennaMode, codeChannel, reserved1, requesterId,
            responderId, requestedDurationMicroseconds])


def anchor_index_round_bin(num_of_anchor, raw_anchor_index):
    if raw_anchor_index<num_of_anchor:
        return raw_anchor_index
    else:
        return raw_anchor_index - num_of_anchor

def slotmap_to_csv(slotmap_txt, id):
    slotmap_csv = []
    slotmap_csv.append(
        ['Slot #', 'Requester  ID', 'Responder ID', 'Pii,Chan.', 'Antenna Mode', 'Req. Data', 'Rsp. Data', 'Type',
         'Sleep', 'Manual Time (ms)', 'Minimum Time (ms)'])
    for i in range(slotmap_txt.shape[0]):
        slotType = slotmap_txt[i,0]
        slot_counter = slotmap_txt[i,1]
        flags = slotmap_txt[i,2]
        integrationIndex = slotmap_txt[i,3]
        antennaMode = slotmap_txt[i,4]
        codeChannel = slotmap_txt[i,5]
        requesterId = slotmap_txt[i,7]
        responderId = slotmap_txt[i,8]
        requestedDurationMicroseconds = slotmap_txt[i,9]

        if slotType==1:
            slotType = 'Range'
        else:
            slotType = 'Data'

        if antennaMode==0:
            antennaMode = 'Antenna A'
        elif antennaMode==1:
            antennaMode = 'Antenna B'

        reqData = 1
        rspData = 1
        slotmap_csv.append(
            [slot_counter, requesterId, responderId, integrationIndex, codeChannel, antennaMode, reqData, rspData,
             slotType, flags, requestedDurationMicroseconds, 0])

    f = open('csv/'+str(id)+'.csv', 'w')
    writer = csv.writer(f, quoting=csv.QUOTE_NONE, escapechar=' ')
    for i, line in enumerate(slotmap_csv):
        writer.writerow(line)
    f.close()

if __name__=="__main__":
    # assume that num_of_mobile >= num_of_anchor
    # --------  configuration ----------
    num_of_anchor = 6
    num_of_mobile = 10

    anchor_id_list = [101, 102, 103, 104, 105, 106]
    mobile_id_list = [301, 302, 303, 304, 305, 306, 307, 308, 309, 310]

    number_of_data_slot = 1
    # --------  configuration ----------

    # first of all, fill the mobile list to have exactly integer batches
    fill_num = int(math.ceil(float(num_of_mobile) / float(num_of_anchor))) * num_of_anchor - num_of_mobile
    print fill_num
    augmented_mobile_id_list = list(mobile_id_list) # deep copy
    for i in range(fill_num):
        augmented_mobile_id_list.append(mobile_id_list[i])
    print('Augmented mobile id list:')
    print(augmented_mobile_id_list)

    # batch process
    batch_num = int(round(float(len(augmented_mobile_id_list)) / float(num_of_anchor)))
    print('batch number: {}'.format(batch_num))

    # the first is data slot, for sync
    totol_num_of_slot = batch_num*num_of_anchor + number_of_data_slot
    anchor_slotmap_list = np.zeros((num_of_anchor, totol_num_of_slot, 10), dtype=np.int)
    mobile_slotmap_list = np.zeros((num_of_mobile, totol_num_of_slot, 10), dtype=np.int)

    slot_counter = 0
    # add the master as the 0 slot for synchronization
    for i, anchor_id in enumerate(anchor_id_list):
        anchor_slotmap_list[i, 0, :] = create_synchronization_slot(slot_counter, anchor_id_list[0])
    for i, mobile_id in enumerate(mobile_id_list):
        mobile_slotmap_list[i, 0, :] = create_synchronization_slot(slot_counter, anchor_id_list[0])
    slot_counter += 1

    data_slot_counter = 1

    # use the batch process style, in each batch, "num_of_anchor" mobiles will be processed.
    # each anchor will use its own channel, i.e. CDMA + TDMA

    for round_bin_index in range(num_of_anchor):
        for busy_batch in range(batch_num):
            # process the mobiles that are the focus of current batch
            busy_mobile_index_list = []
            for i in range(num_of_anchor):
                targeting_anchor_index = anchor_index_round_bin(num_of_anchor, round_bin_index+i)

                # determine mobile index
                mobile_index_in_aug = i + busy_batch * num_of_anchor
                if(mobile_index_in_aug>=num_of_mobile):
                    mobile_id = augmented_mobile_id_list[mobile_index_in_aug]
                    mobile_index = mobile_id_list.index(mobile_id)
                else:
                    mobile_index = mobile_index_in_aug

                mobile_slotmap_list[mobile_index, slot_counter, :] = create_range_slot(
                    slot_counter,
                    mobile_id_list[mobile_index],
                    anchor_id_list[targeting_anchor_index]
                )
                anchor_slotmap_list[targeting_anchor_index, slot_counter, :] = create_range_slot(
                    slot_counter,
                    mobile_id_list[mobile_index],
                    anchor_id_list[targeting_anchor_index]
                )

                # record what mobile is busy
                busy_mobile_index_list.append(mobile_index)
            
            # process the idle mobiles
            for idle_batch in range(batch_num):
                # the busy batch has been processed by the above code
                if idle_batch==busy_batch:
                    continue

                for i in range(num_of_anchor):
                    # determine mobile index
                    mobile_index_in_aug = i + idle_batch * num_of_anchor
                    if (mobile_index_in_aug >= num_of_mobile):
                        mobile_id = augmented_mobile_id_list[mobile_index_in_aug]
                        mobile_index = mobile_id_list.index(mobile_id)
                    else:
                        mobile_index = mobile_index_in_aug

                    if mobile_index in busy_mobile_index_list:
                        continue
                    mobile_slotmap_list[mobile_index, slot_counter, :] = create_data_slot(slot_counter,
                                                                                          mobile_id_list[mobile_index])

            slot_counter += 1






    print anchor_slotmap_list
    print "==========================="
    print mobile_slotmap_list

    # save to txt --------------------------
    for i in range(num_of_anchor):
        np.savetxt('txt/' + str(anchor_id_list[i]) + '.txt', anchor_slotmap_list[i, ...],
                   fmt='%d', delimiter=', ', newline='\n')
    for i in range(num_of_mobile):
        np.savetxt('txt/' + str(mobile_id_list[i]) + '.txt', mobile_slotmap_list[i, ...],
                   fmt='%d', delimiter=', ', newline='\n')

    # save to csv --------------------------
    for i in range(num_of_anchor):
        slotmap_to_csv(anchor_slotmap_list[i,...], anchor_id_list[i])
    for i in range(num_of_mobile):
        slotmap_to_csv(mobile_slotmap_list[i,...], mobile_id_list[i])
