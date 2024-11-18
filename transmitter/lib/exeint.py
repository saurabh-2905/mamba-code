import json
import os




class exeInt:
    def __init__(self):
        pass


    def runtime_detection(self, var_ts, thresholds=None, int2var=None):
        '''
        var_ts: dict of variable and timestamp -> dict
        thresholds: dictionary containing the threshold values for each variable -> dict

        return:
        detected_anomalies: list of detected anomalies -> list
        '''
        detected_anomalies = []
        keys = list(var_ts.keys())
        # print('thresholds:\n', thresholds)
        # print('var_ts:\n', var_ts)
        ### iterate trace and make decision for each exe interval
        # print('keys:', keys)
        for key in keys:
            # print('key:', key)
            var = int2var(key) ### get variable name from the associated integer
            timestamps = var_ts[key]
            # print('var:', var)
            # print('timestamps:', timestamps)
            ### calculate exe interval
            if len(timestamps) > 1:
                for i in range(1, len(timestamps)):
                    exe_time = timestamps[i] - timestamps[i-1]
                    exe_time = round(exe_time/1000, 1)
                    # print('var:', var, 'exe_time:', exe_time)

                    if thresholds != None:
                        # print('var:\n', var)
                        # print('threshold var', thresholds[var])
                        if var in thresholds.keys():
                            ### check if exe_time is an outlier
                            if exe_time < thresholds[var][0] or exe_time > thresholds[var][1]:
                                print('Anomaly detected for {} at '.format(var), timestamps[i])
                                print('exe_time:', exe_time, 'threshold var', thresholds[var])
                                lb = max(timestamps[i]-(thresholds[var][1]*1000*1.5), timestamps[i]-15000)
                                detected_anomalies += [[(key,0), (lb, timestamps[i]), None]]
        return detected_anomalies
    

    def merge_detections(self, detections, diff_val=5):
        '''
        This fucntions merges multiple detections that are less the 2 seconds apart. 
        These multiple detections can be caused because of multiple variables or even multiple anomalies that are colser
        Each anomaly that occurs, affects group of variables, resulting in multiple detections for single ground truth
        This function groups the detections based on the time difference between them and selects one from each group

        detections: list of detected anomalies -> list -> in format: [[(var1, 0), (ts1, ts2), filename], [(var2, 0), (ts1, ts2), filename], ....]
        diff_val: time difference threshold in seconds to group detections -> int/float

        return:
        dedup_detection: list of deduplicated detections -> list
        '''
        DIFF_VAL = diff_val
        pred = detections
        ### sort the list using the first timestamp of every detection
        pred = sorted(pred, key=lambda x: x[1][0])
        # print('sorted detecions:', pred)
        det_ts1 = [ x[1][0]/1000 for x in pred]  ### get first timestamp of every detection and convert from mili second to second
        det_ts2 = [ x[1][1]/1000 for x in pred]  ### get first timestamp of every detection and convert from mili second to second
        # print('merge ts:', pred[0][1], det_ts1[0], det_ts2[0])
        group = []
        group_ind = []
        aggregated_ts = []
        aggregated_ts_ind = []
        ymax = 0
        cond1 = False
        cond2 = False
        for xi, (x1, x2, y1, y2) in enumerate(zip(det_ts1[0:-1], det_ts1[1:], det_ts2[0:-1], det_ts2[1:])):    ### get the first and last timestamp of every detection
            # print(xi)
            ### diff between start points of two detections
            # diff_ts1 = abs(x2 - x1)
            ### diff between start of first detection and end of second detection
            if y1 > ymax:
                ymax = y1
                
            if group != []:
                cond1 = x2 < ymax
            else:
                cond1 = x2 < y1
            diff_ts2 = abs(x2 - y1)
            cond2 = diff_ts2 <= DIFF_VAL
            # print('Merge diff:', diff_ts1, x1, x2)
            ### decision to wether or not group detections. If the difference between the detections is less than diff_val seconds, then group them
            ### if the difference between the detections is more than diff_val seconds, 
            ### then check if the second detection has started before first detection ends or 
            ### the second detecion starts withing diff_val seconds from end of rist detection. If yes, then group them
            if cond1 or cond2:
                group += [pred[xi]]
                group_ind += [xi]
                if xi == len(det_ts1)-2:  ### for last pair
                    group += [pred[xi+1]]
                    group_ind += [xi+1]
                    aggregated_ts_ind += [group_ind]
                    aggregated_ts += [group]
                ### store the highest ts that shows end of the groupped detections
            else:
                group_ind += [xi]
                group += [pred[xi]]   ### group the predictions which have time diff less than 2 seconds
                # print(group)
                aggregated_ts_ind += [group_ind]   
                aggregated_ts += [group]    ### collect all the groups
                group = []
                ymax = 0
                if xi == len(det_ts1)-2:   ### for last pair
                    group += [pred[xi+1]]
                    group_ind += [xi+1]
                    aggregated_ts_ind += [group_ind]
                    aggregated_ts += [group]

        ### merge the detections (starting TS from first detection and ending TS from last detection from each group)
        merge_detection = []
        for gp in aggregated_ts:    #### read single group of detections. in format list of detections
            ### sort the detections in ascending order based on the first timestamp
            gp = sorted(gp, key=lambda x: x[1][0])
            #### scan through the group and get the lowest first timestamp and highest last timestamp
            lowest_ts = gp[0][1][0]
            highest_ts = gp[-1][1][1]
            first_event = gp[0][0][0]
            last_event = gp[-1][0][0]
            for i, (_, gts, _) in enumerate(gp):
                if gts[0] < lowest_ts:
                    lowest_ts = gts[0]
                    first_event = gp[i][0][0]
                if gts[1] > highest_ts:
                    highest_ts = gts[1]
                    last_event = gp[i][0][0]


            merge_detection += [[(first_event, last_event), (lowest_ts, highest_ts), gp[0][2]]]    
            ### eg. [(10, 8), (27249, 30675), 'trace2-sensor'], where first tuple contains variable of first and last detection in group, second tuple is the first TS of first detection and second ts of last detection, and the file name is taken from first detection

        return merge_detection, aggregated_ts

