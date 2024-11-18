'''
TODO: optimization
- use queue to store events during runtime. It has async behavior and can be used to store log to flas asynchronously
- second option is to block a fixed memory in ram using tuple fo lists, so that it is more efficient than appending list where it need to rearrange memory for every append. This will need atleast two buffers
- use u long long for timestamp and uint for event number
'''

import utime
import json
import os
import sys
import gc
from lib.exeint import exeInt

class VarLogger:
    '''
    This class is the part of the tool, it logs data and collects all the information required to monitor the system
    all the code lines that are part of the tools are commented using '#/////'

    The LOGGING mode logs the data to a file and the TRACE mode logs the sequence of events to a file.
    The DETECTION mode performs detection alon with data logging.
    '''
    TRACE_LENGTH = 200
    buffer_select = 1     ##### 1 for data1, 2 for data2
    save_buffer = 0    ##### 1 for data1, 2 for data2, 0 for none
    buffer_index = 0
    DETECTION = True   ### flag to enable detection, if True, it will perform detection+logging, if False, it will only do logging

    gc.collect()  # Run garbage collection to get accurate memory info
    before = gc.mem_free()  # Get available memory before allocation
    data1 = [(0, 0)] * TRACE_LENGTH   ### store variale sequence
    data2 = [(0, 0)] * TRACE_LENGTH   ### store variale sequence
    gc.collect()  # Run garbage collection again, if needed
    after = gc.mem_free()  # Get available memory after allocation
    used_memory = before - after  # Calculate used memory
    print("Memory used by {} event list: {} bytes".format(TRACE_LENGTH, used_memory))  # Print used memory

    gc.collect()  # Run garbage collection to get accurate memory info
    before = gc.mem_alloc()  # Get aallocated memory
    detection_buffer = dict() ### store the timestamps for each variable for runtime detection
    if DETECTION:
        print('Detection mode enabled')
        ### check if thresholds file exists
        if 'thresholds.json' not in os.listdir():
            raise('Thresholds file not found')
        else:
            ### load thresholds
            with open('thresholds.json', 'r') as f:
                thresholds = json.load(f)
                print('Thresholds loaded')
                print('Thresholds:\n', thresholds)

            ### load ei model
            ei_model = exeInt()
            print('EI model loaded')

    gc.collect()  # Run garbage collection to get accurate memory info
    after = gc.mem_alloc()  # Get aallocated memory
    used_memory = after - before  # Calculate used memory
    print('Memory for detection setup (buffer, thresholds, model):', used_memory)
        

    created_timestamp = utime.ticks_ms()  ### start time
    data_dict = {}  ### store timestamp for each variable
    _catchpop = 0  ### temporary storage
    _write_count = 0  ### count to track writing frequency
    _thread_map = dict() ### to map the threads to integer numbers
    write_name, trace_name = ['log0', 'trace0']
    _vardict = dict() ### dict of variables
    cur_file = 0 ### file number
    overhead_time = 0 ### time to write to flash
    avg_detection_time = [0,0]  ### totoal detection time for all detections, count of detections
    

    ####### thread tracking
    threads_info = dict() ### init a dictionary to store the status of each thread

    ####### avoid duplicate events
    prev1_event = -1   ### to avoid clasing with first event with index 0
    prev2_event = -1

    prev1_time = 0
    prev2_time = 0


    # ### avoid overwriting existing files by checking existing files
    # prev_file = ''
    # with open('log_check', 'rb') as f:
    #     f = f.readline()
    #     prev_file = f

    # if prev_file != '':
    #     cur_file = int(prev_file)+1
    #     write_name = 'log{}'.format(cur_file)
    #     trace_name = 'trace{}'.format(cur_file)
    #     with open('log_check', 'wb') as f:
    #         f.write(str(cur_file))


    @classmethod
    def log(cls, var='0', fun='0', clas='0', th='0', val=None, save=None, detection=None):
        '''
        var -> str = name of the variable
        fun -> str = name of the function
        clas -> str = name of the class
        th -> str = thread id
        val -> any = value of the variable
        save -> bool = save the data to flash. If None, it will save the data after 1000 events. If True, it will save the data immediately. If False, it will not save the data
        '''
        dict_keys = cls.data_dict.keys()
        ### map thread id to a single digit integer fo simplicity
        th = cls.map_thread(th)
        ### make the event name based on the scope
        event = '{}-{}-{}-{}'.format(th, clas, fun, var)
        log_time = utime.ticks_ms() - cls.created_timestamp - cls.overhead_time

        event_num = cls._var2int(event)

        # if event_num in dict_keys:
        #     _vartimestamps = cls.data_dict[event_num]

        #     ### save only 500 latest values for each variable
        #     while len(_vartimestamps) >= 500: 
        #         cls._catchpop = _vartimestamps.pop(0)
            
        #     _vartimestamps += [(log_time, val)]
        #     cls.data_dict[event_num] = _vartimestamps ### format of cls.data_dict = {event_num: [(timestamp1,val), (timestamp2, val), ...]}

        # else:
        #     cls.data_dict[event_num] = [(log_time, val)]

        ### log the sequence to trace file, but only unique events (avoid duplicates)
        if cls.prev1_event != event_num:
            cls.log_seq(event_num, log_time)
            cls.detection_dict(event_num, log_time)
            cls._write_count +=1
        else:
            pass

        #print(cls._write_count)
        ### write to flash approx every 6 secs (counting to 1000 = 12 ms)
        num_events = cls.TRACE_LENGTH
        if (cls._write_count >= num_events):
            if save!=False:
                cls._write_count = 0
                start_time = utime.ticks_ms()
                cls.write_data() ### save the data to flash
                cls.overhead_time += (utime.ticks_ms()-start_time)   ### add the time taken to write to flash to avoid spikes in time
                print('write time for {}:'.format(num_events), utime.ticks_ms()-start_time)
                cls.data = [] ### clear the data after writing to flash
                gc.collect()
        
            ### check if in DETECTION mode 
            if cls.DETECTION and detection!=False: 
                start_time = utime.ticks_ms()
                anomalies_detected = cls.ei_model.runtime_detection(cls.detection_buffer, cls.thresholds, cls._int2var)
                if len(anomalies_detected) > 1:
                    merged_anomalies, _ = cls.ei_model.merge_detections(anomalies_detected)
                else:
                    merged_anomalies = anomalies_detected
                print('Anomalies detected:\n', merged_anomalies)
                cls.save_detections(merged_anomalies)
                cls.clear_detection_buffer()
                cls.overhead_time += (utime.ticks_ms()-start_time )  ### add the time taken to write to flash to avoid spikes in time
                print('Detection Time:', utime.ticks_ms()-start_time)
                cls.avg_detection_time[0] += (utime.ticks_ms()-start_time)
                cls.avg_detection_time[1] += 1
            

        ### check previous 3 events to avoid duplicate events
        cls.prev2_event = cls.prev1_event
        cls.prev1_event = event_num

        cls.prev2_time = cls.prev1_time
        cls.prev1_time = log_time
                
    @classmethod
    def detect_ei(cls,threhold_path):
        pass

    @classmethod
    def _var2int(cls, var):
        '''
        map the variable names to integers for easy access
        '''
        if var not in cls._vardict.keys():
            cls._vardict[var] = len(list(cls._vardict.keys()))
        
        return cls._vardict[var]
    

    @classmethod
    def _int2var(cls, num):
        '''
        map the integers back to variable names
        '''
        for key, value in cls._vardict.items():
            if value == num:
                return key


    @classmethod
    def log_seq(cls, event, log_time):
        # print('buffer {} index:'.format(cls.buffer_select), cls.buffer_index)
        if cls.buffer_select == 1:
            # print('storing in buffer 1 at {}:'.format(cls.buffer_index))
            ### if the buffer is full, switch to buffer 2
            if cls.buffer_index == cls.TRACE_LENGTH-1:
                cls.data1[cls.buffer_index] = (event, log_time)
                cls.buffer_select = 2
                cls.save_buffer = 1
                cls.buffer_index = 0
            ### if the buffer is empty/full, initialize it
            elif cls.buffer_index == 0:
                cls.data1[cls.buffer_index] = (event, log_time)
                cls.buffer_index += 1
            else:
                cls.data1[cls.buffer_index] = (event, log_time)
                cls.buffer_index += 1

        elif cls.buffer_select == 2:
            # print('storing in buffer 2 at {}:'.format(cls.buffer_index))
            ### if the buffer is full, switch to buffer 1
            if cls.buffer_index == cls.TRACE_LENGTH-1:
                cls.data2[cls.buffer_index] = (event, log_time)
                cls.buffer_select = 1
                cls.save_buffer = 2
                cls.buffer_index = 0
            ### if the buffer is empty/full, initialize it
            elif cls.buffer_index == 0:
                cls.data2[cls.buffer_index] = (event, log_time)
                cls.buffer_index += 1
            else:
                cls.data2[cls.buffer_index] = (event, log_time)
                cls.buffer_index += 1
        

    @classmethod
    def detection_dict(cls, event, log_time):
        '''
        store the timestamps for each variable for runtime detection
        '''
        if event not in cls.detection_buffer.keys():
            cls.detection_buffer[event] = [log_time]
        else:
            cls.detection_buffer[event].append(log_time)

    
    @classmethod
    def clear_detection_buffer(cls):
        '''
        clear the detection buffer after performing detection
        keep last time stamp for each variable/event
        '''
        keys = cls.detection_buffer.keys()
        for key in keys:
            cls.detection_buffer[key] = [cls.detection_buffer[key][-1]]


    @classmethod
    def save_detections(cls, detections):
        '''
        save the detections to a file
        '''
        if detections != []:
            if 'detections.json' in os.listdir():
                ### append detections to the existing file
                with open('detections.json', 'r') as f:
                    _detections = json.load(f)
                    _detections.extend(detections)
                    with open('detections.json', 'w') as f:
                        to_write = json.dumps(_detections)
                        f.write(to_write)
                        print('Detections saved')
            else:
                ### create a new file and save detections
                with open('detections.json', 'w') as f:
                    to_write = json.dumps(detections)
                    f.write(to_write)
                    print('Detections saved')
        else:
            return
        
    # @classmethod
    # def check_files(cls):
    #     '''
    #     check for previous log and update the name to avoid overwriting
    #     '''
    #     _files = os.listdir()
    #     _filename = 'log0'
    #     _seqname = 'trace0'
    #     _varlistname = 'varlist0'

    #     for i in range(100):
    #         if _filename in _files:
    #             _filename = 'log{}'.format(i+1)
    #             _seqname = 'trace{}'.format(i+1)
    #             _varlistname = 'varlist{}'.format(i+1)
    #         else:
    #             break

    #     return (_filename,_seqname, _varlistname)
    

    @classmethod
    def write_data(cls):
        # with open(cls.write_name, 'w') as fp:
        #     json.dump(cls.data_dict, fp)
        #     print('dict saved', cls.write_name)

        if cls.save_buffer == 1:
            with open(cls.trace_name, 'w') as fp:
                to_write = json.dumps(cls.data1)
                fp.write(to_write)
                # json.dump(cls.data1, fp)
                print('buffer1 saved', cls.trace_name)
                ### clear the buffer after writing to flash
                cls.save_buffer = 0
                gc.collect()  # Run garbage collection again, if needed
                before = gc.mem_free()  # Get available memory after allocation
                cls.data1 = [(0, 0)] * cls.TRACE_LENGTH
                gc.collect()  # Run garbage collection again, if needed
                after = gc.mem_free()  # Get available memory after allocation
                used_memory =  after - before # Calculate used memory
                print("Memory used by full buffer list: {} bytes".format(used_memory))  # Print used memory

        elif cls.save_buffer == 2:
            with open(cls.trace_name, 'w') as fp:
                to_write = json.dumps(cls.data2)
                fp.write(to_write)
                # json.dump(cls.data2, fp)
                print('buffer2 saved', cls.trace_name)
                ### clear the buffer after writing to flash
                cls.save_buffer = 0
                cls.data2 = [(0, 0)] * cls.TRACE_LENGTH

        with open('varlist'+ cls.trace_name[5:], 'w') as fp: ### save the variable list for each log file
            to_write = json.dumps(cls._vardict)
            fp.write(to_write)
            # json.dump(cls._vardict, fp)
            print('varlist saved', cls.trace_name[5:])

        cls.cur_file += 1
        cls.trace_name = 'trace{}'.format(cls.cur_file)


    @classmethod
    def save(cls):
        #### using write_data in main scripts results in empty log files, data is lost
        cls.write_data()


    @classmethod
    def thread_status(cls, thread_id=None, status=None):
        '''
        update or retrive the status of the thread. If no value is given to 'status' and 'thread_id' it will return the status of all the threads and it's ids
        status = ['dead' or 'alive']
        to update: pass arguments to thread_id and status
        to get status: dont pass andy arguments
        '''
        ### update status of the thread if it is active or not
        ids = cls.threads_info.keys()
        
        assert(status=='dead' or status=='active' or status==None)

        ### if status is given then update the status for respective thread
        if status!=None and thread_id!=None :
            ### update the status of the thread in the dict
            cls.threads_info[thread_id] = status

            ### add the thread to mapping dict
            num = cls._thread_map.keys()
            if thread_id not in num:
                ### as kernel thread is already initialized with name='main', so the thread count start with 1, which is also the len(num) due to main thread
                cls._thread_map[thread_id] = len(list(num)) 

        else:
            return(ids, cls.threads_info)
        

    @classmethod
    def map_thread(cls, thread_id):
        ### amp the long thread id to an integer based on thread_map
        num = cls._thread_map.keys()
        if thread_id not in num:
            raise('Thread not found')
        
        mapped_id = cls._thread_map[thread_id]
        return mapped_id
    

    @classmethod
    def traceback(cls, exc):
        ### write the traceback that is generated in the except block to a text file for future debugging
        with open("traceback.txt", "a", encoding="utf-8") as f:
            f.write('\n {} \n'.format(utime.ticks_ms()))
            sys.print_exception(exc, f)