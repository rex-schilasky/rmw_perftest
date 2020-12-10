"""Performance plotter.

This module defines some helper functios to create nice plots out of the performance log files.

"""

import os
import functools
import itertools
import pandas
import pprint
import matplotlib.pyplot as plt

def read_experiment_folders(rootdir):
    """Creates a nested dictionary that represents the folder structure of rootdir."""
    exp_d = {}
    rootdir = rootdir.rstrip(os.sep)
    start = rootdir.rfind(os.sep) + 1
    for path, dirs, files in os.walk(rootdir):
        folders = path[start:].split(os.sep)
        subdir = dict.fromkeys(files)
        parent = functools.reduce(dict.get, folders[:-1], exp_d)
        parent[folders[-1]] = subdir
        for k in subdir: subdir[k] = path
    return exp_d

def cleanup_experiments(exp_d, lprefix):
    """Removes all none log files"""
    for k,v in exp_d.items():
        if isinstance(v, dict):
            v = {k:w for k,w in v.items() if isinstance(w, dict) or ( k.startswith(lprefix) and not k.endswith('pdf'))}
            exp_d[k] = v
            cleanup_experiments(v, lprefix)
    return exp_d

# used from apex ai project (https://gitlab.com/ApexAI/performance_test) !
def load_logfile(filename):
    """Load logfile into header dictionary and pandas dataframe."""
    with open(filename) as source:
        header = {}
        for item in itertools.takewhile(lambda x: not x.startswith('---'), source):
            if not item.strip():  # Don't care about whitespace-only lines
                continue
            try:
                key = item.split(':')[0].strip()
                value = item.split(':', maxsplit=1)[1].strip()
                header[key] = value
            except Exception:
                print('Error trying to parse header line "{}"'.format(item))
                raise
        dataframe = pandas.read_csv(source, sep='[ \t]*,[ \t]*', engine='python')
        unnamed = [col for col in dataframe.keys() if col.startswith('Unnamed: ')]
        if unnamed:
            dataframe.drop(unnamed, axis=1, inplace=True)
    return header, dataframe


def read_logs(exp_d, log_l = []):
    """Read content of log files."""
    for k,v in exp_d.items():
        if isinstance(v, dict):
            read_logs(v, log_l)
        else:
            full_path = os.path.join(v,k)
            log_d = {}
            log_d['path'] = full_path
            log_d['header'], log_d['dataframe'] = load_logfile(full_path)
            log_l.append(log_d)
    return log_l

def calc_mean(log_l, lnames_l):
    """Calculate average value of specific log value names."""
    for d in log_l:
        df = d['dataframe']
        col_l = df.columns.values.tolist()
        for val in lnames_l:
            col_l.remove(val)
        df = df.drop(col_l, axis=1)
        d['dataframe'] = df.mean(axis=0)
    return log_l

def load_logfiles(rootdir, lprefix, lnames_l):
    """Load all log files and read their content."""
    exp_d = read_experiment_folders(rootdir)
    exp_d = cleanup_experiments(exp_d, lprefix)
    log_l = read_logs(exp_d)
    log_l = calc_mean(log_l, lnames_l)
    return log_l

def plot_bar(log_l, rmw_l, msg_l, lname_l):
    """Create bar plot for one run."""
    rmw_d = {r : {m: {} for m in msg_l} for r in rmw_l}
    for d in log_l:
        rmw = d['header']['RMW Implementation']
        msg = d['header']['Msg name']
        df  = d['dataframe']
        if rmw in rmw_l and msg in msg_l:
            rmw_d[rmw][msg] = df[0]
    ax = pandas.DataFrame(rmw_d).plot(kind='bar', title=lname_l[0], grid='True')
    plt.show()


if __name__ == "__main__":

    exp_folder = "../../experiments/003/pub_01-sub_01-rate_0010-hist_last"

    msg_l = ['Array1k','Array4k','Array16k','Array32k','Array60k','Array1m','Array2m','Struct16','Struct256','Struct4k','Struct32k','PointCloud512k','PointCloud1m','PointCloud2m','PointCloud4m','Range','NavSatFix','RadarDetection','RadarTrack']
    
    rmw_l   = ['rmw_cyclonedds_cpp', 'rmw_fastrtps_cpp', 'rmw_ecal_dynamic_cpp']
    lname_l = ['latency_mean (ms)']
    #lname_l = ['ru_maxrss']
    #lname_l = ['cpu_usage (%)']    

    log_l = load_logfiles(exp_folder, 'log', lname_l)
    plot_bar(log_l, rmw_l, msg_l, lname_l)
