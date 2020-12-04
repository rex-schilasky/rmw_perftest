"""Performance plotter.

This module defines some helper functios to create nice plots out of the performance log files.

"""

import os
import functools
import itertools
import pandas
import pprint

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

def load_logfiles(rootdir, lprefix):
    """Load all log files and read their content."""
    exp_d = read_experiment_folders(rootdir)
    exp_d = cleanup_experiments(exp_d, lprefix)
    log_l = read_logs(exp_d)
    return log_l


if __name__ == "__main__":
    exp_folder = "../../experiments/003/pub_01-sub_01-rate_0010-hist_last"
    log_l      = load_logfiles(exp_folder, 'log')
    pprint.pprint(log_l);
