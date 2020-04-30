from __future__ import print_function
import run_vicalib
import run_vicalib_evaluation
import glob
import argparse
from collections import namedtuple
import os
import csv
import multiprocessing as mp
import numpy as np
import tempfile
import shutil


def processer_process(args_in, q):
    dataset = args_in[0]
    args = args_in[1]
    #temp_out_dir = tempfile.mkdtemp()

    input_data_folder = os.path.join(args.input_data_collection_folder, dataset)

    datasets1 = glob.glob(os.path.join(input_data_folder, "**", "vicalibOutput"))
    datasets1 = [os.path.dirname(i) for i in datasets1]
    datasets2 = glob.glob(os.path.join(input_data_folder, "**", "FishEyeImu"))
    datasets2 = [os.path.dirname(i) for i in datasets2]
    datasets3 = glob.glob(os.path.join(input_data_folder, "**", "calibration.json"))
    datasets3 = [os.path.dirname(i) for i in datasets3]

    datasets = list(set(datasets1) & set(datasets2) & set(datasets3))

    if len(datasets) > 0 :
        #process existing calibration results
        Args = namedtuple('Args', ['input_data_folder', 'input_json', 'vicalib_path', 'output_json', 'process_existing'])
        dir_path = datasets[0]

        run_vicalib_args = Args(input_data_folder=dir_path, input_json="", vicalib_path=args.vicalib_path, output_json=os.path.join(dir_path, "calibration.json"), process_existing=True)
        results = run_vicalib.main(run_vicalib_args)

        #calibrate validation dataset
        Args = namedtuple('Args', ['input_data_folder', 'input_json', 'vicalib_path', 'output_json', 'process_existing'])
        run_vicalib_args = Args(input_data_folder=os.path.join(dir_path, "FishEyeImu"), input_json=os.path.join(dir_path, "calibration.json"), vicalib_path=args.vicalib_path, output_json=(dataset + ".json"), process_existing=False)
        results.extend(run_vicalib.main(run_vicalib_args))

        #if os.path.exists(os.path.join(os.path.dirname(dataset),"..","FishEyeImu")):
        #    run_vicalib_args2 = Args(input_data_folder = os.path.join(os.path.dirname(dataset),"..","FishEyeImu"), input_json = run_vicalib_args.output_json, vicalib_path=args.vicalib_path, output_json=None)
        #    results.extend(run_vicalib_evaluation.main(run_vicalib_args2))
        #else:
        #    results.extend([float('nan'),float('nan'),float('nan')])

        #shutil.rmtree(temp_out_dir)
        results.append(os.path.relpath(dir_path,args.input_data_collection_folder).replace(os.sep,"/"))
        q.put(results)

def writer_process(csvfilename, q):
    csvfile = open(csvfilename, 'w', newline='')
    writer = csv.writer(csvfile)
    while True:
        results = q.get()
        if isinstance(results, str) and results == 'done':
            break
        writer.writerow(results)
        csvfile.flush()
    csvfile.close()


def main(args):

    if args.camera_list is None:
        datasets = glob.glob(os.path.join(args.input_data_collection_folder, "**", "vicalibOutput"), recursive=True)
    else:
        datasets = args.camera_list.split(',')
    datasets = sorted(datasets)

    csvfilename = 'vicalib-benchmark-'+os.getenv('BRANCH_NAME', "")+'-'+os.getenv('GIT_COMMIT', "")+'.csv'

    manager = mp.Manager()
    q = manager.Queue()
    pool = mp.Pool(mp.cpu_count())#)4 + 1) # 4 instances in parallel on the tester and one process for python

    writer = pool.apply_async(writer_process, (csvfilename, q))

    jobs = []
    for dataset in datasets:
        job = pool.apply_async(processer_process, ([dataset, args], q))
        jobs.append(job)

    for job in jobs:
        job.get()

    q.put('done')
    pool.close()


    with open(csvfilename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        sortedcsv = sorted(reader, key=lambda row: row[-1])

    with open(csvfilename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for row in sortedcsv:
            writer.writerow(row)
            csvfile.flush()


    #create summary file of stats for reprojection error
    result_numbers = np.array(sortedcsv)[:,0:-1].astype(float)

    #result_numbers = result_numbers[result_numbers[:,-4] > 400,:]

    stats = np.vstack([np.nanpercentile(result_numbers,[0,25,50,75,100], axis=0),np.nanmean(result_numbers, axis=0),np.nanstd(result_numbers, axis=0)])

    summary_filename = 'vicalib-benchmark-summary-'+os.getenv('BRANCH_NAME', "")+'-'+os.getenv('GIT_COMMIT', "")+'.csv'
    np.savetxt(summary_filename, stats, delimiter=",", fmt="%11.7f")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='''
    Vicalib benchmark
    ''')

    parser.add_argument('input_data_collection_folder', help='input data collection folder')
    parser.add_argument('vicalib_path', help='path to vcalib executable',  default=r"./vicalib")
    parser.add_argument('-c', '--camera_list', help="Comma delimited list of serial numbers",  default=None)

    args = parser.parse_args()

    main(args)
