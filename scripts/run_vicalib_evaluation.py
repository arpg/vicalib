import tempfile
import subprocess
import glob
import os
import re
import argparse
import csv
import shutil
import dateutil


def main(args):

    temp_out_dir = tempfile.mkdtemp()

    command_str = [args.vicalib_path,
                   "-grid_spacing", "0.032",         #grid spacing for large target in the tester
                   "-grid_large_rad", "0.0104",      #grid large cirle size for large target in the tester
                   "-grid_small_rad", "0.0065",      #grid small cirle size spacing for large target in the tester
                   "-pattern_preset_only", "large",  #name of the large target in the tester
                   "-output_json", os.path.join(temp_out_dir, "camera.json"),  # Set to avoid extra output in current directory
                   "-output", os.path.join(temp_out_dir, "camera.xml"),        # Set to avoid extra output in current directory
                   "-input_json", args.input_json,
                   "-log_dir", temp_out_dir,
                   "-evaluate_only=True",
                   "-has_initial_guess",
                   "-imu", "csv://" + args.input_data_folder, #path to the imu data used for processing
                   "-cam", "file://" + args.input_data_folder + "/[vifisheye_0_,vifisheye_1_]*.pgm"]  #path the images used for processing
    try:
        subprocess.check_output(command_str, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as e:
        print(e.output)

    log_files = glob.glob(os.path.join(temp_out_dir,"*INFO.*-*.*"))

    res = [float('nan'), float('nan'), float('nan')]
    if len(log_files) == 1:
        result_file  = log_files[0]
        with open(result_file,"r") as f3:
            lines = f3.read()

            timestamps = re.findall("([0-9]+:[0-9]+:[0-9]+\.[0-9]+)", lines)

            duration = (dateutil.parser.parse(timestamps[-1]) - dateutil.parser.parse(timestamps[0])).total_seconds()

            fisheye0_strs = re.findall("Reprojection error for camera 0: .* rmse: (.+)", lines)
            if len(fisheye0_strs) == 1:
                fisheye0 = float(fisheye0_strs[0])
            else:
                fisheye0 = float('nan')

            fisheye1_strs = re.findall("Reprojection error for camera 1: .* rmse: (.+)", lines)
            if len(fisheye1_strs) == 1:
                fisheye1 = float(fisheye1_strs[0])
            else:
                fisheye1 = float('nan')

            imucost_strs = re.findall("Imu cost : .* rmse: (.+)", lines)
            if len(imucost_strs) == 1:
                imucost = float(imucost_strs[0])
            else:
                imucost = float('nan')

        res = [fisheye0, fisheye1, imucost]
    shutil.rmtree(temp_out_dir)

    return res





if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='''
    Run vicalib in evaluation mode
    ''')

    parser.add_argument('input_json', help='input json file in multiple camera format')
    parser.add_argument('input_data_folder', help='input data folder')
    parser.add_argument('output_csv', help='output csv file')
    parser.add_argument('vicalib_path', help='path to vcalib executable',  default=r"./vicalib")

    args = parser.parse_args()
    res = main(args)

    with open(args.output_csv,"w") as outFile:
        csvWriter = csv.writer(outFile, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL, lineterminator='\n')
        csvWriter.writerow(['reprojection error 0', 'reprojection error 1', 'imu cost'])
        csvWriter.writerow(res)
        outFile.flush()